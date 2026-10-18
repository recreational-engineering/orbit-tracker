#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include "net.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "model.h"
#include "orbit.h"
#include "generated/elements.h"
#include "net_secret.h"
#include "esp_heap_caps.h"
#include "nvs.h"

// Known networks from net_secret.h (older single-network files still work).
typedef struct { const char *ssid, *pass; } known_net_t;
#ifdef NET_WIFI_NETWORKS
static const known_net_t KNOWN[] = NET_WIFI_NETWORKS;
#else
static const known_net_t KNOWN[] = {{NET_WIFI_SSID, NET_WIFI_PASS}};
#endif
#define N_KNOWN (int)(sizeof KNOWN / sizeof KNOWN[0])

static const char *TAG = "net";

// The one remote host the device ever contacts for elements. Time comes from NTP.
#define CELESTRAK_HOST "celestrak.org"
#define NTP_SERVER     "pool.ntp.org"
// A commonplace browser User-Agent, so the request carries nothing that identifies the device.
#define HTTP_UA "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 " \
                "(KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36"

// CelesTrak publishes new elements a few times a day and blocks clients that ask more often, so
// the GET is skipped when the last successful one is more recent than this (reboots and the `N`
// command included). The time of that fetch is kept in NVS so it survives a reboot.
#define ELEMENTS_MIN_AGE_S (20 * 3600)
#define FETCH_NVS_NS  "orbit"
#define FETCH_NVS_KEY "omm_at"

#define CONNECTED_BIT BIT0
#define FAIL_BIT      BIT1

static EventGroupHandle_t wifi_events;
static volatile bool busy;
static volatile bool inited;
static int retries;

bool net_busy(void) { return busy; }

static void on_wifi(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    // (STA_START: nothing; net_sync picks the network first, then connects.)
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retries < 5 && busy) { retries++; esp_wifi_connect(); }
        else xEventGroupSetBits(wifi_events, FAIL_BIT);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        retries = 0;
        xEventGroupSetBits(wifi_events, CONNECTED_BIT);
    }
}

void net_init(void)
{
    // The Wi-Fi driver is initialised once here (robust; repeated init/deinit is not). The radio
    // is only started during a sync and stopped afterwards, so no connection is held in between.
    wifi_events = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t le = esp_event_loop_create_default();   // tolerate a loop created elsewhere
    if (le != ESP_OK && le != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(le);
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    // The only identifier the device exposes: a nondescript host name, no vendor hints.
    esp_netif_set_hostname(sta, NET_HOSTNAME);
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi, NULL, NULL));

    inited = true;
    ESP_LOGI(TAG, "network ready (Wi-Fi driver started on first sync); host name \"%s\"", NET_HOSTNAME);
}

// Start the radio. The driver is initialised once, on the first sync (so no Wi-Fi RAM is held
// at boot), and then kept initialised (repeated init/deinit is not reliable); later syncs only
// start and stop the radio.
static bool wifi_started_once;
static bool wifi_up(void)
{
    if (!wifi_started_once) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        if (esp_wifi_init(&cfg) != ESP_OK) return false;
        esp_wifi_set_storage(WIFI_STORAGE_RAM);                // keep credentials out of flash
        esp_wifi_set_mode(WIFI_MODE_STA);                      // station only, never an access point
        wifi_started_once = true;
    }
    return esp_wifi_start() == ESP_OK;
}

// Which known network is in range: a passive scan (the radio only listens, so the names of
// networks that are not here are never broadcast). Returns the first match in order of
// preference, or -1.
static int pick_network(void)
{
    if (N_KNOWN == 1) return 0;   // one network: nothing to choose
    wifi_scan_config_t sc = {
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_PASSIVE,
        .scan_time.passive = 150,   // ms per channel
    };
    if (esp_wifi_scan_start(&sc, true) != ESP_OK) return -1;
    uint16_t n = 0;
    esp_wifi_scan_get_ap_num(&n);
    if (n > 32) n = 32;
    wifi_ap_record_t *aps = heap_caps_calloc(n ? n : 1, sizeof *aps, MALLOC_CAP_SPIRAM);
    if (!aps) { esp_wifi_clear_ap_list(); return -1; }
    esp_wifi_scan_get_ap_records(&n, aps);
    esp_wifi_clear_ap_list();
    int pick = -1;
    for (int k = 0; k < N_KNOWN && pick < 0; k++)
        for (int i = 0; i < n; i++)
            if (strcmp((const char *)aps[i].ssid, KNOWN[k].ssid) == 0) { pick = k; break; }
    free(aps);
    return pick;
}

static bool wifi_join(int k)
{
    wifi_config_t wc = {0};
    strlcpy((char *)wc.sta.ssid, KNOWN[k].ssid, sizeof wc.sta.ssid);
    strlcpy((char *)wc.sta.password, KNOWN[k].pass, sizeof wc.sta.password);
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wc.sta.pmf_cfg.capable = true;
    return esp_wifi_set_config(WIFI_IF_STA, &wc) == ESP_OK && esp_wifi_connect() == ESP_OK;
}

// Stop the radio (no connection held between syncs); the driver stays initialised.
static void wifi_down(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
}

// ---------- SNTP ----------
static bool sync_time(void)
{
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(NTP_SERVER);
    cfg.sync_cb = NULL;
    esp_netif_sntp_init(&cfg);
    bool ok = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) == ESP_OK;
    esp_netif_sntp_deinit();
    if (ok) {
        model_request((double)time(NULL));   // recompute for the corrected clock
        ESP_LOGI(TAG, "time synced: %lld", (long long)time(NULL));
    } else {
        ESP_LOGW(TAG, "time sync timed out");
    }
    return ok;
}

// ---------- element refresh (HTTP GET to CelesTrak, at most once per ELEMENTS_MIN_AGE_S) ----------
typedef struct { char *buf; int len, cap; } body_t;

static esp_err_t on_http(esp_http_client_event_t *e)
{
    if (e->event_id != HTTP_EVENT_ON_DATA) return ESP_OK;
    body_t *b = e->user_data;
    if (b->len + e->data_len + 1 > b->cap) {
        int cap = b->cap ? b->cap * 2 : 4096;
        while (cap < b->len + e->data_len + 1) cap *= 2;
        if (cap > 64 * 1024) return ESP_FAIL;   // the element set is a few hundred bytes
        char *n = realloc(b->buf, cap);
        if (!n) return ESP_FAIL;
        b->buf = n; b->cap = cap;
    }
    memcpy(b->buf + b->len, e->data, e->data_len);
    b->len += e->data_len;
    b->buf[b->len] = 0;
    return ESP_OK;
}

static double epoch_to_unix(const char *s)
{
    int y = 0, mo = 0, d = 0, h = 0, mi = 0;
    double sec = 0;
    if (sscanf(s, "%d-%d-%dT%d:%d:%lf", &y, &mo, &d, &h, &mi, &sec) != 6) return 0;
    // Days from 1970-01-01 to y-mo-d (civil calendar, proleptic Gregorian).
    int yy = y - (mo <= 2);
    long era = (yy >= 0 ? yy : yy - 399) / 400;
    unsigned yoe = (unsigned)(yy - era * 400);
    unsigned doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    long days = era * 146097L + (long)doe - 719468;
    return (double)days * 86400.0 + h * 3600 + mi * 60 + sec;
}

// The CelesTrak OMM reply is one flat JSON object (in an array) of "KEY": value pairs. A small
// reader keeps the firmware free of a JSON library. find_field returns the text just after the
// colon for a top-level "KEY", or NULL.
static const char *find_field(const char *json, const char *key)
{
    char pat[40];
    int n = snprintf(pat, sizeof pat, "\"%s\"", key);
    if (n <= 0 || n >= (int)sizeof pat) return NULL;
    const char *p = json;
    while ((p = strstr(p, pat))) {
        const char *q = p + n;
        while (*q == ' ' || *q == '\t') q++;
        if (*q == ':') { q++; while (*q == ' ' || *q == '\t') q++; return q; }
        p += n;
    }
    return NULL;
}

static double jnum(const char *json, const char *key)
{
    const char *p = find_field(json, key);
    return p ? strtod(p, NULL) : 0;
}

static bool jstr(const char *json, const char *key, char *out, size_t cap)
{
    const char *p = find_field(json, key);
    if (!p || *p != '"') return false;
    p++;
    size_t i = 0;
    while (*p && *p != '"' && i + 1 < cap) out[i++] = *p++;
    out[i] = 0;
    return *p == '"';
}

static bool parse_and_apply(const char *json)
{
    char epoch[40];
    if (!jstr(json, "EPOCH", epoch, sizeof epoch)) {
        ESP_LOGW(TAG, "elements: no EPOCH (CelesTrak error?)");
        return false;
    }
    omm_t omm = {
        .norad_cat_id = (int)jnum(json, "NORAD_CAT_ID"),
        .epoch_unix = epoch_to_unix(epoch),
        .mean_motion = jnum(json, "MEAN_MOTION"),
        .eccentricity = jnum(json, "ECCENTRICITY"),
        .inclination = jnum(json, "INCLINATION"),
        .ra_of_asc_node = jnum(json, "RA_OF_ASC_NODE"),
        .arg_of_pericenter = jnum(json, "ARG_OF_PERICENTER"),
        .mean_anomaly = jnum(json, "MEAN_ANOMALY"),
        .bstar = jnum(json, "BSTAR"),
        .mean_motion_dot = jnum(json, "MEAN_MOTION_DOT"),
        .mean_motion_ddot = jnum(json, "MEAN_MOTION_DDOT"),
    };
    bool ok = omm.epoch_unix > 0 && model_set_omm(&omm);
    if (!ok) ESP_LOGW(TAG, "elements: NORAD %d not usable for this device (%d)", omm.norad_cat_id, OMM_NORAD_CAT_ID);
    return ok;
}

static int64_t last_fetch(void)
{
    int64_t t = 0;
    nvs_handle_t h;
    if (nvs_open(FETCH_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i64(h, FETCH_NVS_KEY, &t);
        nvs_close(h);
    }
    return t;
}

static void save_fetch(int64_t t)
{
    nvs_handle_t h;
    if (nvs_open(FETCH_NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i64(h, FETCH_NVS_KEY, t);
    nvs_commit(h);
    nvs_close(h);
}

// True when the last successful fetch is recent enough to skip this one. With an unset clock
// (time sync failed, before 2020) or a fetch time in the future, the age is unknown: fetch.
static bool elements_fresh(void)
{
    int64_t now = time(NULL), last = last_fetch();
    if (now < 1577836800 || last <= 0 || last > now) return false;
    if (now - last >= ELEMENTS_MIN_AGE_S) return false;
    ESP_LOGI(TAG, "elements: fetched %lld min ago, next fetch in %lld min",
             (long long)(now - last) / 60, (long long)(ELEMENTS_MIN_AGE_S - (now - last)) / 60);
    return true;
}

static bool refresh_elements(void)
{
    if (elements_fresh()) return true;
    // Fetched over plain HTTP: the response is public orbital data and the request reveals only
    // the (public) NORAD id, which TLS would expose via SNI anyway. TLS to this host does not
    // complete over the Wi-Fi co-processor link, so HTTP is what works here.
    char url[160];
    snprintf(url, sizeof url, "http://%s/NORAD/elements/gp.php?CATNR=%d&FORMAT=JSON",
             CELESTRAK_HOST, OMM_NORAD_CAT_ID);
    body_t body = {0};
    esp_http_client_config_t cfg = {
        .url = url,
        .event_handler = on_http,
        .user_data = &body,
        .timeout_ms = 15000,
        .user_agent = HTTP_UA,
    };
    esp_http_client_handle_t cli = esp_http_client_init(&cfg);
    bool ok = false;
    esp_err_t err = esp_http_client_perform(cli);
    if (err == ESP_OK) {
        int code = esp_http_client_get_status_code(cli);
        if (code == 200 && body.buf) ok = parse_and_apply(body.buf);
        if (ok) save_fetch(time(NULL));
        else ESP_LOGW(TAG, "elements: HTTP %d", code);
    } else {
        ESP_LOGW(TAG, "elements: request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(cli);
    free(body.buf);
    return ok;
}

// ---------- one sync window ----------
bool net_sync(net_sync_t what)
{
    if (!inited || busy) return false;
    busy = true;
    retries = 0;
    xEventGroupClearBits(wifi_events, CONNECTED_BIT | FAIL_BIT);
    ESP_LOGI(TAG, "connecting to update%s%s", what.time ? " time" : "", what.elements ? " elements" : "");
    if (!wifi_up()) { busy = false; ESP_LOGW(TAG, "Wi-Fi driver could not start"); return false; }
    int k = pick_network();
    if (k < 0 || !wifi_join(k)) {
        ESP_LOGW(TAG, "no known network in range");
        wifi_down();
        busy = false;
        return false;
    }
    ESP_LOGI(TAG, "joining known network %d of %d", k + 1, N_KNOWN);

    EventBits_t bits = xEventGroupWaitBits(wifi_events, CONNECTED_BIT | FAIL_BIT, pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(20000));
    bool connected = bits & CONNECTED_BIT;
    if (connected) {
        if (what.time) sync_time();
        if (what.elements) refresh_elements();
    } else {
        ESP_LOGW(TAG, "could not join the network");
    }

    // Release the Wi-Fi driver again: no connection is held and its RAM is freed between syncs.
    wifi_down();
    busy = false;
    ESP_LOGI(TAG, "disconnected, Wi-Fi released");
    return connected;
}

void net_sync_now(void *arg)
{
    (void)arg;
    net_sync((net_sync_t){.time = true, .elements = true});
    vTaskDelete(NULL);
}
