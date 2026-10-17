/* The device's only use of the network: read the current time (SNTP) and refresh the saved
 * satellite's orbital elements (one HTTPS GET to CelesTrak for the compiled-in NORAD id).
 *
 * The radio is off except during a sync. The device joins as a station only, presents a
 * nondescript host name (net_secret.h), never starts an access point, a server or any
 * discovery service, and makes no other outbound connection. There is no code path that
 * sends device data anywhere or that lets the network command the device.
 */
#pragma once
#include <stdbool.h>

// What a sync should refresh.
typedef struct { bool time; bool elements; } net_sync_t;

// Prepare Wi-Fi (station, radio stopped). Call once, after nvs_flash_init(). Safe to skip
// calling net_sync afterwards; nothing connects until then.
void net_init(void);

// Bring the radio up, join the network, do the requested refreshes, then stop the radio.
// Blocks (run it from its own task, never the frame loop). Returns true if it connected;
// per-item success is logged. A second call while one is running is ignored.
bool net_sync(net_sync_t what);

// Task entry for a one-off manual sync (time + elements): xTaskCreate(net_sync_now, ...).
// It syncs once and deletes itself.
void net_sync_now(void *arg);

// True while a sync is in progress.
bool net_busy(void);
