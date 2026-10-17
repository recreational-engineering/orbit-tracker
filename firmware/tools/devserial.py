"""Open the board's console without toggling DTR/RTS (which would reset it)."""
import glob

import serial

BAUD = 2_000_000


def default_port():
    # The board's CH343 USB-serial bridge (WCH, 1a86); other USB serial devices (monitors,
    # docks) may be plugged in too.
    from serial.tools import list_ports
    ports = sorted(p.device for p in list_ports.comports() if p.vid == 0x1A86 and p.device.startswith("/dev/cu."))
    if not ports:
        ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        raise SystemExit("no serial port found; pass --port")
    return ports[0]


def open_console(port=None, timeout=1.0, reset=False):
    s = serial.Serial()
    s.port = port or default_port()
    s.baudrate = BAUD
    s.timeout = timeout
    s.open()
    if reset:
        s.dtr = False   # RTS alone now: EN low until RTS is released below
    else:
        # The auto-reset circuit pulls EN low while RTS is asserted without DTR, so release
        # RTS before DTR (pyserial's own order on open is DTR first, which resets the board).
        s.rts = False
        s.dtr = False
    if reset:
        # Restart the board and wait until it draws frames (the same line order as the
        # first open: RTS alone holds EN low).
        import time
        s.rts = True
        time.sleep(0.1)
        s.rts = False
        end, seen = time.time() + 15, b""
        while time.time() < end and b"STATS" not in seen and b"DBG" not in seen:
            seen = (seen + s.read(65536))[-4096:]
    return s
