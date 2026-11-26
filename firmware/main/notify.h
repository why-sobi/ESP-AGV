#pragma once

#include <cstddef>

// Push notifications over WiFi through ntfy (https://ntfy.sh by default).
//
// The WiFi network and ntfy topic are read at boot from the "wificfg" NVS
// partition, which scripts/esp/flash_wifi.ps1 writes from firmware/wifi.csv
// without rebuilding the firmware. Without that configuration WiFi stays off
// and notify_send() returns false; the controller works as before.
//
// Sending never blocks the caller: messages are queued and posted by a
// low-priority task on core 0, which waits for WiFi and retries a few times.

void notify_start();

// Queue one notification. Returns false if WiFi is not configured or the
// queue is full.
bool notify_send(const char *title, const char *message);

// One-line status for the serial 'W' query:
// "<OFF|CONNECTING|CONNECTED>,<ip or ->,<last result>".
void notify_status(char *out, size_t size);
