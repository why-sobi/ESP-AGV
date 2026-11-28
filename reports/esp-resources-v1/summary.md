# ESP resource usage (v1)

Firmware as deployed (`models/esp-conv-v2`, `models/esp-gru-v2`) plus the `U`
resource query, ESP-IDF 6.0.1, 160 MHz, ESP32-S3 N16R8. KB = 1000 bytes, MiB = 2^20 bytes.

## Flash and static RAM (`idf.py size`, `idf.py size-components`)

| | Used | Total | Main users |
|---|---|---|---|
| App image | 943 KB (30%) | 3 MiB `factory` partition | WiFi + lwIP 371 KB, TLS/HTTPS (mbedTLS, certificate bundle, HTTP client) 252 KB, our code 48 KB (29.5 KB of it model weights), esp-dsp < 1 KB, rest of ESP-IDF/libc 164 KB |
| Flash chip | 3.07 MiB partitioned | 16 MiB | 12.9 MiB unused |
| Static internal RAM (DIRAM) | 162 KB (47%) | 342 KB | our code 63 KB (detectors 55 KB: motor conv 24.6, IR conv 24.6, GRU 5.0, stuck rule 0.9), WiFi 46 KB |
| IRAM | 16.4 KB (100%) | 16.4 KB | ESP-IDF interrupt and flash-cache code (fixed size) |
| PSRAM | 0 | 8 MiB | not enabled (`CONFIG_SPIRAM` off) |

Each conv detector keeps a RAM copy of its weights sized for the larger
model (`ConvWeights`), so the motor conv buffer is mostly unused.

## Runtime (`data/esp_runs/resources-v1`, 8 live round trips, 193 `U` queries)

Healthy A-D (4/4 completed) and motor weak right 30% A-D (4/4 fault stop,
4/4 notified). Queried every 1 s of simulation time between frames.

| | Value |
|---|---|
| Heap (internal) | 301 KB total; 204-205 KB free while driving (WiFi connected) |
| Lowest free heap since boot | 159 KB, during the HTTPS notification (TLS takes ~46 KB) |
| Largest free block (lowest) | 123 KB |
| Core 1 (control) | `serial_control` 2.31 ms CPU per 32 ms frame (max 2.45), **7.2% at real time**; the detectors are 0.81 ms of it, the rest is serial parsing/formatting and control |
| Core 0 (WiFi) | 1.6% mean load (max 4.9%), WiFi task 1.3% |

Webots runs in fast mode (frames faster than real time), so the measured
core-1 load (11.6%) is converted to CPU time per frame and then to a 32 ms
frame. The notification is sent after the route ends, outside the CPU
windows; its heap use shows in the lowest-since-boot figure.

| Task | Core | Stack | Peak used |
|---|---|---|---|
| serial_control | 1 | 12,288 B | 7,776 B |
| notify | 0 | 8,192 B | 4,448 B |
| tcpip | any | 3,072 B | 1,200 B |
| esp_timer | 0 | 3,584 B | 460 B |
| wifi | 0 | (library) | 4,204 B free |

Reproduce: set `AGV_ESP_RESOURCE_LOG_S=1`, run
`python -m scripts.faults.collect_fault_runs --matrix data\esp_runs\resources-v1\fault-matrix.json --out <dir> --timeout 300`,
then `python -m scripts.esp.resource_usage --runs <dir> --out reports\esp-resources-v1 --port COM3`
(`summary.json`).
