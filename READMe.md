# Funkwetter (NRW) – VHF/UHF/HF “Radio Weather” Display

A small ESP8266 project that estimates **VHF/UHF ducting quality** (2 m, 70 cm) and **10 m band openness**, shows it on an **SSD1309 128×64 OLED**, and serves a **web UI** to tweak inputs and see raw/derived values.  
It can run purely on **local sensors** *or* enrich results with **solar activity APIs** (NOAA/SWPC). A **second ESP8266** with **BMP280 + DHT22** can act as a remote sensor publisher.

---

## Features

- **Bands**: 2 m, 70 cm (from tropospheric refractivity gradient), 10 m (on-air proxy or solar fallback)
- **Display**: 4 progress bars (2 m / 70 cm / 10 m / Solar), header icons (Wi-Fi + sun/moon), bottom **ticker**
- **Overlay**: periodic **RAW sensor overlay** (configurable: enable, interval, duration)
- **Web UI** (`/local`): enter/measured values, scaling (0–5 or 1–10), today override, solar/flare toggles, OLED contrast, remote host settings, overlay timing
- **API** (`/json`): machine-readable summary
- **Persistence**: all settings in EEPROM
- **Time**: NTP + local sunrise/sunset and day/night handling
- **mDNS**: reachable as `http://funkwetter.local/`

---

## What you need

### Main node (Display + Web)

- **ESP8266 NodeMCU (ESP-12E)**
- **SSD1309 128×64 OLED (SPI, 4-wire)**  
  Pins used in the sketch:
  - CS → **D1 (GPIO5)**
  - DC → **D2 (GPIO4)**
  - RST → **D0 (GPIO16)**
  - SCK → **D5 (GPIO14)**
  - MOSI → **D7 (GPIO13)**
  - VCC → 3V3, GND → GND
- Optional local sensors (if you later want direct measurements):
  - Pressure/temperature (e.g., BMP280/BME280)
  - Humidity (e.g., DHT22 or from BME280)
  - A second height level (barometric/temperature/humidity) if available

### Remote sensor node (optional)

- **ESP8266 NodeMCU**
- **BMP280 (I²C)**  
  - VCC → 3V3, GND → GND  
  - SCL → **D1 (GPIO5)**, SDA → **D2 (GPIO4)**  
  - I²C Address: typically **0x76** (SDO→GND) / **0x77** (SDO→3V3)
- **DHT22**  
  - VCC → 3V3, GND → GND, DATA → **D6 (GPIO12)**
- Publishes `http://<host>/sensors.json` with:  
  `{"P_hPa":1018.3,"T_C":22.4,"RH_pct":50.0,"z_m":120.0}`

---

## Software & Libraries

- **Arduino IDE** with **ESP8266 core** installed via Boards Manager
- **U8g2** (for SSD1309 OLED)
- (Remote node) **Adafruit BMP280** (or any BMP280 lib) + **DHT** library

> On the main node we use the ESP8266 core’s BearSSL for HTTPS calls (SWPC APIs).

---

## How it works (math & logic)

### Refractivity and tropospheric ducting (2 m / 70 cm)

1. From pressure \(P\) [hPa], temperature \(T\) [°C], and relative humidity RH [%], compute **radio refractivity** \(N\):

\[
e = RH/100 \cdot E_s(T),\quad E_s(T)=6.112\cdot\exp\left(\frac{17.62T}{243.12+T}\right)
\]
\[
N = 77.6\frac{P}{T_K} + 3.73\cdot10^5\frac{e}{T_K^2},\quad T_K=T+273.15
\]

2. With two heights \(z_1, z_2\), compute **gradient**:

\[
\frac{dN}{dz} = \frac{N_2 - N_1}{z_2 - z_1}\cdot 1000 \quad [\text{N/km}]
\]

3. Heuristics for ducting:
- **Super-refraction/ducting** likely if \(dN/dz < -157\) N/km → “Tropo possible”
- **Enhanced range** if \(-157 \le dN/dz < -79\)
- Else **normal**

4. Map to **scores** (configurable 1–10 or 0–5):
   - Construct a normalized score from \(dN/dz\) and scale slightly differently for 2 m vs 70 cm (UHF a tad more sensitive).
   - If you only have one height, a fallback **heuristic** uses pressure anomaly, night factor, and RH proximity to ~60 %.

### 10 m band

- **Primary (on-air proxy)** if provided:
  - **Spots/15 min**, **max DX distance**, **median SNR** → weighted blend → score
- **Fallback** if not provided:
  - Use **F10.7 solar flux** and **Kp** (geomagnetic) + **time-of-day** + **season (equinox proximity)** to estimate MUF trends → score
- Optional **solar adjustment**: very high solar score nudges +2/+1; high Kp (>5) subtracts 1.

### Solar activity & flares

- **Solar score (SOL)** from **F10.7** (↑ is better) and **Kp** (↓ is better).
- **Flares (GOES X-ray)**: if a C/M/X flare is current, apply a **penalty** primarily to 10 m (especially **daytime**). Values and class are shown.

### Disturbance probability (%)

- Combines **Kp** and any active **flare penalty** (weighted); reported as **low / medium / high**.

---

## Web UI and endpoints

- `http://funkwetter.local/` → redirects to `/local`
- **`/local`** (HTML):
  - Enter or edit: P/T/RH/z (two heights), 10 m on-air proxy (spots/dx/snr)
  - Toggles: Solar, Solar-adjust-10m, Flares (day-only option), VHF heuristic, Splash
  - Scale: 1–10 or 0–5
  - Today override: manually set VHF score (e.g., 8/10 for a great day)
  - OLED contrast slider
  - **Overlay settings**: enable, interval (s), duration (ms)
  - **Remote sensor**: enable + host (e.g., `funk-remote.local`)
- **`/json`** (machine readable): all inputs + scores + labels + flags
- **`/reset`**: factory default (and clears EEPROM state)

---

## Display UX

- **Top row**: “Funkwetter”, Wi-Fi icon, sun/moon (day/night), clock (HH:MM)
- **Bars**: 2 m, 70 cm, 10 m, Solar (with a soft animated “shine”)
- **Ticker** (bottom line): readable ASCII text with **band scores + labels**, solar values, disturbance %, **H1/H2** raw readings, **N1/N2/dN/dz/k**, 10 m proxy/fallback info, and remote status.

**Overlay screen** (periodic, configurable): a compact **RAW/Sensor** view while the ticker continues to scroll.

---

## Build & Flash

1. **Wire the OLED (SPI)** to the ESP8266 as shown above.
2. Install **ESP8266 core** in Arduino IDE; install **U8g2** library.
3. Open the main sketch and set:
   - `WIFI_SSID`, `WIFI_PASS`, `HOSTNAME`
   - Your coordinates: `LAT`, `LON` (used for sunrise/sunset)
   - Optionally change default overlay timing, score scale, etc.
4. **Board**: *NodeMCU 1.0 (ESP-12E Module)*, CPU 80 MHz (default fine)
5. **Flash** the sketch. On serial monitor you should see IP and mDNS info.
6. Open **`http://funkwetter.local/`** (or the IP) to configure values.

### Remote sensor node

1. Wire **BMP280 (I²C)** and **DHT22** as above.
2. Flash a minimal sketch that:
   - Reads BMP280 (P, T) and DHT22 (RH), and a configured site height `z_m`
   - Serves JSON at `/sensors.json`  
   Example payload:
   ```json
   {
     "P_hPa": 1019.0,
     "T_C": 22.0,
     "RH_pct": 51.0,
     "z_m": 120.0
   }
   ```
3. In the **main node** web UI, enable **Remote** and set the host (e.g., `funk-remote.local`). The main node will poll every 60 s with two retries.

---

## Typical workflow

1. Power the main node. Splash screen runs (can be disabled).
2. Time sync (NTP), sunrise/sunset computed; first API fetch (if enabled).
3. Enter local values in `/local` **or** enable the remote sensor.
4. Watch the OLED bars and the ticker.  
   Use **today override** if you know “today is great” beyond the model.

---

## Interpreting results

- **2 m / 70 cm**:  
  - **9–10**: “Tropo possible” – expect enhanced long-haul paths  
  - **7–8**: “conditionally possible” – some enhancement windows  
  - **4–6**: “slightly elevated” – modest improvement  
  - **1–3** (or 0–2 in coarse scale): “normal”
- **10 m**:  
  - **9–10**: “Band open”  
  - **7–8**: “partly open”  
  - **4–6**: “moderate”  
  - **1–3**: “attenuated/closed”
- **Solar**: higher is better (high F10.7, low Kp).  
- **Disturbance %**: operational risk of **HF degradation**; very high Kp or strong flares push this up.

---

## Sensor placement tips

- **Two-level readings** (best): separate your two pressure/temperature/humidity sensors by **≥ 50 m vertical** if possible. If not, any measurable height difference helps; even **10–15 m** can give useful signals with stable sensors.
- **Shelter** from direct sun/rain; ensure airflow; avoid heat sources (roofs).
- **DHT22** can drift; BME280 or SHT sensors are more stable for RH.
- If only one level is available, the project’s **heuristic** still gives a reasonable VHF/UHF estimate.

---

## Troubleshooting

- **Blank OLED**: verify SPI pins, 3V3 power, use **`U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI`** constructor (as in the sketch). Try different contrast (170–210).
- **Flicker on Solar bar**: the sketch uses a **non-XOR shine** animation at ~25 FPS — should be smooth.
- **mDNS not resolving**: open via IP (shown on serial). Check that your network supports mDNS / multicast.
- **Remote sensor “ERR”**: confirm you can open `http://<host>/sensors.json` in a browser. The main node retries twice and polls every 60 s.
- **Time shows “unsynced”**: ensure Internet/NTP reachable.
- **API slow/missing**: the main node throttles solar API fetches (10 min) and flare checks (5 min). If requests fail twice, it skips until the next interval.

---

## Security / Privacy

- Web server is **unauthenticated HTTP** on your LAN; don’t expose it to the internet.
- API calls go to NOAA/SWPC over **HTTPS**.

---

## License

Do whatever you like for personal/hobby use. If you publish or fork, a small credit mention “Funkwetter ESP8266/SSD1309” is appreciated.

---

## Credits

- **U8g2** by olikraus (OLED font & graphics)
- **ESP8266 Arduino Core**
- **NOAA/SWPC** public data (F10.7, Kp, GOES flares)

---

**Happy DX!** If you want a ready-to-print quickstart card (wiring + setup), I can make one.
