[English](README.md) · **繁體中文**

# 智慧寵物門 🐾

<img width="1917" height="1078" alt="image" src="https://github.com/user-attachments/assets/cc415e5f-6828-416f-af6a-ca0d08f0b3f9" />


一款具備隔熱設計、電子驅動的寵物門，僅在確認偵測到動物時開啟 — 以完全密封的馬達驅動門扇取代傳統被動式活動門片所造成的永久性熱橋。

> 傳統活動門片會在建築外殼留下永久縫隙，每年可能使空調與暖通（HVAC）耗能增加 6% 以上。本寵物門在待機時會完全密封開口，僅在寵物確實靠近時開啟，並在通過後立即關閉重新密封。

以 **ESP32-C6** 為核心建置，配備雙毫米波雷達（mmWave radar）存在感測、飛行時間（Time-of-Flight, ToF）防夾安全光幕，以及兩顆由靜音 TMC2130 驅動的步進馬達。透過 **Zigbee**（Door Lock 與 Occupancy clusters）整合智慧家庭系統，並以相同硬體支援 Matter-over-Thread 作為後續升級路線。

---

## 目錄

- [運作原理](#運作原理)
- [功能特點](#功能特點)
- [硬體規格](#硬體規格)
- [GPIO 腳位對應表](#gpio-腳位對應表)
- [儲存庫結構](#儲存庫結構)
- [建置與燒錄](#建置與燒錄)
- [參數設定](#參數設定)
- [安全系統](#安全系統)
- [進出模式與連線能力](#進出模式與連線能力)
- [開發路線圖](#開發路線圖)
- [作者](#作者)

---

## 運作原理

一對 **24 GHz 毫米波雷達** 分別監控門檻的兩側：

- **內側 — LD2410** (UART0)：存在感測／狀態回報。
- **外側 — LD2410** (UART1)：存在感測／狀態回報。

當雷達偵測到目標接近（且目前的進出模式允許該方向通行時），控制器便會將門扇推開。當感測區域淨空並經過一段保持延遲後，門扇隨即關閉。門洞處的 **VL53L1X ToF 感測器** 構成一道隱形安全光幕 — 關閉過程中的任何障礙物都會使門扇反轉開啟（防夾機制）— 同時兼作門洞中央的「穿越（crossing）」事件偵測，用於寵物計數。

> 兩個插槽目前皆使用 **LD2410**。韌體保留了兩種 UART 協定的解析器（LD2450 `AA FF 03 00 … 55 CC`；LD2410 `F4 F3 F2 F1 | len | … | F8 F7 F6 F5`）並可針對每個插槽獨立選擇，因此日後只需修改一行型別設定，即可**直接將內側插槽升級為多目標 LD2450**。

門扇運作採用四狀態狀態機：

```
        radar present (mode-gated)        reached open angle
CLOSED ───────────────────────► OPENING ───────────────────► OPEN
   ▲                                                            │
   │ both leaves shut (end-stop or pos 0)        clear + delay  │
   │                                                            ▼
   └──────────────── CLOSING ◄─────────────────────────────────┘
                        │
       ToF obstruction  │  → reverse → OPENING  (anti-pinch)
```

兩顆 NEMA 17 馬達在鉸鏈處以**背對背**方式安裝，並以鏡像方式驅動（門葉 2 執行門葉 1 的反向目標位置），使雙開門板能同步動作宛如單一開口。每片門葉皆可裝設各自的極限開關（end-stop switch，可獨立啟用），作為關門位置的真值基準。驅動器在靜止時保持啟用，透過保持扭矩防止門被推開，同時以降低的保持電流（`IHOLD ≈ 0.3 × IRUN`）使馬達在靜止時維持低溫與安靜。

### 寵物計數

只有在逾時限制內依序觀察到完整序列時，才會計為一次通過：某一側雷達 → 穿越 ToF 光束 → **另一側**雷達。行進方向決定當前在室內數量的增減：由外至內為 `+1`，由內至外為 `−1`（下限為 0）。即時數量會透過 Zigbee 發布（詳見下文）。計數功能需要啟用 ToF 感測器（`USE_TOF 1`）。

## 功能特點

- **靜音驅動** — TMC2130 StealthChop 徹底消除了 A4988/DRV8825 等級驅動器的明顯高頻噪音。
- **雙存在感測雷達** — 內外兩側各配備一顆 **LD2410**，均具備資料過期防護解析機制。韌體亦保留了 LD2450 解析器，因此可直接將內側通道替換為多目標 LD2450 而無需修改程式碼。
- **預先防夾機制** — ToF 光幕會在接觸*之前*（而非碰撞後）立即停止並反轉門扇。
- **靜止鎖定防推** — 具反向驅動特性的傳動機構由馬達扭矩保持閉合，能抵抗外力推開。
- **獨立門葉極限開關** — 兩組獨立開關（`END_STOP1`/`END_STOP2`），可分別單獨啟用，用於原點復歸（homing）與關門基準真值確認。
- **方向性寵物計數** — 透過「雷達 → ToF → 雷達」通過偵測維持即時在室數量統計。
- **原生智慧家庭整合** — 具備 Zigbee Door Lock（進出模式）、Occupancy Sensing（是否有人/寵物佔用）與 Analog Input（寵物計數）clusters；建置時可自由選擇無線電設定檔（radio profile）。

## 硬體規格

| 子系統 | 零件 | 備註 |
|---|---|---|
| MCU | **ESP32-C6** DevKitM-1 | RISC-V, Wi-Fi 6, BLE 5, 802.15.4 (Zigbee/Thread), 支援 Matter |
| 馬達驅動器 | **TMC2130** ×2 | StealthChop（靜音）、SPI 設定、1/16 微步進（µstep） |
| 馬達 | **NEMA 17** ×2 | 1.8°/步、0.3–0.6 Nm、鉸鏈直接驅動、背對背安裝 |
| 存在感測 | **LD2410** ×2 | 24 GHz FMCW, UART @256000, ~5 m，內外各一（內側 + 外側）。若裝配 LD2450 韌體亦能解析 |
| 安全防護 | **VL53L1X** ToF ×1 | I²C, 940 nm（不受雷達干擾），門框向下朝向安裝 |
| 電源 | 12 V SMPS → **LM2596** 降壓模組 → 5 V | 12 V 供驅動器，5 V 供 MCU 與感測器；每個驅動器配備 1000 µF 反電動勢濾波電容 |
| 門板 | 鋼板貼面 **20 mm PIR 發泡板** + 橡膠密封條 | R ≈ 0.87 m²·K/W；具備周邊密封的重疊式雙開門板 |

完整的物料清單（BOM）以及各項零件選型的設計理念均收錄於專案文件中。

## GPIO 腳位對應表

依照 [`src/main.cpp`](src/main.cpp) 中針對 ESP32-C6 的接線設定（與電路板接線表一致）：

| 功能 | GPIO | 匯流排 | 連接至 |
|---|---|---|---|
| SPI SCK / MISO / MOSI | 19 / 12 / 20 | SPI | TMC2130 ×2 |
| 致能 Enable（共用，active-low） | 21 | — | TMC2130 ×2 — EN |
| 驅動器 1 STEP / DIR / CS | 11 / 10 / 18 | SPI/step | 馬達驅動器 1（門葉 1） |
| 驅動器 2 STEP / DIR / CS | 23 / 22 / 3 | SPI/step | 馬達驅動器 2（門葉 2） |
| 極限開關 1 / 極限開關 2 (End-stop 1 / 2) | 13 / 2 | — | 各門葉關閉開關（`USE_END_STOP1/2`） |
| ToF SDA / SCL / INT | 6 / 7 / 0 | I²C | VL53L1X |
| 雷達 1 內側 (RX / TX) | 17 / 16 | UART0 | LD2410 |
| 雷達 2 外側 (RX / TX) | 4 / 5 | UART1 | LD2410 |
| 狀態 NeoPixel | 8 | — | 板載 WS2812 |

> **主控台注意事項：** 內側雷達（LD2410）佔用了 **UART0 (GPIO16/17)** — 這與 CH340 USB 序列主控台共用相同腳位 — 且原生 USB-Serial-JTAG 腳位（GPIO12/13）也被複用於 MISO / END_STOP1。因此一旦接上內側雷達，CH340 主控台將無法使用，**即時日誌必須透過 Wi-Fi telnet 傳輸**。CH340 主控台僅在停用內側雷達（`USE_RADAR_IN 0`）時可用。當未連接日誌時，板載狀態 NeoPixel 會以燈號編碼表示目前階段作為備用方案。

## 儲存庫結構

```
.
├── platformio.ini          Three build envs (debug-wifi / deploy-zigbee / debug-both)
├── partitions_zigbee.csv   Single-app (no-OTA) 4MB layout for the Zigbee build
├── src/
│   ├── main.cpp            Firmware — dual-radar door FSM, pet counting, Zigbee
│   └── ToF_Code.ino.ref    Standalone VL53L1X polling reference sketch
├── include/                Project headers
├── lib/                    Private libraries
└── test/                   PlatformIO tests
```

## 建置與燒錄

需要 [PlatformIO](https://platformio.org/)（CLI 或 VS Code 擴充套件）。由於 C6 開發板**未**收錄於官方 `espressif32` 平台（Arduino-ESP32 v2.x）中，因此 `platformio.ini` 指定了 [pioarduino](https://github.com/pioarduino/platform-espressif32) 分支（Arduino-ESP32 v3.x）。相依套件（`TMCStepper`、`AccelStepper`、`Adafruit VL53L1X`）會在首次建置時自動解析。運動控制採用 **AccelStepper**（可在任意 GPIO 上輸出軟體 STEP/DIR 脈衝）：FastAccelStepper 的硬體脈衝產生在 ESP32-C6 上會導致其中一扇門葉失效，而 AccelStepper 則不受週邊通道數限制。TMC2130 仍透過 SPI 進行設定以啟用 StealthChop。

ESP32-C6 同時具備 2.4 GHz Wi-Fi 與 802.15.4 (Zigbee) 無線電。可透過選擇建置環境來挑選 **無線電設定檔（radio profile）** — 各環境會設定對應的 `RADIO_MODE` 旗標：

| 環境 | `RADIO_MODE` | 無線電 | 日誌輸出 | 用途 |
|---|:---:|---|---|---|
| `debug-wifi` *(預設)* | 0 | 僅 Wi-Fi | telnet `:23`（若關閉內側雷達可加用 CH340） | 開發測試／除錯（bring-up / debugging） |
| `deploy-zigbee` | 1 | 僅 Zigbee | USB 序列埠（當內側雷達關閉時） | 正式部署智慧家庭裝置 |
| `debug-both` | 2 | Wi-Fi + Zigbee (共存 coex) | telnet `:23` | 搭配即時日誌除錯 Zigbee（消耗較多 RAM） |

```bash
# Build a profile
pio run -e debug-wifi
pio run -e deploy-zigbee

# Flash + monitor (debug build over CH340, only valid when inside radar disabled)
pio run -e deploy-zigbee -t upload
pio device monitor -b 115200
```

Wi-Fi 建置版本透過 telnet 輸出日誌：連線至 `petdoor.local:23`（或裝置 IP）— 開機累積的日誌會重播給後連線的客戶端。Zigbee 建置版本使用單一應用程式、**無 OTA** 的分割表（`partitions_zigbee.csv`），因為 zboss 協定堆疊大小會使映像檔超出預設雙 OTA `zigbee.csv` 的應用程式分割槽容量。

## 參數設定

位於 [`src/main.cpp`](src/main.cpp) 頂端的主要編譯期選項：

| 巨集 | 預設值 | 用途 |
|---|---|---|
| `RADIO_MODE` | 依環境而定 | `0` Wi-Fi / `1` Zigbee / `2` 兩者皆有 — 由 PlatformIO 環境設定，請勿手動修改 |
| `USE_RADAR_IN` / `USE_RADAR_OUT` | `1` / `1` | 啟用各雷達插槽；停用內側雷達可釋放 UART0 供 CH340 主控台使用 |
| `TEST_TIMED_CYCLE` | 衍生推導 | `(!USE_RADAR_IN && !USE_RADAR_OUT)` — 雙雷達皆關閉 ⇒ 定時開關循環；任一雷達開啟 ⇒ 雷達驅動。無需擔心忘記設定獨立旗標 |
| `MOTOR_SELFTEST` | `0` | `1` = 開機時微動測試（jog）各門葉，記錄命令位置與回報位置（用於釐清門葉不動問題：接線/馬達電源 VM 或脈衝產生問題） |
| `SENSOR_DEBUG` | `0` | `1` = 串流輸出原始 ToF 與雷達讀數（約 4 Hz，包含 `rx`/`good`/`bad` 訊框計數）並跳過門機狀態機 — 用於感測器初測與對焦校準 |
| `USE_END_STOP1` / `USE_END_STOP2` | `0` / `0` | 接線完成後啟用各門葉的關閉極限開關 |
| `USE_TOF` | `0` | 啟用 VL53L1X — 防夾**與**寵物計數皆必需 |
| `OPEN_DEG` | `180` | 門扇開啟角度（度） |
| `RMS_CURRENT` | `800` | 馬達運轉電流（mA, IRUN） |
| `IHOLD_MULT` | `0.3` | 靜止保持電流佔運轉電流之比例 |
| `CLOSE_DELAY_MS` | `3000` | 雷達區域淨空後至開始關門前的保持延遲（毫秒） |
| `TOF_DETECT_MM` | `350` | 判定為障礙物的 ToF 距離閾值（毫米，盲區下限 40 mm） |

初測輔助工具：在停用兩側雷達時，韌體會自動切換為定時開關循環（無需額外設定旗標），讓您在接上雷達前即可先行驗證機構動作、ToF 防夾反轉以及馬達電流／發熱情形。`MOTOR_SELFTEST` 可用於排除門葉不轉的問題（注意：即使沒有 12 V 馬達供電，TMC2130 透過 SPI 仍會回報有效的 `version` — `configDriver` 會針對電荷泵欠壓單獨發出警告，避免將缺少 VM 電源誤認為驅動器正常運作）。`SENSOR_DEBUG` 可即時串流輸出 ToF／雷達數值，方便調整感測角度與確認訊框解析正確性。

## 安全系統

原設計規劃了三層障礙物防護機制；第 1–2 層為預先防護（在接觸前即觸發動作）：

| 層級 | 感測器 | 方法 | 狀態 |
|---|---|---|---|
| 1 | VL53L1X ToF | 關門時輪詢距離 → 反轉開啟 | ✅ 已實作（防夾重新開啟） |
| 2 | 毫米波雷達 (mmWave radar) | 存在感測保持，在有物體佔用時維持開門 | ✅ 已實作 |
| 3 | TMC2130 StallGuard | 透過 SPI 讀回反電動勢堵轉偵測 | ❌ 已放棄 — StallGuard 需要 SpreadCycle 模式，無法與所選的 StealthChop 模式共存；改由 ToF 感測器涵蓋障礙物偵測 |

## 進出模式與連線能力

透過雷達配對所判定的移動方向，強制執行四種可遠端設定的進出模式：

| 方向 \ 模式 | UNLOCKED（解鎖） | IN-ONLY（僅進） | OUT-ONLY（僅出） | LOCKED（上鎖） |
|---|:---:|:---:|:---:|:---:|
| 進入（IN）— 外側雷達 | ✓ | ✓ | ✗ | ✗ |
| 出去（OUT）— 內側雷達 | ✓ | ✗ | ✓ | ✗ |

Zigbee 介面（建置於 `deploy-zigbee` / `debug-both`）公開了四個端點（endpoints）：

- **Door Lock cluster** — 安全／上鎖介面。裝置廣播為 Door Lock，標準 `LockState` 會反映門扇是否處於 `LOCKED` 模式（任何通用控制器皆可讀取）。
- **Multistate Output cluster** — 4 向進出模式 `UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED`（索引 0–3）。屬於標準 cluster，因此智慧家庭閘道器（ZHA / Zigbee2MQTT）能直接將其呈現為可寫入的下拉選單，無需撰寫自訂 quirk。
- **Occupancy Sensing cluster** — `occupied` 旗標（當室內有寵物時為 true）。
- **Analog Input cluster** — 即時寵物計數（目前在室內的寵物數量），在數值變更時主動回報。

這兩種模式介面與韌體的 `doorMode` 保持同步一致：在任一介面設定 `LOCKED` 都會將 Door Lock 切換為 Locked，且任何模式變更都會同步推送至兩者。

### Zigbee 實作筆記（設計考量）

- **透過開發框架直接使用 esp-zigbee-sdk，而非獨立外部相依。** Arduino-ESP32 v3.x（pioarduino 分支）*內建（vendors）* 了樂鑫原廠的 [esp-zigbee-sdk](https://github.com/espressif/esp-zigbee-sdk) — 包含 `esp_zb_*` API 以及預先編譯的 zboss 函式庫。`#include <Zigbee.h>` 是其上層的輕量級 Arduino 包裝層。自訂的門鎖端點直接呼叫 SDK（`esp_zb_door_lock_clusters_create` 等）；`ZIGBEE_MODE_ED` 建置旗標則從框架中連結 zboss。無需在 `lib_deps` 加入條目 — 額外手動加入 SDK 儲存庫會與內建版本發生衝突。
- **為何使用 Multistate 實作模式而非廠商自訂屬性（manufacturer attr）。** 標準 ZCL Door Lock 僅有二元狀態（LockState Locked/Unlocked）— 沒有放置方向性 IN-ONLY/OUT-ONLY 的欄位。最初版本將 4 種模式放在 Door Lock cluster 的*廠商專屬（manufacturer-specific）* 屬性中，但通用閘道器若無自訂轉換器／quirk 就無法識別。改用 **標準 Multistate Output** 則能原生呈現為 4 選項的選單，開箱即可直接控制各種模式。Door Lock cluster 則純粹保留用於安全／上鎖廣播與狀態顯示。
- **自訂 Door Lock 端點；其餘皆使用標準元件。** Arduino 包裝層提供了 Multistate、Occupancy 與 Analog 的類別（皆直接原樣使用），但 **完全沒有提供 Door Lock 類別**，因此 `ZigbeePetDoorLock` 是手動在 cluster `0x0101` 上實作的 `ZigbeeEP` 子類別。
- **非 Zigbee 環境中設定 `lib_ignore = Zigbee`。** 內建的 Zigbee 函式庫預設都會編譯，但 zboss 只有在帶有 `ZIGBEE_MODE_ED` 時才會進行連結；若缺少該旗標，建置會在連結階段失敗（`undefined reference to esp_zb_ep_list_create`）。在 `debug-wifi` 中忽略該函式庫可避免此問題。
- **無 OTA 分割表。** 在 4 MB 快閃記憶體（Flash）上，zboss 映像檔（約 1.39 MB）會超出預設雙 OTA `zigbee.csv` 的應用程式分割槽（每個 1.31 MB）。`partitions_zigbee.csv` 犧牲 OTA 功能以換取單一 2.5 MB 的應用程式分割槽。

在相同的 ESP32-C6 硬體上，Matter-over-Thread 是未來的量產升級途徑 — 無需修改電路板硬體。

## 開發路線圖

- [x] 外側雷達通道的 LD2410 訊框解析器
- [x] UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED 進出模式強制執行機制
- [x] Zigbee 配對加入網路 + Door Lock、Occupancy Sensing 與 Analog (count) clusters
- [x] 透過「雷達 → ToF → 雷達」進行在室佔用計數（進 +1 / 出 −1，下限為 0）
- [x] 用於原點復歸基準真值的各門葉關閉位置極限開關
- [ ] 內側插槽選用 LD2450 進行目標座標方向推估（提升計數精確度）
- [ ] OTA 支援（需更大容量的快閃記憶體或更精簡的 Zigbee 映像檔以恢復雙應用程式分割槽）
- [ ] Matter-over-Thread 遷移

## 作者

第 2 組 — 漢斯應用科技大學（Hanze University of Applied Sciences）：

- Yu-I Juan
- Ali Rowshanzadeh
- Ayyan Jayakar
- Andrei Dogari
- Wout de Moel
