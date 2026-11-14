[English](README.md) · **繁體中文**

# 智慧寵物門 🐾

<img width="1917" height="1078" alt="image" src="https://github.com/user-attachments/assets/cc415e5f-6828-416f-af6a-ca0d08f0b3f9" />

馬達驅動的寵物門：雷達確認有動物靠近才開，平常關著、把開口封好。

## 一覽

| | |
|---|---|
| 要解決的問題 | 傳統活動門片會讓建築外殼一直有個缺口，空調／暖通（HVAC）耗能每年可能多出 6% 以上。 |
| 我做了什麼 | 一扇電子驅動、隔熱的寵物門。門檻內外各一顆毫米波雷達；ToF 安全光幕在門葉碰到東西之前就讓它停下；兩顆 TMC2130 驅動的步進馬達安靜地推門。 |
| 成果 | 舊版 README 沒有實測結果。主要預設值：開門角度 180°；雷達淨空後 3000 ms 才關門；運轉電流 800 mA，保持電流為運轉電流的 0.3 倍。 |
| 技術組合 | ESP32-C6 DevKitM-1，PlatformIO 加 Arduino-ESP32 v3.x（pioarduino 分支），Zigbee（Door Lock + Occupancy Sensing + Analog Input）或 Wi-Fi telnet 日誌，AccelStepper、TMCStepper、Adafruit VL53L1X |
| 狀態 | archived |
| 我的角色 | Group 2 成員（漢斯應用科技大學 Hanze University of Applied Sciences） |

## 運作原理

門檻內外各放一顆 **24 GHz 毫米波雷達**（每側一顆 **LD2410**）。雷達偵測到目標靠近、而且目前進出模式允許這個方向時，控制器就把門打開。感測區淨空並經過一段保持延遲後，門再關上。

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

兩顆 NEMA 17 馬達背對背裝在鉸鏈處，以鏡像方式驅動，兩片門葉像同一個開口一樣一起開合。門口的 **VL53L1X ToF 感測器**是一道看不見的安全光幕：關門時光束上一有東西擋著，門就反轉開回去。同一道光束也負責偵測門洞中央的「穿越」事件，用來數寵物。

## 工程歷程

- **症狀：一片門葉不動。** FastAccelStepper 的硬體脈衝產生在 ESP32-C6 上只推得動一側。
- **原因。** 舊版 README 沒寫。AccelStepper 沒有週邊通道數限制。
- **解法。** 改用 **AccelStepper**：用軟體在任意 GPIO 上打 STEP/DIR 脈衝，不受週邊通道數限制。TMC2130 仍走 SPI 設定 StealthChop。
- **症狀：關門時可能夾到寵物。**
- **解法。** ToF 光幕在接觸前就動作，這是安全設計的第 1 層；雷達存在感測保持開門（第 2 層）負責寵物在附近時不關門。第 3 層 StallGuard 放棄了。
- **症狀：閘道器看不到進出模式。** 四種模式一開始放在廠商專屬屬性裡，一般閘道器讀不到。
- **解法。** 改放標準 Multistate Output cluster，閘道器直接顯示成四選一的下拉選單。
- **還開著的事。** OTA；內側插槽選配 LD2450（用目標座標推方向，提升計數準確度）；Matter-over-Thread 遷移。

## 建置與執行

需要 [PlatformIO](https://platformio.org/)（CLI 或 VS Code 擴充套件）。C6 開發板**未**收錄在官方 `espressif32` 平台（Arduino-ESP32 v2.x），所以 `platformio.ini` 指定了 [pioarduino](https://github.com/pioarduino/platform-espressif32) 分支（Arduino-ESP32 v3.x）。相依套件（`TMCStepper`、`AccelStepper`、`Adafruit VL53L1X`）會在第一次建置時自動下載。

用建置環境挑無線電設定檔，每個環境會設定對應的 `RADIO_MODE`。

```bash
# Build a profile
pio run -e debug-wifi
pio run -e deploy-zigbee

# Flash + monitor (debug build over CH340, only valid when inside radar disabled)
pio run -e deploy-zigbee -t upload
pio device monitor -b 115200
```

Wi-Fi 版本的日誌走 telnet：連到 `petdoor.local:23`（或裝置 IP）即可，晚連上來的客戶端會先收到開機累積的日誌。

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

## 限制與待辦

- **主控台。** 內側雷達佔用 **UART0 (GPIO16/17)**，這跟 CH340 USB 序列主控台是同一組腳位；原生 USB-Serial-JTAG 腳位（GPIO12/13）又被拿去接 MISO / END_STOP1。所以內側雷達一接上，CH340 主控台就不能用，**即時日誌只剩 Wi-Fi telnet**。只有在停用內側雷達（`USE_RADAR_IN 0`）時才用得到 CH340 主控台。
- **沒有 OTA。** 4 MB flash 上，zboss 映像約 1.39 MB，塞不進原廠雙 OTA `zigbee.csv` 每個 1.31 MB 的應用程式分割槽。`partitions_zigbee.csv` 改用單一 2.5 MB 分割槽，拿掉 OTA。
- **計數準確度還能再調。** 內側插槽可選配的 LD2450 還沒做。
- **安全第 3 層放棄。** StallGuard 需要 SpreadCycle，無法跟所選的 StealthChop 並存，改由 ToF 負責障礙物偵測。第 1、2 層已實作：防夾重開，以及存在感測保持開門。
- **未完成。** OTA 支援（需要更大的 flash 或更精簡的 Zigbee 映像，才能恢復雙應用程式分割槽），以及 Matter-over-Thread 遷移。

其他細節：

- [硬體](docs/hardware.zh-TW.md)：零件表、功能、GPIO 與主控台注意事項。
- [參數設定](docs/configuration.zh-TW.md)：編譯期選項與初測輔助工具。
- [Zigbee、進出模式與安全](docs/zigbee.zh-TW.md)：四種進出模式、Zigbee 端點與實作筆記。
- [路線圖與作者](docs/roadmap.zh-TW.md)：已完成、未完成項目與小組名單。

## 授權

MIT，詳見 [LICENSE](LICENSE)。
