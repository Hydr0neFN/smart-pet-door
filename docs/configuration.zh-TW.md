[English](configuration.md) · **繁體中文**

# 參數設定

[`src/main.cpp`](../src/main.cpp) 最上方幾個主要的編譯期選項：

| 巨集 | 預設值 | 用途 |
|---|---|---|
| `RADIO_MODE` | 依環境而定 | `0` Wi-Fi / `1` Zigbee / `2` 兩者 — 由 PlatformIO 環境設定，不要手動改 |
| `USE_RADAR_IN` / `USE_RADAR_OUT` | `1` / `1` | 啟用各雷達插槽；停用內側雷達可以把 UART0 還給 CH340 主控台 |
| `TEST_TIMED_CYCLE` | 自動推導 | `(!USE_RADAR_IN && !USE_RADAR_OUT)` — 兩顆雷達都關 ⇒ 定時開關循環；只要有一顆開 ⇒ 由雷達驅動。沒有另一個旗標會忘記設 |
| `MOTOR_SELFTEST` | `0` | `1` = 開機時讓每片門葉動一下，記錄命令位置和回報位置（查門葉不動的原因：接線/VM 還是脈衝產生） |
| `SENSOR_DEBUG` | `0` | `1` = 串流輸出原始 ToF 和雷達讀值（約 4 Hz，含 `rx`/`good`/`bad` 訊框計數），並跳過門的 FSM — 感測器初測與對準 |
| `USE_END_STOP1` / `USE_END_STOP2`（`END_STOP1`/`END_STOP2`） | `0` / `0` | 各門葉接好關閉開關後再啟用 |
| `USE_TOF` | `0` | 啟用 VL53L1X — 防夾**和**寵物計數都要用 |
| `OPEN_DEG` | `180` | 開門角度（度） |
| `RMS_CURRENT` | `800` | 馬達運轉電流（mA, IRUN） |
| `IHOLD_MULT` | `0.3` | 靜止保持電流佔運轉電流多少 |
| `CLOSE_DELAY_MS` | `3000` | 雷達淨空後等多久才開始關門 |
| `TOF_DETECT_MM` | `350` | ToF 距離低於這個值就判定有障礙物（盲區下限 40 mm） |

初測工具：兩顆雷達都關掉時，韌體會自己切成定時開關循環（不用另外設旗標）。這樣在接雷達之前就能先看機構動作、ToF 防夾反轉和電流／發熱。門葉不轉時用 `MOTOR_SELFTEST` 排查。注意：就算沒有 12 V 馬達電源，TMC2130 透過 SPI 還是會回報正常的 `version`；`configDriver` 會另外對電荷泵欠壓發警告，不會把少了 VM 當成驅動器正常。要對準感測器或確認訊框解析時，用 `SENSOR_DEBUG` 看即時的 ToF／雷達數值。
