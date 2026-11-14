[English](zigbee.md) · **繁體中文**

# 無線電設定檔、安全、進出模式與 Zigbee

## 無線電設定檔

ESP32-C6 同時有 2.4 GHz Wi-Fi 和 802.15.4（Zigbee）無線電。用建置環境選**無線電設定檔（radio profile）**，每個環境會設定對應的 `RADIO_MODE`：

| 環境 | `RADIO_MODE` | 無線電 | 日誌 | 用途 |
|---|:---:|---|---|---|
| `debug-wifi` *(預設)* | 0 | 只有 Wi-Fi | telnet `:23`（內側雷達關掉時可加 CH340） | 初測／除錯 |
| `deploy-zigbee` | 1 | 只有 Zigbee | USB 序列（內側雷達關掉時） | 出貨用的智慧家庭裝置 |
| `debug-both` | 2 | Wi-Fi + Zigbee（共存 coex） | telnet `:23` | 邊看即時日誌邊除錯 Zigbee（吃較多 RAM） |

Zigbee 版本用單一應用程式、**沒有 OTA** 的分割表（`partitions_zigbee.csv`），原因見下面的實作筆記。

## 安全系統

原本規劃三層障礙物防護，第 1、2 層是預先防護（碰到之前就動作）。

| 層級 | 感測器 | 方法 | 狀態 |
|---|---|---|---|
| 1 | VL53L1X ToF | 關門時輪詢距離 → 反轉 | ✅ 已實作（防夾重開） |
| 2 | 毫米波雷達 (mmWave radar) | 存在感測保持，有人在時不關門 | ✅ 已實作 |
| 3 | TMC2130 StallGuard | 透過 SPI 讀反電動勢判斷堵轉 | ❌ 放棄 — StallGuard 需要 SpreadCycle，跟所選的 StealthChop 無法並存；障礙物改由 ToF 負責 |

## 進出模式

雷達對判定的移動方向會強制執行四種進出模式，四種都能遠端設定。

| 方向 \ 模式 | UNLOCKED（解鎖） | IN-ONLY（僅進） | OUT-ONLY（僅出） | LOCKED（上鎖） |
|---|:---:|:---:|:---:|:---:|
| 進入（IN）— 外側雷達 | ✓ | ✓ | ✗ | ✗ |
| 出去（OUT）— 內側雷達 | ✓ | ✗ | ✓ | ✗ |

## Zigbee 介面

Zigbee 介面（`deploy-zigbee` / `debug-both` 版本才有）有四個端點：

- **Door Lock cluster。** 安全／上鎖介面。裝置以 Door Lock 身分出現，標準 `LockState` 會顯示門是不是 `LOCKED` 模式（一般控制器都讀得到）。
- **Multistate Output cluster。** 四向進出模式 `UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED`（索引 0–3）。這是標準 cluster，所以閘道器（ZHA / Zigbee2MQTT）會直接把它顯示成可寫入的選單，不用另外寫 quirk。
- **Occupancy Sensing cluster。** `occupied` 旗標（室內有寵物時為 true）。
- **Analog Input cluster。** 即時寵物數量（目前室內幾隻），有變動就回報。

兩種模式介面都會跟韌體的 `doorMode` 同步：任一邊設成 `LOCKED`，Door Lock 都會變成 Locked，模式一改兩邊都會收到。

## Zigbee 實作筆記（為什麼這樣做）

- **esp-zigbee-sdk 跟著框架走，沒有另外加相依。** Arduino-ESP32 v3.x（pioarduino 分支）*內建* Espressif 的 [esp-zigbee-sdk](https://github.com/espressif/esp-zigbee-sdk)：`esp_zb_*` API 和預先編譯好的 zboss 函式庫。`#include <Zigbee.h>` 只是上面一層輕量 Arduino 包裝。自訂的門鎖端點直接呼叫 SDK（`esp_zb_door_lock_clusters_create` 等），`ZIGBEE_MODE_ED` 建置旗標讓框架連結 zboss。`lib_deps` 不用加東西，因為另外加一份 SDK 會跟內建的那份衝突。
- **模式用 Multistate 而不是廠商屬性。** 標準 ZCL Door Lock 只有二元狀態（LockState Locked/Unlocked），沒有地方放有方向的 IN-ONLY/OUT-ONLY。最早是把四種模式塞在 Door Lock cluster 的*廠商專屬屬性*裡，但一般閘道器沒有自訂轉換器／quirk 就看不到。換成**標準 Multistate Output** 後，閘道器原生就顯示成四個選項，開箱就能控制。Door Lock cluster 只負責安全／上鎖的廣告和狀態。
- **Door Lock 端點自己寫，其他都用現成的。** Arduino 包裝有 Multistate、Occupancy、Analog 的類別（都直接用），但**沒有 Door Lock**，所以 `ZigbeePetDoorLock` 是自己在 cluster `0x0101` 上手寫的 `ZigbeeEP` 子類別。
- **非 Zigbee 環境用 `lib_ignore = Zigbee`。** 內建的 Zigbee 函式庫不管怎樣都會編譯，但 zboss 只有在有 `ZIGBEE_MODE_ED` 時才會連結；少了它會在連結階段失敗（`undefined reference to esp_zb_ep_list_create`）。在 `debug-wifi` 忽略這個函式庫就不會發生。
- **沒有 OTA 的分割表。** 4 MB flash 上，zboss 映像（約 1.39 MB）塞不進原廠雙 OTA `zigbee.csv` 的應用程式分割槽（各 1.31 MB）。`partitions_zigbee.csv` 拿 OTA 換一個 2.5 MB 的分割槽。

Matter-over-Thread 是同一塊 ESP32-C6 硬體上的量產升級路，不用改板子。

## 動作函式庫

動作控制用 **AccelStepper**（軟體 STEP/DIR 脈衝，任何 GPIO 都能用）。FastAccelStepper 的硬體脈衝產生在 ESP32-C6 上只推得動一片門葉，AccelStepper 則不受週邊通道數限制。TMC2130 仍走 SPI 設定 StealthChop。
