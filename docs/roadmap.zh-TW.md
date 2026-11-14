[English](roadmap.md) · **繁體中文**

# 路線圖與作者

## 開發路線圖

- [x] 外側雷達通道的 LD2410 訊框解析器
- [x] UNLOCKED / IN-ONLY / OUT-ONLY / LOCKED 進出模式的強制執行
- [x] Zigbee 配網 + Door Lock、Occupancy Sensing、Analog（計數）clusters
- [x] 用雷達 → ToF → 雷達做在室數量計數（進 +1 / 出 −1，最低到 0）
- [x] 各門葉關閉位置的極限開關，作為 homing 的基準
- [ ] 內側插槽選配 LD2450，用目標座標推方向（提升計數準確度）
- [ ] OTA 支援（需要更大的 flash 或更精簡的 Zigbee 映像，才能恢復雙應用程式分割槽）
- [ ] Matter-over-Thread 遷移

## 作者

Group 2，漢斯應用科技大學（Hanze University of Applied Sciences）：

- Yu-I Juan
- Ali Rowshanzadeh
- Ayyan Jayakar
- Andrei Dogari
- Wout de Moel
