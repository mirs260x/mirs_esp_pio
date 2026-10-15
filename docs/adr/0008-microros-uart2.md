# 0008: micro-ROSをUART2へ分離

- Status: 採用
- Date: 2026-10-06

## 背景

- Raspberry Pi上のmicro-ROS AgentとESP32をGPIO UARTで接続する。
- UART0 (`Serial`) はUSB-UARTブリッジと接続され、書き込みや診断ログにも使う。
- micro-ROSフレームと診断ログが同じUARTを共有すると、ログがプロトコル通信を壊す。

## 決定

- micro-ROS transportはUART2 (`Serial2`) に分離する。
- ESP32-DevKitのGPIO16をRX、GPIO17をTXとして使用し、速度は115200 baud、8N1とする。
- UART0 (`Serial`) はUSB経由のファームウェア書き込みと診断ログに残す。
- GPIO16/17を使えないESP32モジュールでは、`hardware_config.hpp` のピン割当てを確認・変更する。

## 配線

- Pi TX → ESP32 GPIO16 (RX)
- Pi RX ← ESP32 GPIO17 (TX)
- Pi GND ↔ ESP32 GND
- 両側とも3.3V UARTとし、Pi GPIOに5Vを入力しない。

## 結果

- USB診断ログとmicro-ROS通信を同時に利用できる。
- Pi側Agentのデバイスと速度は `mirs` launch引数 `esp_port` / `esp_baudrate` で同じ値に設定する。
- 実機接続・走行試験は別途行う。
