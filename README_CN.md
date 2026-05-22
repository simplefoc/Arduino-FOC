# SimpleFOClibrary — 简单磁场定向控制 (FOC) 库

### 基于 Arduino IDE 和 PlatformIO 的跨平台 FOC 实现
### 适用于 BLDC 无刷电机和步进电机

[![arduino-library-badge](https://www.ardu-badge.com/badge/Simple%20FOC.svg)](https://www.ardu-badge.com/badge/Simple%20FOC.svg)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/askuric/library/Simple%20FOC.svg)](https://registry.platformio.org/libraries/askuric/Simple%20FOC)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## 这是什么

SimpleFOC 是一个**开源的无刷直流电机 (BLDC) 和步进电机 FOC 控制库**，专为普通 MCU 设计。支持 Arduino、PlatformIO，覆盖 STM32 / ESP32 / RP2040 / Teensy 等主流平台。

**一句话**：把 FOC 的 SVPWM、电流环、速度环、位置环全部封装好了，7 行代码就能让电机跑起来。

---

## 为什么选 SimpleFOC？

| 手写 FOC | SimpleFOC |
|----------|-----------|
| 六扇区 SVPWM 手写 | 内置 |
| PI 参数手动反复调 | 在线实时调参 |
| 换 MCU 全部重写 | 改 3 个引脚即可 |
| 开发到首次转动：3~7 天 | **20 分钟** |

---

## 功能一览

- ✅ **SVPWM** — 空间矢量调制，支持 3PWM / 6PWM
- ✅ **电流环** — 在线电流采样 + PI 控制
- ✅ **速度环** — 闭环速度控制
- ✅ **位置环** — 伺服级角度/位置控制
- ✅ **无感 FOC** — 无传感器观测器
- ✅ **自动对准** — 上电自动找转子初始角度
- ✅ **实时调参** — 串口监控 + 在线改 PID

## 支持硬件

| MCU 平台 | 驱动方式 | 传感器 |
|---------|---------|--------|
| **STM32** (F1/F4/G0/G4) | 3PWM / 6PWM / 4PWM | SPI / I2C / ABI / 模拟 / 霍尔 |
| **ESP32** (全系列) | 3PWM / 6PWM / 4PWM + MCPWM | 同上 |
| **RP2040** (Raspberry Pi Pico) | 3PWM / 6PWM / 4PWM | 同上 |
| **Teensy** (3.x/4.x) | 3PWM / 6PWM / 4PWM | 同上 |
| **SAMD** (Arduino Zero/MKR) | 3PWM / 6PWM | 同上 |

支持的传感器：AS5600、AS5048、AS5147、MA730、MT6701、TLE5012、霍尔、编码器……

---

## 5 分钟快速上手

### 硬件清单（~¥80）

| 器件 | 型号 | 约价 |
|------|------|------|
| 主控 | ESP32-DevKitC | ¥25 |
| 驱动 | L6234 模块 | ¥35 |
| 电机 | 2204 云台无刷 (7对极) | ¥20 |
| 编码器 | AS5600 磁编码器 | ¥8 |

### 代码

```cpp
#include <SimpleFOC.h>

// 电机 + 驱动
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11);

// 传感器
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

void setup() {
  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);
  motor.controller = MotionControlType::velocity;
  motor.init();
  motor.initFOC();
}

void loop() {
  motor.loopFOC();
  motor.move(2); // 2 rad/s
}
```

**就是这样。7 行核心代码，速度环跑起来了。**

---

## 安装

**Arduino IDE**：库管理器搜索 `Simple FOC` → 安装

**PlatformIO**：`platformio.ini` 中加：
```ini
lib_deps = askuric/Simple FOC@^2.3.5
```

**手动安装**：
```bash
git clone https://github.com/simplefoc/Arduino-FOC.git
```

---

## 文档与资源

- 📖 [官方文档](https://docs.simplefoc.com)
- 💬 [社区论坛](https://community.simplefoc.com)
- 📺 [YouTube 教程](https://www.youtube.com/@simplefoc)
- 🎓 [Arduino FOC 视频课程](https://www.youtube.com/playlist?list=PL9ZcB2YBbmnsz9pfwSgl0zBcSdvGe8Fhr)

---

## 贡献

欢迎提 PR！无论是修 bug、加驱动、改进文档还是翻译，都非常欢迎。

**如何开始**：
1. Fork 本仓库
2. 创建分支：`git checkout -b feature/xxx`
3. 提交：`git commit -m 'feat: xxx'`
4. 推送：`git push origin feature/xxx`
5. 提 Pull Request

---

## 常见问题

**Q: 电机狂抖？**
A: 检查对极对数是否填对（2204 电机通常是 7 对极），检查电流采样方向。

**Q: USB 通信不稳定？**
A: FOC 循环占用 CPU 较高，建议降低 `motor.loopFOC()` 频率或用 RTOS 分任务。

**Q: 想用 STM32 但不想要 Arduino？**
A: 参考社区讨论 [Standalone library usage without Arduino](https://github.com/simplefoc/Arduino-FOC/issues/427)，或关注 v3.0 roadmap。

**Q: 无感 FOC 怎么用？**
A: 参考 `examples/sensorless` 目录，选择观测器类型（线性/非线性），注意供电要足。

---

> 本文由社区贡献者「困困困困好困啊」翻译  
> 原文：[SimpleFOC README](https://github.com/simplefoc/Arduino-FOC)
