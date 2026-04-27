# SDEQ Tab5

M5Stack Tab5（ESP32-P4）上的气压传感器实时折线图，支持归零校准。

## 硬件

| 组件 | 说明 |
|---|---|
| 主控 | M5Stack Tab5（ESP32-P4，带 PSRAM） |
| 传感器 | I2C 气压传感器，地址 `0x6D` |
| 接口 | Grove 口（SDA = GPIO53，SCL = GPIO54） |
| 量程 | 0 – 400 hPa（相对压力） |

## 功能

- **3 秒归零校准**：启动后采集基准压力，后续显示相对值
- **实时折线图**：200 个历史数据点，自动缩放 Y 轴
- **当前值气泡**：跟随曲线末端显示，自动避边
- **点击切换**：屏幕任意位置点击开始 / 停止记录

## 渲染架构

```
Core 1 (loop)          Core 0 (taskPush)
─────────────────      ─────────────────────
render → back canvas   push front canvas → display
wait push_done    ←→   give push_done
swap front/back
give render_done  ─→   take render_done
```

双 Canvas ping-pong + FreeRTOS 双核并行，`pushSprite` 走 `r=0 memcpy` 快速路径，无格式转换开销。传感器 I2C 读取非阻塞，渲染帧率与采样周期解耦。

## 项目结构

```
.
├── eq_tab5.ino       # 状态机 + 双 Canvas ping-pong 调度
├── config.h          # 引脚 / 量程 / 布局 / 颜色 / 时序常量
├── sensor.h/.cpp     # 非阻塞 I2C 气压传感器驱动
├── chart_data.h/.cpp # 环形缓冲 + 增量 min/max
├── chart_view.h/.cpp # 状态栏 / 校准进度条 / 折线图渲染
├── platformio.ini
└── boards/           # M5Stack Tab5 自定义 board 定义
```

## 依赖

- [pioarduino/platform-espressif32](https://github.com/pioarduino/platform-espressif32)（ESP32-P4 / arduino-esp32 3.x 支持）
- [M5Unified](https://github.com/m5stack/M5Unified) >= 0.2.14
- [M5GFX](https://github.com/m5stack/M5GFX) >= 0.2.8

## 构建 & 烧录

```bash
# 安装 PlatformIO CLI
pip install platformio

# 编译
pio run

# 编译并烧录（默认端口 /dev/cu.usbmodem1101）
pio run -t upload

# 串口监视（FPS 输出）
pio device monitor
```

## 内存占用（ESP32-P4）

| 区域 | 已用 |
|---|---|
| Flash | 39% |
| RAM (DIRAM) | 6% |
| PSRAM | ~3.7 MB（双 Canvas） |
