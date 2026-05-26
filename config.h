#pragma once

// ── I2C 配置 ──────────────────────────────────────────────
#define I2C_SDA      53
#define I2C_SCL      54
#define I2C_FREQ     400000
#define I2C_ADDR     0x6D

// 传感器量程 (Pa)
#define P_MIN        0
#define P_MAX        40000

// ── 图表布局（逻辑横屏 1280×720）─────────────────────────
#define SCREEN_W     1280
#define SCREEN_H     720
#define STATUS_H     80

#define CHART_X      100
#define CHART_Y      690
#define CHART_W      1160
#define CHART_H      590
#define CHART_VISIBLE_POINTS 200
#define MAX_POINTS   10000  // 约 5 分钟历史（按 32ms/点估算）

#define RESUME_BTN_W 180
#define RESUME_BTN_H 56
#define RESUME_BTN_X (SCREEN_W - RESUME_BTN_W - 24)
#define RESUME_BTN_Y (SCREEN_H - RESUME_BTN_H - 24)

// ── 颜色 ───────────────────────────────────────────────────
#define COLOR_BG      0x000000UL
#define COLOR_AXIS    0xFFFFFFUL
#define COLOR_LINE    0x00FF00UL
#define COLOR_ZERO    0x888888UL
#define COLOR_CURRENT 0xFFFF00UL
#define COLOR_CALIB   0x00BFFFUL
#define COLOR_PAUSED  0xFFA000UL
#define COLOR_BUTTON  0x1E88E5UL

// ── 时序 ───────────────────────────────────────────────────
#define SENSOR_CONV_MS    32
#define CALIB_DURATION_MS 3000
#define RUNNING_FRAME_MS  16
#define CALIB_FRAME_MS    33
