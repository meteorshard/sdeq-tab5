/*
 * eq_tab5.ino
 * 气压传感器实时折线图显示（移植自 eq_cardputer.ino）
 *
 * 硬件：M5Stack Tab5 (ESP32-P4)
 * 传感器：I2C 压力传感器，地址 0x6D，接 Grove 口 (SDA=GPIO53, SCL=GPIO54)
 *
 * 依赖库：M5Unified (>=0.2.7)、M5GFX (>=0.2.8)
 * Board Manager URL: https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json
 *
 * 操作说明：
 *   - 点击屏幕任意位置开始（先 3 秒校准归零，再开始记录）/ 停止
 */

#include <M5Unified.h>
#include <Wire.h>

// ── I2C 配置 ──────────────────────────────────────────────
#define I2C_SDA      53
#define I2C_SCL      54
#define I2C_FREQ     100000
#define I2C_ADDR     0x6D

// 传感器量程 (Pa)
#define P_MIN        0
#define P_MAX        40000

// ── 图表布局（屏幕坐标，1280×720）─────────────────────────
#define SCREEN_W     1280
#define SCREEN_H     720
#define STATUS_H     80    // 顶部状态栏高度

#define CHART_X      100   // Y 轴位置（左侧留给标签）
#define CHART_Y      690   // X 轴位置（底部）
#define CHART_W      1160
#define CHART_H      590
#define MAX_POINTS   200

// ── 颜色 (RGB888，M5GFX 自动转换) ─────────────────────────
#define COLOR_BG      0x000000UL
#define COLOR_AXIS    0xFFFFFFUL
#define COLOR_LINE    0x00FF00UL
#define COLOR_ZERO    0x888888UL
#define COLOR_CURRENT 0xFFFF00UL
#define COLOR_CALIB   0x00BFFFUL

// 每隔多少毫秒推入一个新数据点（传感器单次读取需 30ms，此值不应低于 50ms）
#define DATA_INTERVAL_MS  150
// 校准持续时间
#define CALIB_DURATION_MS 3000

// ── 状态机 ────────────────────────────────────────────────
enum AppState { STATE_IDLE, STATE_CALIBRATING, STATE_RUNNING, STATE_STOPPED };

AppState      g_state          = STATE_IDLE;
float         g_data[MAX_POINTS];
int           g_data_count     = 0;
unsigned long g_last_push_ms   = 0;
float         g_live_hpa       = 0.0f;  // 渲染用缓存值，与传感器读取解耦

// 校准相关
float         g_offset         = 0.0f;
double        g_calib_sum      = 0.0;
int           g_calib_count    = 0;
unsigned long g_calib_start_ms = 0;

// ── 双缓冲精灵 ────────────────────────────────────────────
M5Canvas g_canvas(&M5.Display);

// ──────────────────────────────────────────────────────────
// 读取压强（hPa，原始值）；失败返回 NAN
// ──────────────────────────────────────────────────────────
float readPressureHpa() {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(0x30);
    Wire.write(0x0A);
    if (Wire.endTransmission() != 0) return NAN;

    delay(30);

    Wire.beginTransmission(I2C_ADDR);
    Wire.write(0x06);
    if (Wire.endTransmission(false) != 0) return NAN;

    if (Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)3) < 3) return NAN;

    uint8_t b0 = Wire.read();
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();

    int32_t adc_val = ((int32_t)b0 << 16) | ((int32_t)b1 << 8) | b2;
    if (adc_val > 8388608L) adc_val -= 16777216L;

    float pressure_pa = (adc_val / 2097152.0f) * (P_MAX - P_MIN) + P_MIN;
    return pressure_pa / 100.0f;
}

// ──────────────────────────────────────────────────────────
// 在精灵上绘制坐标轴
// ──────────────────────────────────────────────────────────
void drawAxes() {
    g_canvas.drawLine(CHART_X, CHART_Y - CHART_H, CHART_X, CHART_Y, COLOR_AXIS);
    g_canvas.drawLine(CHART_X, CHART_Y, CHART_X + CHART_W, CHART_Y, COLOR_AXIS);
}

// ──────────────────────────────────────────────────────────
// 渲染整帧到精灵并一次性推送（消除闪烁）
// hpa：已减去 offset 的相对值（STATE_RUNNING 时有效）
// elapsed_ms：校准已用时间（STATE_CALIBRATING 时有效）
// ──────────────────────────────────────────────────────────
void pushFrame(float hpa, int elapsed_ms) {
    g_canvas.fillScreen(COLOR_BG);

    // ── 顶部状态栏 ──
    g_canvas.setTextSize(3);
    g_canvas.setTextColor(COLOR_AXIS, COLOR_BG);
    if (g_state == STATE_IDLE) {
        g_canvas.drawString("Tap anywhere to Start", 10, 20);
    } else if (g_state == STATE_STOPPED) {
        g_canvas.drawString("Stopped. Tap anywhere to Start", 10, 20);
    } else if (g_state == STATE_RUNNING) {
        g_canvas.drawString("Recording...", 10, 20);
    } else if (g_state == STATE_CALIBRATING) {
        int secs_left = (int)((CALIB_DURATION_MS - elapsed_ms + 999) / 1000);
        if (secs_left < 0) secs_left = 0;
        char buf[32];
        snprintf(buf, sizeof(buf), "Calibrating...  %d", secs_left);
        g_canvas.setTextColor(COLOR_CALIB, COLOR_BG);
        g_canvas.drawString(buf, 10, 20);
    }

    // ── 校准进度条（水平居中）──
    if (g_state == STATE_CALIBRATING) {
        int bar_w = CHART_W;
        int bar_x = (SCREEN_W - bar_w) / 2;
        int bar_y = SCREEN_H / 2 - 10;
        int bar_h = 20;
        int filled = (int)((float)elapsed_ms / CALIB_DURATION_MS * bar_w);
        if (filled > bar_w) filled = bar_w;
        g_canvas.drawRect(bar_x, bar_y, bar_w, bar_h, COLOR_AXIS);
        if (filled > 1)
            g_canvas.fillRect(bar_x + 1, bar_y + 1, filled - 1, bar_h - 2, COLOR_CALIB);
        g_canvas.pushSprite(0, 0);
        return;
    }

    // ── 坐标轴 ──
    drawAxes();

    // ── 折线图（仅 RUNNING 且有足够数据时）──
    if (g_state == STATE_RUNNING && g_data_count >= 2) {
        float cur_min = g_data[0], cur_max = g_data[0];
        for (int i = 1; i < g_data_count; i++) {
            if (g_data[i] < cur_min) cur_min = g_data[i];
            if (g_data[i] > cur_max) cur_max = g_data[i];
        }

        // 将当前实时值纳入范围，防止曲线越界
        if (hpa < cur_min) cur_min = hpa;
        if (hpa > cur_max) cur_max = hpa;

        float display_min = (cur_min > 0.0f) ? 0.0f : cur_min;
        float y_range     = cur_max - display_min;
        if (y_range < 100.0f) y_range = 100.0f;
        float display_max = ceilf(display_min + y_range);
        y_range = display_max - display_min;

        // 0 基线
        if (display_min < 0.0f) {
            int y_zero = (int)(CHART_Y - (0.0f - display_min) / y_range * CHART_H);
            g_canvas.drawLine(CHART_X, y_zero, CHART_X + CHART_W, y_zero, COLOR_ZERO);
            g_canvas.setTextSize(2);
            g_canvas.setTextColor(COLOR_AXIS, COLOR_BG);
            g_canvas.drawString("0", 2, y_zero - 12);
        }

        // 当前实时值的 Y 坐标（每帧更新）
        int cur_y = (int)(CHART_Y - ((hpa - display_min) / y_range) * CHART_H);

        // 实时段动态 X：根据距上次推入数据点的时间线性插值，平滑向右移动
        float x_step = (float)CHART_W / (MAX_POINTS - 1);
        float frac   = (float)(millis() - g_last_push_ms) / DATA_INTERVAL_MS;
        if (frac > 1.0f) frac = 1.0f;
        int live_x;
        if (g_data_count < MAX_POINTS) {
            live_x = (int)(CHART_X + (g_data_count - 1 + frac) * x_step);
        } else {
            live_x = CHART_X + CHART_W;
        }
        if (live_x > CHART_X + CHART_W) live_x = CHART_X + CHART_W;

        // 历史折线（已存数据）
        for (int i = 0; i < g_data_count - 1; i++) {
            int x1 = (int)(CHART_X + i * x_step);
            int y1 = (int)(CHART_Y - ((g_data[i]     - display_min) / y_range) * CHART_H);
            int x2 = (int)(CHART_X + (i + 1) * x_step);
            int y2 = (int)(CHART_Y - ((g_data[i + 1] - display_min) / y_range) * CHART_H);
            g_canvas.drawLine(x1, y1, x2, y2, COLOR_LINE);
        }

        // 实时段：从最后存入点 → 当前实时 hpa
        if (g_data_count >= 1) {
            int x_last = (int)(CHART_X + (g_data_count - 1) * x_step);
            int y_last = (int)(CHART_Y - ((g_data[g_data_count - 1] - display_min) / y_range) * CHART_H);
            if (live_x > x_last) {
                g_canvas.drawLine(x_last, y_last, live_x, cur_y, COLOR_LINE);
            }
        }

        // Y 轴标注
        g_canvas.setTextSize(2);
        char buf[16];
        g_canvas.setTextColor(COLOR_AXIS, COLOR_BG);
        snprintf(buf, sizeof(buf), "%d", (int)display_max);
        g_canvas.drawString(buf, 2, CHART_Y - CHART_H);
        snprintf(buf, sizeof(buf), "%d", (int)floorf(display_min));
        g_canvas.drawString(buf, 2, CHART_Y - 20);

        // 当前值标签（黄色浮动）
        int text_y = cur_y - 12;
        if (text_y < CHART_Y - CHART_H) text_y = CHART_Y - CHART_H;
        if (text_y > CHART_Y - 20)      text_y = CHART_Y - 20;

        g_canvas.setTextColor(COLOR_CURRENT, COLOR_BG);
        snprintf(buf, sizeof(buf), "%.2f", hpa);
        g_canvas.drawString(buf, 2, text_y);

        // 虚线：从 Y 轴（CHART_X）→ live_x，指向当前值
        const int DASH = 8, GAP = 5;
        for (int x = CHART_X + 1; x < live_x; x += DASH + GAP) {
            int w = DASH;
            if (x + w > live_x) w = live_x - x;
            if (w > 0) g_canvas.fillRect(x, cur_y, w, 2, COLOR_CURRENT);
        }
    }

    // ── 一次性推送到屏幕（双缓冲，无闪烁）──
    g_canvas.pushSprite(0, 0);
}

// ──────────────────────────────────────────────────────────
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Display.setRotation(1);  // 横屏：默认竖屏(720×1280)，旋转后变(1280×720)

    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    M5.Display.fillScreen(COLOR_BG);

    // 创建全屏精灵（16 位色，ESP32-P4 PSRAM 充足）
    g_canvas.setColorDepth(16);
    g_canvas.createSprite(SCREEN_W, SCREEN_H);
    g_canvas.setTextSize(3);

    pushFrame(0.0f, 0);
}

// ──────────────────────────────────────────────────────────
void loop() {
    M5.update();

    // ── 触摸检测（点击任意位置触发）──
    auto touch = M5.Touch.getDetail();
    if (touch.wasPressed()) {
        if (g_state == STATE_IDLE || g_state == STATE_STOPPED) {
            g_state           = STATE_CALIBRATING;
            g_calib_sum       = 0.0;
            g_calib_count     = 0;
            g_calib_start_ms  = millis();
            g_data_count      = 0;
        } else if (g_state == STATE_RUNNING) {
            g_state = STATE_STOPPED;
            pushFrame(0.0f, 0);
        }
        // 校准中忽略触摸
    }

    // ── 校准阶段 ──
    if (g_state == STATE_CALIBRATING) {
        unsigned long elapsed = millis() - g_calib_start_ms;

        float raw = readPressureHpa();
        if (!isnan(raw)) {
            g_calib_sum += raw;
            g_calib_count++;
        }

        pushFrame(0.0f, (int)elapsed);

        if (elapsed >= CALIB_DURATION_MS) {
            g_offset       = (g_calib_count > 0) ? (float)(g_calib_sum / g_calib_count) : 0.0f;
            g_state        = STATE_RUNNING;
            g_last_push_ms = millis();
        }
        return;
    }

    // ── 记录阶段 ──
    if (g_state == STATE_RUNNING) {
        unsigned long now = millis();

        // 传感器只在需要推入新数据点时才读（避免 30ms 阻塞拖慢渲染）
        if (now - g_last_push_ms >= DATA_INTERVAL_MS) {
            float raw = readPressureHpa();
            if (!isnan(raw)) {
                g_live_hpa     = raw - g_offset;
                g_last_push_ms = now;
                if (g_data_count < MAX_POINTS) {
                    g_data[g_data_count++] = g_live_hpa;
                } else {
                    memmove(g_data, g_data + 1, (MAX_POINTS - 1) * sizeof(float));
                    g_data[MAX_POINTS - 1] = g_live_hpa;
                }
            }
        }

        // 渲染每次 loop 都跑，用缓存值画图，不再被传感器阻塞
        pushFrame(g_live_hpa, 0);
    }
}
