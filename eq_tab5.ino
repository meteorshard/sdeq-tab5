/*
 * eq_tab5.ino
 * 气压传感器实时折线图显示（移植自 eq_cardputer.ino）
 *
 * 硬件：M5Stack Tab5 (ESP32-P4)
 * 传感器：I2C 压力传感器，地址 0x6D，接 Grove 口 (SDA=GPIO53, SCL=GPIO54)
 *
 * 依赖库：M5Unified (>=0.2.7)、M5GFX (>=0.2.8)
 *
 * 操作说明：
 *   - 点击屏幕任意位置开始（先 3 秒校准归零，再开始记录）/ 停止
 *
 * 渲染策略：
 *   - Display r=0（物理竖屏）；Canvas 720×1280 + setRotation(1) → 逻辑 1280×720 横屏
 *   - pushSprite 走 r=0 memcpy 快速路径，无格式转换
 *   - 双 Canvas ping-pong + FreeRTOS：
 *       Core 1（loop）渲染到后台 canvas，Core 0（taskPush）将上一帧
 *       前台 canvas pushSprite 到显示器；两核并行，帧时间 = max(渲染, 推送)
 *   - 传感器读取非阻塞（sensorTrigger / sensorReadResult 分离）
 *   - RUNNING 状态：每帧渲染后立即触发下次传感器转换，
 *     下帧开头读取结果 → 每帧都有新数据，无"帧间插值"卡顿
 */

#include <M5Unified.h>
#include <Wire.h>

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
#define MAX_POINTS   200

// ── 颜色 ───────────────────────────────────────────────────
#define COLOR_BG      0x000000UL
#define COLOR_AXIS    0xFFFFFFUL
#define COLOR_LINE    0x00FF00UL
#define COLOR_ZERO    0x888888UL
#define COLOR_CURRENT 0xFFFF00UL
#define COLOR_CALIB   0x00BFFFUL

constexpr float X_STEP = (float)CHART_W / (MAX_POINTS - 1);

// 传感器转换所需最短等待时间（ms）
#define SENSOR_CONV_MS    32
// 校准持续时间
#define CALIB_DURATION_MS 3000

// ── 状态机 ────────────────────────────────────────────────
enum AppState { STATE_IDLE, STATE_CALIBRATING, STATE_RUNNING, STATE_STOPPED };

AppState      g_state          = STATE_IDLE;
float         g_data[MAX_POINTS];
int           g_data_count     = 0;
float         g_live_hpa       = 0.0f;

// 校准相关
float         g_offset         = 0.0f;
double        g_calib_sum      = 0.0;
int           g_calib_count    = 0;
unsigned long g_calib_start_ms = 0;

// 非阻塞传感器状态
bool          g_sensor_converting  = false;
unsigned long g_sensor_trigger_ms  = 0;

// 帧率统计
uint32_t      g_frame_count    = 0;
unsigned long g_fps_last_ms    = 0;

// ── 双 Canvas ping-pong（物理 720×1280，逻辑旋转为 1280×720）──
// 必须传入 &M5.Display 作为 parent，createSprite 才能从 PSRAM 分配 1.84MB 缓冲
M5Canvas g_canvas[2] = { M5Canvas(&M5.Display), M5Canvas(&M5.Display) };
static int g_back  = 0;            // 当前正在渲染的后台 canvas 索引
static int g_front = 1;            // 当前正在推送到显示器的前台 canvas 索引

static SemaphoreHandle_t g_sem_render_done;  // 后台渲染完成，通知推送任务
static SemaphoreHandle_t g_sem_push_done;    // 推送完成，通知主循环可以交换

// ── Core 0 推送任务：pushSprite 前台 canvas 到显示器 ──────
static void taskPush(void*) {
    for (;;) {
        xSemaphoreTake(g_sem_render_done, portMAX_DELAY);
        g_canvas[g_front].pushSprite(0, 0);
        xSemaphoreGive(g_sem_push_done);
        vTaskDelay(1);  // 让出 1ms，避免 IDLE0 饥饿触发 task watchdog
    }
}

// ──────────────────────────────────────────────────────────
// 非阻塞传感器：触发一次 ADC 转换（约 1ms，无 delay）
// ──────────────────────────────────────────────────────────
bool sensorTrigger() {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(0x30);
    Wire.write(0x0A);
    return Wire.endTransmission() == 0;
}

// 读取上一次转换结果（需在 trigger 后 ≥ SENSOR_CONV_MS 调用）
float sensorReadResult() {
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

// 阻塞式读取（仅用于校准阶段）
float readPressureHpa() {
    if (!sensorTrigger()) return NAN;
    delay(SENSOR_CONV_MS);
    return sensorReadResult();
}

// ──────────────────────────────────────────────────────────
// 渲染整帧到后台 canvas（g_canvas[g_back]），不推送到屏幕
// ──────────────────────────────────────────────────────────
void pushFrame(int elapsed_ms) {
    auto& cv = g_canvas[g_back];
    cv.fillScreen(COLOR_BG);

    // ── 顶部状态栏 ──
    cv.setTextSize(3);
    cv.setTextColor(COLOR_AXIS, COLOR_BG);
    if (g_state == STATE_IDLE) {
        cv.drawString("Tap anywhere to Start", 10, 20);
    } else if (g_state == STATE_STOPPED) {
        cv.drawString("Stopped. Tap anywhere to Start", 10, 20);
    } else if (g_state == STATE_RUNNING) {
        cv.drawString("Recording...", 10, 20);
    } else if (g_state == STATE_CALIBRATING) {
        int secs_left = (int)((CALIB_DURATION_MS - elapsed_ms + 999) / 1000);
        if (secs_left < 0) secs_left = 0;
        char buf[32];
        snprintf(buf, sizeof(buf), "Calibrating...  %d", secs_left);
        cv.setTextColor(COLOR_CALIB, COLOR_BG);
        cv.drawString(buf, 10, 20);
    }

    // ── 校准进度条 ──
    if (g_state == STATE_CALIBRATING) {
        int bar_w = CHART_W;
        int bar_x = (SCREEN_W - bar_w) / 2;
        int bar_y = SCREEN_H / 2 - 10;
        int bar_h = 20;
        int filled = (int)((float)elapsed_ms / CALIB_DURATION_MS * bar_w);
        if (filled > bar_w) filled = bar_w;
        cv.drawRect(bar_x, bar_y, bar_w, bar_h, COLOR_AXIS);
        if (filled > 1)
            cv.fillRect(bar_x + 1, bar_y + 1, filled - 1, bar_h - 2, COLOR_CALIB);
        return;
    }

    // ── 坐标轴 ──
    cv.drawLine(CHART_X, CHART_Y - CHART_H, CHART_X, CHART_Y, COLOR_AXIS);
    cv.drawLine(CHART_X, CHART_Y, CHART_X + CHART_W, CHART_Y, COLOR_AXIS);

    // ── 折线图（仅 RUNNING 且有数据时）──
    if (g_state == STATE_RUNNING && g_data_count >= 1) {
        // 计算 Y 轴范围
        float cur_min = g_data[0], cur_max = g_data[0];
        for (int i = 1; i < g_data_count; i++) {
            if (g_data[i] < cur_min) cur_min = g_data[i];
            if (g_data[i] > cur_max) cur_max = g_data[i];
        }

        float display_min = (cur_min > 0.0f) ? 0.0f : cur_min;
        float y_range     = cur_max - display_min;
        if (y_range < 100.0f) y_range = 100.0f;
        float display_max = ceilf(display_min + y_range);
        y_range = display_max - display_min;

        // 0 基线
        if (display_min < 0.0f) {
            int y_zero = (int)(CHART_Y - (0.0f - display_min) / y_range * CHART_H);
            cv.drawLine(CHART_X, y_zero, CHART_X + CHART_W, y_zero, COLOR_ZERO);
            cv.setTextSize(2);
            cv.setTextColor(COLOR_AXIS, COLOR_BG);
            cv.drawString("0", 2, y_zero - 12);
        }

        // 历史折线
        for (int i = 0; i < g_data_count - 1; i++) {
            int x1 = (int)(CHART_X + i * X_STEP);
            int y1 = (int)(CHART_Y - ((g_data[i]     - display_min) / y_range) * CHART_H);
            int x2 = (int)(CHART_X + (i + 1) * X_STEP);
            int y2 = (int)(CHART_Y - ((g_data[i + 1] - display_min) / y_range) * CHART_H);
            cv.drawLine(x1, y1, x2, y2, COLOR_LINE);
        }

        // 最新值指示点
        int last_x = (int)(CHART_X + (g_data_count - 1) * X_STEP);
        int last_y = (int)(CHART_Y - ((g_data[g_data_count - 1] - display_min) / y_range) * CHART_H);
        cv.fillRect(last_x - 3, last_y - 3, 7, 7, COLOR_CURRENT);

        // Y 轴标注
        cv.setTextSize(2);
        char buf[16];
        cv.setTextColor(COLOR_AXIS, COLOR_BG);
        snprintf(buf, sizeof(buf), "%d", (int)display_max);
        cv.drawString(buf, 2, CHART_Y - CHART_H);
        snprintf(buf, sizeof(buf), "%d", (int)floorf(display_min));
        cv.drawString(buf, 2, CHART_Y - 20);

        // 当前值标签（黄色）
        int text_y = last_y - 12;
        if (text_y < CHART_Y - CHART_H) text_y = CHART_Y - CHART_H;
        if (text_y > CHART_Y - 20)      text_y = CHART_Y - 20;
        cv.setTextColor(COLOR_CURRENT, COLOR_BG);
        snprintf(buf, sizeof(buf), "%.2f", g_data[g_data_count - 1]);
        cv.drawString(buf, 2, text_y);
    }

    // 每秒打印 FPS
    g_frame_count++;
    unsigned long now_ms = millis();
    if (now_ms - g_fps_last_ms >= 1000) {
        Serial.printf("FPS: %lu\n", g_frame_count);
        g_frame_count = 0;
        g_fps_last_ms = now_ms;
    }
}

// ──────────────────────────────────────────────────────────
// 渲染后台 canvas，然后交换前/后台并通知推送任务
// 等待上一帧推送完成后再交换，确保无撕裂
// ──────────────────────────────────────────────────────────
void displayFrame(int elapsed_ms) {
    pushFrame(elapsed_ms);
    xSemaphoreTake(g_sem_push_done, portMAX_DELAY);  // 等上一帧推送完
    std::swap(g_back, g_front);                       // 交换前/后台
    xSemaphoreGive(g_sem_render_done);                // 通知 Core 0 推送
}

// ──────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    auto cfg = M5.config();
    M5.begin(cfg);

    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    M5.Display.fillScreen(COLOR_BG);

    // 两个 canvas：物理 720×1280，逻辑旋转为 1280×720 横屏
    // Display 保持 r=0，pushSprite 走 memcpy 快速路径
    M5.Display.setRotation(0);  // 物理竖屏 r=0，确保 pushSprite 走 memcpy 快速路径

    for (int i = 0; i < 2; i++) {
        // 必须与 Panel_DSI 的 rgb565_nonswapped 格式一致，否则每帧逐像素字节交换极慢
        g_canvas[i].setColorDepth(lgfx::rgb565_nonswapped);
        g_canvas[i].createSprite(SCREEN_H, SCREEN_W);  // 720 × 1280 物理
        g_canvas[i].setRotation(1);                     // 逻辑 1280 × 720
        g_canvas[i].setTextSize(3);
    }

    g_sem_render_done = xSemaphoreCreateBinary();
    g_sem_push_done   = xSemaphoreCreateBinary();
    xSemaphoreGive(g_sem_push_done);  // 初始状态：无推送进行中

    xTaskCreatePinnedToCore(taskPush, "push", 4096, nullptr, 5, nullptr, 0);

    displayFrame(0);
}

// ──────────────────────────────────────────────────────────
void loop() {
    M5.update();

    // ── 触摸检测 ──
    auto touch = M5.Touch.getDetail();
    if (touch.wasPressed()) {
        if (g_state == STATE_IDLE || g_state == STATE_STOPPED) {
            g_state           = STATE_CALIBRATING;
            g_calib_sum       = 0.0;
            g_calib_count     = 0;
            g_calib_start_ms  = millis();
            g_data_count      = 0;
            g_sensor_converting = false;
        } else if (g_state == STATE_RUNNING) {
            g_state = STATE_STOPPED;
            g_sensor_converting = false;
            displayFrame(0);
        }
    }

    // ── 校准阶段（阻塞读取，精度优先）──
    if (g_state == STATE_CALIBRATING) {
        unsigned long elapsed = millis() - g_calib_start_ms;
        float raw = readPressureHpa();
        if (!isnan(raw)) {
            g_calib_sum += raw;
            g_calib_count++;
        }
        displayFrame((int)elapsed);
        if (elapsed >= CALIB_DURATION_MS) {
            g_offset    = (g_calib_count > 0) ? (float)(g_calib_sum / g_calib_count) : 0.0f;
            g_state     = STATE_RUNNING;
            g_live_hpa  = 0.0f;
            // 立即触发第一次转换
            sensorTrigger();
            g_sensor_trigger_ms = millis();
            g_sensor_converting = true;
        }
        return;
    }

    // ── 记录阶段 ──
    if (g_state == STATE_RUNNING) {
        unsigned long now = millis();

        // 1. 读取上一帧触发的转换结果（非阻塞：只在等待够 SENSOR_CONV_MS 后读）
        if (g_sensor_converting && now - g_sensor_trigger_ms >= SENSOR_CONV_MS) {
            float raw = sensorReadResult();
            g_sensor_converting = false;
            if (!isnan(raw)) {
                g_live_hpa = raw - g_offset;
                if (g_data_count < MAX_POINTS) {
                    g_data[g_data_count++] = g_live_hpa;
                } else {
                    memmove(g_data, g_data + 1, (MAX_POINTS - 1) * sizeof(float));
                    g_data[MAX_POINTS - 1] = g_live_hpa;
                }
            }
        }

        // 2. 渲染到后台 canvas，交换，通知推送（Core 0 与 Core 1 并行）
        displayFrame(0);

        // 3. 渲染完成后立即触发下一次传感器转换（下帧开头读结果）
        if (!g_sensor_converting) {
            sensorTrigger();
            g_sensor_trigger_ms = millis();
            g_sensor_converting = true;
        }
    }
}
