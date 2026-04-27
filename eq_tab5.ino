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
// UI 刷新节流：仅在有变化时按该上限刷新
#define RUNNING_FRAME_MS  16
#define CALIB_FRAME_MS    33

// ── 状态机 ────────────────────────────────────────────────
enum AppState { STATE_IDLE, STATE_CALIBRATING, STATE_RUNNING, STATE_STOPPED };

AppState      g_state          = STATE_IDLE;
float         g_data[MAX_POINTS];
int16_t       g_x_pos[MAX_POINTS];
int           g_data_head      = 0;
int           g_data_count     = 0;
float         g_data_min       = 0.0f;
float         g_data_max       = 0.0f;
float         g_live_hpa       = 0.0f;
int           g_cached_display_max = 0;
int           g_cached_display_min = 0;
float         g_cached_label_value = NAN;
char          g_label_max_text[16] = "0";
char          g_label_min_text[16] = "0";
char          g_current_value_text[16] = "0.00";

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
unsigned long g_last_frame_ms  = 0;
bool          g_frame_dirty    = true;

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

static inline int dataIndex(int logical_index) {
    return (g_data_head + logical_index) % MAX_POINTS;
}

static float dataAt(int logical_index) {
    return g_data[dataIndex(logical_index)];
}

static float dataLast() {
    return g_data[dataIndex(g_data_count - 1)];
}

static void recomputeDataRange() {
    if (g_data_count <= 0) {
        g_data_min = 0.0f;
        g_data_max = 0.0f;
        return;
    }

    float cur_min = dataAt(0);
    float cur_max = cur_min;
    for (int i = 1; i < g_data_count; i++) {
        float value = dataAt(i);
        if (value < cur_min) cur_min = value;
        if (value > cur_max) cur_max = value;
    }
    g_data_min = cur_min;
    g_data_max = cur_max;
}

static void appendData(float value) {
    if (g_data_count < MAX_POINTS) {
        g_data[dataIndex(g_data_count)] = value;
        g_data_count++;
        if (g_data_count == 1) {
            g_data_min = value;
            g_data_max = value;
        } else {
            if (value < g_data_min) g_data_min = value;
            if (value > g_data_max) g_data_max = value;
        }
        return;
    }

    float dropped = g_data[g_data_head];
    g_data[g_data_head] = value;
    g_data_head = (g_data_head + 1) % MAX_POINTS;
    if (value < g_data_min) g_data_min = value;
    if (value > g_data_max) g_data_max = value;
    if (dropped == g_data_min || dropped == g_data_max) {
        recomputeDataRange();
    }
}

static void resetData() {
    g_data_head = 0;
    g_data_count = 0;
    g_data_min = 0.0f;
    g_data_max = 0.0f;
}

static void markFrameDirty() {
    g_frame_dirty = true;
}

static bool shouldRender(uint32_t now_ms, uint32_t interval_ms) {
    return g_frame_dirty && now_ms - g_last_frame_ms >= interval_ms;
}

static void resetMeasurement() {
    resetData();
    g_live_hpa = 0.0f;
    g_sensor_converting = false;
    g_cached_display_max = 0;
    g_cached_display_min = 0;
    g_cached_label_value = NAN;
    snprintf(g_label_max_text, sizeof(g_label_max_text), "0");
    snprintf(g_label_min_text, sizeof(g_label_min_text), "0");
    snprintf(g_current_value_text, sizeof(g_current_value_text), "0.00");
}

static void initChartGeometry() {
    for (int i = 0; i < MAX_POINTS; i++) {
        g_x_pos[i] = (int16_t)(CHART_X + i * X_STEP);
    }
}

static void updateChartLabelTexts(int display_max_i, int display_min_i, float last_value) {
    if (display_max_i != g_cached_display_max) {
        g_cached_display_max = display_max_i;
        snprintf(g_label_max_text, sizeof(g_label_max_text), "%d", display_max_i);
    }
    if (display_min_i != g_cached_display_min) {
        g_cached_display_min = display_min_i;
        snprintf(g_label_min_text, sizeof(g_label_min_text), "%d", display_min_i);
    }
    if (isnan(g_cached_label_value) || g_cached_label_value != last_value) {
        g_cached_label_value = last_value;
        snprintf(g_current_value_text, sizeof(g_current_value_text), "%.2f", last_value);
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
    if (adc_val >= 0x800000L) adc_val -= 0x1000000L;

    float pressure_pa = (adc_val / 2097152.0f) * (P_MAX - P_MIN) + P_MIN;
    return pressure_pa / 100.0f;
}

static bool startSensorConversion() {
    if (!sensorTrigger()) {
        Serial.println("sensorTrigger failed");
        return false;
    }
    g_sensor_trigger_ms = millis();
    g_sensor_converting = true;
    return true;
}

static bool readCompletedRawSample(float* raw_out) {
    if (!g_sensor_converting) return false;

    uint32_t now = millis();
    if (now - g_sensor_trigger_ms < SENSOR_CONV_MS) return false;

    float raw = sensorReadResult();
    g_sensor_converting = false;

    if (!startSensorConversion()) {
        markFrameDirty();
    }

    if (isnan(raw)) return false;

    if (raw_out) {
        *raw_out = raw;
    }
    return true;
}

static bool consumeSensorSample() {
    float raw = NAN;
    if (!readCompletedRawSample(&raw)) return false;

    if (isnan(raw)) return false;

    g_live_hpa = raw - g_offset;
    appendData(g_live_hpa);
    markFrameDirty();
    return true;
}

static void renderStatusBar(M5Canvas& cv, int elapsed_ms) {
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
}

static void renderCalibrationView(M5Canvas& cv, int elapsed_ms) {
    int bar_w = CHART_W;
    int bar_x = (SCREEN_W - bar_w) / 2;
    int bar_y = SCREEN_H / 2 - 10;
    int bar_h = 20;
    int filled = (int)((float)elapsed_ms / CALIB_DURATION_MS * bar_w);
    if (filled > bar_w) filled = bar_w;
    cv.drawRect(bar_x, bar_y, bar_w, bar_h, COLOR_AXIS);
    if (filled > 1) {
        cv.fillRect(bar_x + 1, bar_y + 1, filled - 1, bar_h - 2, COLOR_CALIB);
    }
}

static void renderChartStatic(M5Canvas& cv) {
    // ── 坐标轴 ──
    cv.drawLine(CHART_X, CHART_Y - CHART_H, CHART_X, CHART_Y, COLOR_AXIS);
    cv.drawLine(CHART_X, CHART_Y, CHART_X + CHART_W, CHART_Y, COLOR_AXIS);
}

static void renderChartDynamic(M5Canvas& cv) {
    // ── 折线图（仅 RUNNING 且有数据时）──
    if (g_state != STATE_RUNNING || g_data_count < 1) return;

    float display_min = (g_data_min > 0.0f) ? 0.0f : g_data_min;
    float y_range     = g_data_max - display_min;
    if (y_range < 100.0f) y_range = 100.0f;
    float display_max = ceilf(display_min + y_range);
    y_range = display_max - display_min;
    float y_scale = (float)CHART_H / y_range;

    if (display_min < 0.0f) {
        int y_zero = (int)(CHART_Y - (0.0f - display_min) * y_scale);
        cv.drawLine(CHART_X, y_zero, CHART_X + CHART_W, y_zero, COLOR_ZERO);
        cv.setTextSize(2);
        cv.setTextColor(COLOR_AXIS, COLOR_BG);
        cv.drawString("0", 2, y_zero - 12);
    }

    float last_value = dataLast();
    int last_y = (int)(CHART_Y - ((last_value - display_min) * y_scale));
    int last_x = g_x_pos[g_data_count - 1];

    if (g_data_count > 1) {
        int idx = g_data_head;
        float prev_value = g_data[idx];
        int prev_y = (int)(CHART_Y - ((prev_value - display_min) * y_scale));

        for (int i = 0; i < g_data_count - 2; i++) {
            idx++;
            if (idx == MAX_POINTS) idx = 0;
            float next_value = g_data[idx];
            int next_y = (int)(CHART_Y - ((next_value - display_min) * y_scale));
            cv.drawLine(g_x_pos[i], prev_y, g_x_pos[i + 1], next_y, COLOR_LINE);
            prev_y = next_y;
        }

        cv.drawLine(g_x_pos[g_data_count - 2], prev_y, last_x, last_y, COLOR_LINE);
    }

    cv.fillRect(last_x - 3, last_y - 3, 7, 7, COLOR_CURRENT);

    int display_max_i = (int)display_max;
    int display_min_i = (int)floorf(display_min);
    updateChartLabelTexts(display_max_i, display_min_i, last_value);

    cv.setTextSize(2);
    cv.setTextColor(COLOR_AXIS, COLOR_BG);
    cv.drawString(g_label_max_text, 2, CHART_Y - CHART_H);
    cv.drawString(g_label_min_text, 2, CHART_Y - 20);

    const int bubble_pad_x = 14;
    const int bubble_pad_y = 10;
    const int bubble_gap = 14;
    cv.setTextSize(3);
    int text_w = cv.textWidth(g_current_value_text);
    int text_h = 24;
    int bubble_w = text_w + bubble_pad_x * 2;
    int bubble_h = text_h + bubble_pad_y * 2;

    int bubble_x = last_x + bubble_gap;
    if (bubble_x + bubble_w > CHART_X + CHART_W) {
        bubble_x = last_x - bubble_gap - bubble_w;
    }
    if (bubble_x < CHART_X + 4) {
        bubble_x = CHART_X + 4;
    }

    int bubble_y = last_y - bubble_h / 2;
    if (bubble_y < CHART_Y - CHART_H + 4) {
        bubble_y = CHART_Y - CHART_H + 4;
    }
    if (bubble_y + bubble_h > CHART_Y - 4) {
        bubble_y = CHART_Y - 4 - bubble_h;
    }

    cv.drawRoundRect(bubble_x, bubble_y, bubble_w, bubble_h, 8, COLOR_CURRENT);
    cv.fillRoundRect(bubble_x + 1, bubble_y + 1, bubble_w - 2, bubble_h - 2, 8, COLOR_BG);
    cv.setTextColor(COLOR_CURRENT, COLOR_BG);
    cv.drawString(g_current_value_text, bubble_x + bubble_pad_x, bubble_y + bubble_pad_y);
}

// ──────────────────────────────────────────────────────────
// 渲染整帧到后台 canvas（g_canvas[g_back]），不推送到屏幕
// ──────────────────────────────────────────────────────────
void pushFrame(int elapsed_ms) {
    auto& cv = g_canvas[g_back];
    cv.fillScreen(COLOR_BG);
    renderStatusBar(cv, elapsed_ms);

    if (g_state == STATE_CALIBRATING) {
        renderCalibrationView(cv, elapsed_ms);
    } else {
        renderChartStatic(cv);
        renderChartDynamic(cv);
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
    g_last_frame_ms = millis();
    g_frame_dirty = false;
}

static void startCalibration() {
    g_state = STATE_CALIBRATING;
    g_calib_sum = 0.0;
    g_calib_count = 0;
    g_calib_start_ms = millis();
    resetMeasurement();
    markFrameDirty();
}

static void stopRecording() {
    g_state = STATE_STOPPED;
    g_sensor_converting = false;
    markFrameDirty();
    displayFrame(0);
}

static void handleTouchInput() {
    auto touch = M5.Touch.getDetail();
    if (!touch.wasPressed()) return;

    if (g_state == STATE_IDLE || g_state == STATE_STOPPED) {
        startCalibration();
    } else if (g_state == STATE_RUNNING) {
        stopRecording();
    }
}

static void updateCalibration() {
    if (!g_sensor_converting) {
        startSensorConversion();
    }

    float raw = NAN;
    if (readCompletedRawSample(&raw)) {
        g_calib_sum += raw;
        g_calib_count++;
    }

    uint32_t now = millis();
    uint32_t elapsed = now - g_calib_start_ms;
    markFrameDirty();  // 校准进度和倒计时按时间推进，需持续刷新

    if (shouldRender(now, CALIB_FRAME_MS)) {
        displayFrame((int)elapsed);
    }

    if (elapsed < CALIB_DURATION_MS) return;

    g_offset = (g_calib_count > 0) ? (float)(g_calib_sum / g_calib_count) : 0.0f;
    g_state = STATE_RUNNING;
    g_live_hpa = 0.0f;
    startSensorConversion();
    markFrameDirty();
}

static void updateRunning() {
    uint32_t now = millis();

    consumeSensorSample();
    if (!g_sensor_converting) {
        startSensorConversion();
    }

    if (shouldRender(now, RUNNING_FRAME_MS)) {
        displayFrame(0);
    }
}

// ──────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    auto cfg = M5.config();
    M5.begin(cfg);

    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    M5.Display.fillScreen(COLOR_BG);
    initChartGeometry();

    // 两个 canvas：物理 720×1280，逻辑旋转为 1280×720 横屏
    // Display 保持 r=0，pushSprite 走 memcpy 快速路径
    M5.Display.setRotation(0);  // 物理竖屏 r=0，确保 pushSprite 走 memcpy 快速路径

    for (int i = 0; i < 2; i++) {
        // 必须与 Panel_DSI 的 rgb565_nonswapped 格式一致，否则每帧逐像素字节交换极慢
        g_canvas[i].setColorDepth(lgfx::rgb565_nonswapped);
        if (!g_canvas[i].createSprite(SCREEN_H, SCREEN_W)) {
            Serial.println("createSprite failed");
            for (;;) delay(1000);
        }
        g_canvas[i].setRotation(1);                     // 逻辑 1280 × 720
        g_canvas[i].setTextSize(3);
    }

    g_sem_render_done = xSemaphoreCreateBinary();
    g_sem_push_done   = xSemaphoreCreateBinary();
    if (!g_sem_render_done || !g_sem_push_done) {
        Serial.println("semaphore create failed");
        for (;;) delay(1000);
    }
    xSemaphoreGive(g_sem_push_done);  // 初始状态：无推送进行中

    if (xTaskCreatePinnedToCore(taskPush, "push", 4096, nullptr, 5, nullptr, 0) != pdPASS) {
        Serial.println("push task create failed");
        for (;;) delay(1000);
    }

    displayFrame(0);
}

// ──────────────────────────────────────────────────────────
void loop() {
    M5.update();
    handleTouchInput();

    switch (g_state) {
        case STATE_CALIBRATING:
            updateCalibration();
            return;
        case STATE_RUNNING:
            updateRunning();
            return;
        case STATE_IDLE:
        case STATE_STOPPED:
        default:
            if (g_frame_dirty) {
                displayFrame(0);
            }
            return;
    }
}
