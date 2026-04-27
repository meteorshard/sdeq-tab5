/*
 * eq_tab5.ino
 * 气压传感器实时折线图显示
 *
 * 硬件：M5Stack Tab5 (ESP32-P4)
 * 传感器：I2C 压力传感器，地址 0x6D，接 Grove 口 (SDA=GPIO53, SCL=GPIO54)
 * 依赖：M5Unified (>=0.2.7)、M5GFX (>=0.2.8)
 *
 * 模块划分：
 *   - config.h        : 引脚 / 量程 / 布局 / 颜色 / 时序常量
 *   - sensor.{h,cpp}  : 非阻塞 I2C 气压传感器驱动
 *   - chart_data.*    : 环形缓冲 + 增量 min/max
 *   - chart_view.*    : 状态栏 / 校准条 / 折线图绘制（含文本缓存）
 *   - eq_tab5.ino     : 状态机 + 双 Canvas ping-pong 调度
 *
 * 渲染策略：
 *   - Display r=0；Canvas 720×1280 + setRotation(1) → 逻辑 1280×720 横屏
 *   - pushSprite 走 r=0 memcpy 快速路径，无格式转换
 *   - 双 Canvas ping-pong + FreeRTOS：Core 1 渲染后台 canvas，
 *     Core 0 推送前台 canvas，两核并行
 */

#include <M5Unified.h>
#include "config.h"
#include "sensor.h"
#include "chart_data.h"
#include "chart_view.h"

// ── 状态 ──────────────────────────────────────────────────
AppState      g_state          = STATE_IDLE;
ChartData     g_data;
float         g_live_hpa       = 0.0f;
float         g_offset         = 0.0f;
double        g_calib_sum      = 0.0;
int           g_calib_count    = 0;
unsigned long g_calib_start_ms = 0;

uint32_t      g_frame_count    = 0;
unsigned long g_fps_last_ms    = 0;
unsigned long g_last_frame_ms  = 0;
bool          g_frame_dirty    = true;

// ── 双 Canvas ping-pong（PSRAM 分配 1.84MB × 2）──────────
M5Canvas g_canvas[2] = { M5Canvas(&M5.Display), M5Canvas(&M5.Display) };
static int g_back  = 0;
static int g_front = 1;

static SemaphoreHandle_t g_sem_render_done;
static SemaphoreHandle_t g_sem_push_done;

// ── Core 0 推送任务 ──────────────────────────────────────
static void taskPush(void*) {
    for (;;) {
        xSemaphoreTake(g_sem_render_done, portMAX_DELAY);
        g_canvas[g_front].pushSprite(0, 0);
        xSemaphoreGive(g_sem_push_done);
        vTaskDelay(1);  // 让出 1ms，避免 IDLE0 饥饿触发 task watchdog
    }
}

static inline void markFrameDirty() { g_frame_dirty = true; }

static bool shouldRender(uint32_t now_ms, uint32_t interval_ms) {
    return g_frame_dirty && now_ms - g_last_frame_ms >= interval_ms;
}

static void renderToBack(int elapsed_ms) {
    auto& cv = g_canvas[g_back];
    cv.fillScreen(COLOR_BG);
    ChartView::renderStatusBar(cv, g_state, elapsed_ms);

    if (g_state == STATE_CALIBRATING) {
        ChartView::renderCalibrationView(cv, elapsed_ms);
    } else {
        ChartView::renderChartStatic(cv);
        if (g_state == STATE_RUNNING) {
            ChartView::renderChartDynamic(cv, g_data);
        }
    }

    g_frame_count++;
    unsigned long now_ms = millis();
    if (now_ms - g_fps_last_ms >= 1000) {
        Serial.printf("FPS: %lu\n", g_frame_count);
        g_frame_count = 0;
        g_fps_last_ms = now_ms;
    }
}

// 渲染 → 等上一帧推送完 → 交换 → 通知 Core 0 推送新前台
static void displayFrame(int elapsed_ms) {
    renderToBack(elapsed_ms);
    xSemaphoreTake(g_sem_push_done, portMAX_DELAY);
    std::swap(g_back, g_front);
    xSemaphoreGive(g_sem_render_done);
    g_last_frame_ms = millis();
    g_frame_dirty = false;
}

// ── 传感器调度封装 ────────────────────────────────────────
static void ensureSensorRunning() {
    if (PressureSensor::isConverting()) return;
    if (!PressureSensor::startConversion()) {
        Serial.println("sensor trigger failed");
    }
}

// 取一个完成的样本到 *raw_out 并立即触发下一次转换。
// 返回 true 表示拿到有效（非 NaN）的样本。
static bool consumeSensorSample(float* raw_out) {
    float raw = NAN;
    if (!PressureSensor::readResultIfReady(&raw)) return false;
    ensureSensorRunning();
    if (isnan(raw)) return false;
    if (raw_out) *raw_out = raw;
    return true;
}

// ── 状态切换 ──────────────────────────────────────────────
static void resetMeasurement() {
    g_data.reset();
    g_live_hpa = 0.0f;
    PressureSensor::cancel();
    ChartView::resetTextCache();
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
    PressureSensor::cancel();
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

// ── 状态更新 ──────────────────────────────────────────────
static void updateCalibration() {
    ensureSensorRunning();

    float raw = NAN;
    if (consumeSensorSample(&raw)) {
        g_calib_sum += raw;
        g_calib_count++;
    }

    uint32_t now = millis();
    uint32_t elapsed = now - g_calib_start_ms;
    markFrameDirty();  // 倒计时按时间推进，需持续刷新

    if (shouldRender(now, CALIB_FRAME_MS)) {
        displayFrame((int)elapsed);
    }

    if (elapsed < CALIB_DURATION_MS) return;

    g_offset = (g_calib_count > 0) ? (float)(g_calib_sum / g_calib_count) : 0.0f;
    g_state = STATE_RUNNING;
    g_live_hpa = 0.0f;
    PressureSensor::startConversion();
    markFrameDirty();
}

static void updateRunning() {
    uint32_t now = millis();

    float raw = NAN;
    if (consumeSensorSample(&raw)) {
        g_live_hpa = raw - g_offset;
        g_data.append(g_live_hpa);
        markFrameDirty();
    }
    ensureSensorRunning();

    if (shouldRender(now, RUNNING_FRAME_MS)) {
        displayFrame(0);
    }
}

// ──────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    auto cfg = M5.config();
    M5.begin(cfg);

    PressureSensor::begin();
    M5.Display.fillScreen(COLOR_BG);
    ChartView::initGeometry();

    // 物理竖屏 r=0，pushSprite 走 memcpy 快速路径
    M5.Display.setRotation(0);

    for (int i = 0; i < 2; i++) {
        // 必须与 Panel_DSI 的 rgb565_nonswapped 一致，否则每帧逐像素字节交换极慢
        g_canvas[i].setColorDepth(lgfx::rgb565_nonswapped);
        if (!g_canvas[i].createSprite(SCREEN_H, SCREEN_W)) {
            Serial.println("createSprite failed");
            for (;;) delay(1000);
        }
        g_canvas[i].setRotation(1);  // 逻辑 1280 × 720
        g_canvas[i].setTextSize(3);
    }

    g_sem_render_done = xSemaphoreCreateBinary();
    g_sem_push_done   = xSemaphoreCreateBinary();
    if (!g_sem_render_done || !g_sem_push_done) {
        Serial.println("semaphore create failed");
        for (;;) delay(1000);
    }
    xSemaphoreGive(g_sem_push_done);  // 初始无推送进行中

    if (xTaskCreatePinnedToCore(taskPush, "push", 4096, nullptr, 5, nullptr, 0) != pdPASS) {
        Serial.println("push task create failed");
        for (;;) delay(1000);
    }

    displayFrame(0);
}

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
            if (g_frame_dirty) displayFrame(0);
            return;
    }
}
