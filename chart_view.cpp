#include "chart_view.h"
#include "config.h"
#include <math.h>
#include <stdio.h>

namespace {

constexpr float X_STEP = (float)CHART_W / (CHART_VISIBLE_POINTS - 1);

int16_t s_x_pos[CHART_VISIBLE_POINTS];
int     s_cached_display_max = 0;
int     s_cached_display_min = 0;
float   s_cached_label_value = NAN;
char    s_label_max_text[16] = "0";
char    s_label_min_text[16] = "0";
char    s_current_value_text[16] = "0.00";

void updateChartLabelTexts(int display_max_i, int display_min_i, float last_value) {
    if (display_max_i != s_cached_display_max) {
        s_cached_display_max = display_max_i;
        snprintf(s_label_max_text, sizeof(s_label_max_text), "%d", display_max_i);
    }
    if (display_min_i != s_cached_display_min) {
        s_cached_display_min = display_min_i;
        snprintf(s_label_min_text, sizeof(s_label_min_text), "%d", display_min_i);
    }
    // 按显示精度（%.2f → 0.005 阈值）比较，避免 float 抖动每帧重绘文字
    if (isnan(s_cached_label_value) || fabsf(s_cached_label_value - last_value) >= 0.005f) {
        s_cached_label_value = last_value;
        snprintf(s_current_value_text, sizeof(s_current_value_text), "%.2f", last_value);
    }
}

} // namespace

void ChartView::initGeometry() {
    for (int i = 0; i < CHART_VISIBLE_POINTS; i++) {
        s_x_pos[i] = (int16_t)(CHART_X + i * X_STEP);
    }
}

void ChartView::resetTextCache() {
    s_cached_display_max = 0;
    s_cached_display_min = 0;
    s_cached_label_value = NAN;
    snprintf(s_label_max_text, sizeof(s_label_max_text), "0");
    snprintf(s_label_min_text, sizeof(s_label_min_text), "0");
    snprintf(s_current_value_text, sizeof(s_current_value_text), "0.00");
}

void ChartView::renderStatusBar(M5Canvas& cv, AppState state, int elapsed_ms) {
    cv.setTextSize(3);
    cv.setTextColor(COLOR_AXIS, COLOR_BG);
    if (state == STATE_IDLE) {
        cv.drawString("Tap anywhere to Start", 10, 20);
    } else if (state == STATE_STOPPED) {
        cv.drawString("Stopped. Tap anywhere to Start", 10, 20);
    } else if (state == STATE_RUNNING) {
        cv.drawString("Recording...", 10, 20);
    } else if (state == STATE_PAUSED) {
        cv.setTextColor(COLOR_PAUSED, COLOR_BG);
        cv.drawString("Paused. Swipe history", 10, 20);
    } else if (state == STATE_CALIBRATING) {
        int secs_left = (int)((CALIB_DURATION_MS - elapsed_ms + 999) / 1000);
        if (secs_left < 0) secs_left = 0;
        char buf[32];
        snprintf(buf, sizeof(buf), "Calibrating...  %d", secs_left);
        cv.setTextColor(COLOR_CALIB, COLOR_BG);
        cv.drawString(buf, 10, 20);
    }
}

void ChartView::renderCalibrationView(M5Canvas& cv, int elapsed_ms) {
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

void ChartView::renderChartStatic(M5Canvas& cv) {
    cv.drawLine(CHART_X, CHART_Y - CHART_H, CHART_X, CHART_Y, COLOR_AXIS);
    cv.drawLine(CHART_X, CHART_Y, CHART_X + CHART_W, CHART_Y, COLOR_AXIS);
}

void ChartView::renderChartDynamic(M5Canvas& cv, const ChartData& data, int view_start) {
    const int total = data.count();
    if (total < 1) return;

    int max_start = total - CHART_VISIBLE_POINTS;
    if (max_start < 0) max_start = 0;
    if (view_start < 0) view_start = max_start;
    if (view_start > max_start) view_start = max_start;

    int n = total - view_start;
    if (n > CHART_VISIBLE_POINTS) n = CHART_VISIBLE_POINTS;
    if (n < 1) return;

    float window_min = data.at(view_start);
    float window_max = window_min;
    for (int i = 1; i < n; i++) {
        float value = data.at(view_start + i);
        if (value < window_min) window_min = value;
        if (value > window_max) window_max = value;
    }

    float display_min = (window_min > 0.0f) ? 0.0f : window_min;
    float y_range = window_max - display_min;
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

    float prev_value = data.at(view_start);
    int prev_y = (int)(CHART_Y - ((prev_value - display_min) * y_scale));

    for (int i = 0; i < n - 1; i++) {
        float next_value = data.at(view_start + i + 1);
        int next_y = (int)(CHART_Y - ((next_value - display_min) * y_scale));
        cv.drawLine(s_x_pos[i], prev_y, s_x_pos[i + 1], next_y, COLOR_LINE);
        prev_y = next_y;
    }

    float last_value = data.at(view_start + n - 1);
    int last_x = s_x_pos[n - 1];
    int last_y = prev_y;

    cv.fillRect(last_x - 3, last_y - 3, 7, 7, COLOR_CURRENT);

    int display_max_i = (int)display_max;
    int display_min_i = (int)floorf(display_min);
    updateChartLabelTexts(display_max_i, display_min_i, last_value);

    cv.setTextSize(2);
    cv.setTextColor(COLOR_AXIS, COLOR_BG);
    cv.drawString(s_label_max_text, 2, CHART_Y - CHART_H);
    cv.drawString(s_label_min_text, 2, CHART_Y - 20);

    const int bubble_pad_x = 14;
    const int bubble_pad_y = 10;
    const int bubble_gap = 14;
    cv.setTextSize(3);
    int text_w = cv.textWidth(s_current_value_text);
    int text_h = 24;
    int bubble_w = text_w + bubble_pad_x * 2;
    int bubble_h = text_h + bubble_pad_y * 2;

    int bubble_x = last_x + bubble_gap;
    if (bubble_x + bubble_w > CHART_X + CHART_W) {
        bubble_x = last_x - bubble_gap - bubble_w;
    }
    if (bubble_x < CHART_X + 4) bubble_x = CHART_X + 4;

    int bubble_y = last_y - bubble_h / 2;
    if (bubble_y < CHART_Y - CHART_H + 4) bubble_y = CHART_Y - CHART_H + 4;
    if (bubble_y + bubble_h > CHART_Y - 4) bubble_y = CHART_Y - 4 - bubble_h;

    cv.drawRoundRect(bubble_x, bubble_y, bubble_w, bubble_h, 8, COLOR_CURRENT);
    cv.fillRoundRect(bubble_x + 1, bubble_y + 1, bubble_w - 2, bubble_h - 2, 8, COLOR_BG);
    cv.setTextColor(COLOR_CURRENT, COLOR_BG);
    cv.drawString(s_current_value_text, bubble_x + bubble_pad_x, bubble_y + bubble_pad_y);
}

void ChartView::renderResumeButton(M5Canvas& cv) {
    cv.fillRoundRect(RESUME_BTN_X, RESUME_BTN_Y, RESUME_BTN_W, RESUME_BTN_H, 8, COLOR_BUTTON);
    cv.drawRoundRect(RESUME_BTN_X, RESUME_BTN_Y, RESUME_BTN_W, RESUME_BTN_H, 8, COLOR_AXIS);
    cv.setTextSize(3);
    cv.setTextColor(COLOR_AXIS, COLOR_BUTTON);
    const char* label = "Resume";
    int text_x = RESUME_BTN_X + (RESUME_BTN_W - cv.textWidth(label)) / 2;
    int text_y = RESUME_BTN_Y + 16;
    cv.drawString(label, text_x, text_y);
}
