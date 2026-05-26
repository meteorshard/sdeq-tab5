#pragma once
#include <M5Unified.h>
#include "chart_data.h"

enum AppState { STATE_IDLE, STATE_CALIBRATING, STATE_RUNNING, STATE_PAUSED, STATE_STOPPED };

namespace ChartView {
    void initGeometry();
    void resetTextCache();
    void renderStatusBar(M5Canvas& cv, AppState state, int elapsed_ms);
    void renderCalibrationView(M5Canvas& cv, int elapsed_ms);
    void renderChartStatic(M5Canvas& cv);
    void renderChartDynamic(M5Canvas& cv, const ChartData& data, int view_start);
    void renderResumeButton(M5Canvas& cv);
}
