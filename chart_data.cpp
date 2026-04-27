#include "chart_data.h"

int ChartData::physicalIndex(int logical_index) const {
    return (head_ + logical_index) % MAX_POINTS;
}

float ChartData::at(int logical_index) const {
    return data_[physicalIndex(logical_index)];
}

float ChartData::last() const {
    return data_[physicalIndex(count_ - 1)];
}

void ChartData::reset() {
    head_  = 0;
    count_ = 0;
    min_   = 0.0f;
    max_   = 0.0f;
}

void ChartData::recomputeRange() {
    if (count_ <= 0) {
        min_ = 0.0f;
        max_ = 0.0f;
        return;
    }
    float lo = at(0);
    float hi = lo;
    for (int i = 1; i < count_; i++) {
        float v = at(i);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    min_ = lo;
    max_ = hi;
}

void ChartData::append(float value) {
    if (count_ < MAX_POINTS) {
        data_[physicalIndex(count_)] = value;
        count_++;
        if (count_ == 1) {
            min_ = value;
            max_ = value;
        } else {
            if (value < min_) min_ = value;
            if (value > max_) max_ = value;
        }
        return;
    }

    float dropped = data_[head_];
    data_[head_] = value;
    head_ = (head_ + 1) % MAX_POINTS;
    if (value < min_) min_ = value;
    if (value > max_) max_ = value;
    // dropped 是从数组里取的精确值，与 min_/max_ 是同一来源，可直接 == 比较
    if (dropped == min_ || dropped == max_) recomputeRange();
}
