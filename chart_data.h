#pragma once
#include <stdint.h>
#include "config.h"

// 固定容量环形缓冲，带增量 min/max 跟踪。
// dropped 等于 min/max 时退化为全量重算（O(N)），其余 append 为 O(1)。
class ChartData {
public:
    void  reset();
    void  append(float value);
    int   count() const { return count_; }
    float at(int logical_index) const;
    float last() const;
    float min() const { return min_; }
    float max() const { return max_; }

    // 渲染需要直接遍历底层数组以避免每点取模
    int          head() const   { return head_; }
    const float* buffer() const { return data_; }

private:
    void recomputeRange();
    int  physicalIndex(int logical_index) const;

    float data_[MAX_POINTS];
    int   head_  = 0;
    int   count_ = 0;
    float min_   = 0.0f;
    float max_   = 0.0f;
};
