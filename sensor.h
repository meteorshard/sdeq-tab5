#pragma once
#include <Arduino.h>

// 非阻塞 I2C 气压传感器（地址 0x6D）。
// 调用 startConversion 触发一次 ADC 转换，约 32ms 后 readResultIfReady 返回 hPa。
namespace PressureSensor {
    void begin();
    bool startConversion();
    bool isConverting();
    // 仅当上一次转换完成（已超过 SENSOR_CONV_MS）时返回 true，
    // 此时 *hpa_out 可能为 NAN（I2C 错误），调用方需自行判定。
    bool readResultIfReady(float* hpa_out);
    void cancel();
}
