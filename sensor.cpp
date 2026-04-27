#include "sensor.h"
#include "config.h"
#include <Wire.h>

namespace {

bool s_converting = false;
uint32_t s_trigger_ms = 0;

bool triggerHardware() {
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(0x30);
    Wire.write(0x0A);
    return Wire.endTransmission() == 0;
}

float readResultHardware() {
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

} // namespace

void PressureSensor::begin() {
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    s_converting = false;
}

bool PressureSensor::startConversion() {
    if (!triggerHardware()) {
        s_converting = false;
        return false;
    }
    s_trigger_ms = millis();
    s_converting = true;
    return true;
}

bool PressureSensor::isConverting() {
    return s_converting;
}

bool PressureSensor::readResultIfReady(float* hpa_out) {
    if (!s_converting) return false;
    if (millis() - s_trigger_ms < SENSOR_CONV_MS) return false;
    float v = readResultHardware();
    s_converting = false;
    if (hpa_out) *hpa_out = v;
    return true;
}

void PressureSensor::cancel() {
    s_converting = false;
}
