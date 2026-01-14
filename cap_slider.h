#ifndef CAP_SLIDER_H
#define CAP_SLIDER_H

#include <Arduino.h>

class CapSlider
{
public:
    CapSlider(const int *pins, int pinCount);

    // Call once in setup()
    void begin();

    // Continuous movement
    bool read(uint8_t &direction, float &magnitude);

    // Fires once when finger is released
    bool gestureEnded(uint8_t &direction, float &magnitude);

private:
    float readPosition();

    const int *m_pins;
    int m_pinCount;

    // Baseline calibration
    float *m_baseline;

    unsigned long m_lastSampleTime;
    bool m_touchActive;
    bool m_hasPrevSample;

    float m_prevPosition;
    unsigned long m_prevTime;

    float m_startPosition;
    unsigned long m_startTime;

    // ---- Tunables ----
    static constexpr float TOUCH_DROP_PERCENT = 0.30f; // 20% drop = touch
    static constexpr int   CALIBRATION_SAMPLES = 10;
    static constexpr int   CALIBRATION_DELAY_MS = 100;

    static constexpr unsigned long SAMPLE_INTERVAL_MS = 10;
    static constexpr float MAX_VELOCITY = 3.5f;
    static constexpr float DEADZONE = 0.05f;
};

#endif
