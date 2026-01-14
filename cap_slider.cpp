#include "cap_slider.h"

CapSlider::CapSlider(const int *pins, int pinCount)
    : m_pins(pins),
      m_pinCount(pinCount),
      m_baseline(nullptr),
      m_lastSampleTime(0),
      m_touchActive(false),
      m_hasPrevSample(false),
      m_prevPosition(0.0f),
      m_prevTime(0),
      m_startPosition(0.0f),
      m_startTime(0)
{
}

void CapSlider::begin()
{
    m_baseline = new float[m_pinCount];

    // Initialise baseline to zero
    for (int i = 0; i < m_pinCount; i++)
        m_baseline[i] = 0.0f;

    // Calibration loop
    for (int s = 0; s < CALIBRATION_SAMPLES; s++)
    {
        for (int i = 0; i < m_pinCount; i++)
            m_baseline[i] += touchRead(m_pins[i]);

        delay(CALIBRATION_DELAY_MS);
    }

    // Average
    for (int i = 0; i < m_pinCount; i++)
        m_baseline[i] /= CALIBRATION_SAMPLES;
}

float CapSlider::readPosition()
{
    float weightedSum = 0.0f;
    float totalWeight = 0.0f;

    for (int i = 0; i < m_pinCount; i++)
    {
        float v = (float)touchRead(m_pins[i]);
        float threshold = m_baseline[i] * (1.0f - TOUCH_DROP_PERCENT);

        // Touch if value drops sufficiently
        if (v < threshold)
        {
            float w = (threshold - v);  // deeper touch = more weight
            float pos = (float)i / (float)(m_pinCount - 1);

            weightedSum += pos * w;
            totalWeight += w;
        }
    }

    if (totalWeight <= 0.0f)
        return -1.0f;

    return weightedSum / totalWeight;
}

bool CapSlider::read(uint8_t &direction, float &magnitude)
{
    unsigned long now = millis();
    if (now - m_lastSampleTime < SAMPLE_INTERVAL_MS)
        return false;

    m_lastSampleTime = now;

    float pos = readPosition();

    if (pos < 0.0f)
    {
        m_touchActive = false;
        m_hasPrevSample = false;
        magnitude = 0.0f;
        return false;
    }

    if (!m_touchActive)
    {
        m_touchActive = true;
        m_startPosition = pos;
        m_startTime = now;
        m_prevPosition = pos;
        m_prevTime = now;
        m_hasPrevSample = true;
        magnitude = 0.0f;
        return false;
    }

    float dx = pos - m_prevPosition;
    float dt = (float)(now - m_prevTime);

    m_prevPosition = pos;
    m_prevTime = now;

    if (dt <= 0.0f)
        return false;

    float velocity = dx / (dt / 1000.0f);

    direction = (velocity >= 0.0f) ? 1 : 0;

    magnitude = fabs(velocity) / MAX_VELOCITY;
    if (magnitude > 1.0f) magnitude = 1.0f;
    if (magnitude < DEADZONE) magnitude = 0.0f;

    return (magnitude > 0.0f);
}

bool CapSlider::gestureEnded(uint8_t &direction, float &magnitude)
{
    float pos = readPosition();
    if (pos >= 0.0f || !m_touchActive)
        return false;

    m_touchActive = false;

    unsigned long endTime = millis();
    float dx = m_prevPosition - m_startPosition;
    float dt = (float)(endTime - m_startTime);

    if (dt <= 0.0f)
        return false;

    float velocity = dx / (dt / 1000.0f);

    direction = (velocity >= 0.0f) ? 1 : 0;

    magnitude = fabs(velocity) / MAX_VELOCITY;
    if (magnitude > 1.0f) magnitude = 1.0f;
    if (magnitude < DEADZONE) magnitude = 0.0f;

    return (magnitude > 0.0f);
}
