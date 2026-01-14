#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

#include "NCV78723Driver.h"
#include "cap_slider.h"

// ---------------- Pins ----------------
#define TEMP_SENSOR_PIN 26
#define BUTTON_PIN      12

const int touchPins[] = {32, 33, 27, 14, 2, 15, 13};
CapSlider slider(touchPins, 7);

// ---------------- Drivers ----------------
NCV78723Driver leds;
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// ---------------- State ----------------
enum ControlMode { MODE_WARM, MODE_COOL, MODE_GLOBAL };
ControlMode currentMode = MODE_WARM;

float warmLevel = 0.5f;
float coolLevel = 0.5f;
bool lampOn = true;

float warmTarget = warmLevel;
float coolTarget = coolLevel;

unsigned long lastInputTime = 0;

// ---------------- Constants ----------------
static constexpr float SLIDER_MULTIPLIER = 50.0f;     // counts (0..4095) scaled into 0..1
static constexpr float SMOOTHING_ALPHA  = 0.15f;
static constexpr float TEMP_LIMIT_C     = 75.0f;
static constexpr float TEMP_HYST_C      = 10.0f;       // hysteresis
static constexpr uint32_t INACTIVITY_MS = 30000;

// -------- Button capacitive state --------
static float buttonBaseline = 0.0f;
static bool  buttonActive   = false;
static bool  buttonHandled  = false;
static unsigned long buttonPressTime = 0;

// Timing (ms)
static constexpr uint32_t BUTTON_TAP_MAX_MS   = 200;
static constexpr uint32_t BUTTON_HOLD_MIN_MS  = 300;

// Capacitive threshold (same model as slider)
static constexpr float BUTTON_DROP_PERCENT = 0.20f;

// ---------------- Thermal shared state (single-writer LED rule) ----------------
// tempTask updates these, applyLEDs consumes them.
static volatile bool  thermalOverrideActive = false;
static volatile float thermalPulsePhase = 0.0f;   // phase accumulator for pulsing
static volatile float lastTempC = NAN;

// Power ramp state
static bool  powerTargetOn = true;
static float powerLevel   = 1.0f;   // 0.0 → 1.0
static constexpr float POWER_RAMP_ALPHA = 0.12f;

// ---------------- Mode-change feedback ----------------
static bool modeFeedbackActive = false;
static unsigned long modeFeedbackStart = 0;
static ControlMode modeFeedbackMode;

static float savedWarmLevel = 0.0f;
static float savedCoolLevel = 0.0f;

// Timing (ms)
static constexpr uint32_t FEEDBACK_PRE_DELAY  = 500;
static constexpr uint32_t FEEDBACK_PULSE_TIME = 1000; // total pulse window
static constexpr uint32_t FEEDBACK_POST_DELAY = 500;

// Pulse params
static constexpr uint32_t FEEDBACK_PWM = 1000;

// ---------------- EEPROM layout ----------------
struct PersistState {
    float warm;
    float cool;
    bool on;
};

PersistState persisted;

// ---------------- Forward declarations ----------------
void tempTask(void *param);
void IRAM_ATTR dummyTouchISR() {}
void applyLEDs();
void saveState();
float clamp01(float v);
bool isButtonTouched();

// ================= SETUP =================
void setup()
{
    Serial.begin(115200);

    EEPROM.begin(sizeof(PersistState));
    EEPROM.get(0, persisted);

    // Validate EEPROM
    if (isnan(persisted.warm) || isnan(persisted.cool) ||
        persisted.warm < 0.0f || persisted.warm > 1.0f ||
        persisted.cool < 0.0f || persisted.cool > 1.0f)
    {
        persisted.warm = 0.5f;
        persisted.cool = 0.5f;
        persisted.on   = true;
    }

    warmLevel = warmTarget = persisted.warm;
    coolLevel = coolTarget = persisted.cool;
    lampOn    = persisted.on;

    powerTargetOn = lampOn;
    powerLevel    = lampOn ? 1.0f : 0.0f;

    slider.begin();

    // ---- Calibrate capacitive button ----
    buttonBaseline = 0.0f;
    for (int i = 0; i < 10; i++)
    {
        buttonBaseline += touchRead(BUTTON_PIN);
        delay(100);
    }
    buttonBaseline /= 10.0f;

    leds.begin();
    tempSensor.begin();

    // Temperature safety task (Core 0)
    xTaskCreatePinnedToCore(
        tempTask,
        "TempTask",
        4096,
        nullptr,
        1,
        nullptr,
        0
    );

    applyLEDs();

    pinMode(25, OUTPUT);
    digitalWrite(25, HIGH);
}

// ================= MAIN LOOP (Core 1) =================
void loop()
{
    uint8_t dir;
    float mag;
    unsigned long now = millis();

    // =========================================================
    // Slider processing
    // =========================================================
    if (slider.read(dir, mag))
    {
        float delta = (mag * SLIDER_MULTIPLIER) / 4095.0f;
        if (dir == 0) delta = -delta;

        switch (currentMode)
        {
            case MODE_WARM:
                warmTarget += delta;
                break;

            case MODE_COOL:
                coolTarget += delta;
                break;

            case MODE_GLOBAL:
                warmTarget += delta;
                coolTarget += delta;
                break;
        }

        warmTarget = clamp01(warmTarget);
        coolTarget = clamp01(coolTarget);

        lastInputTime = now;
    }

    // =========================================================
    // Gesture end → commit target
    // =========================================================
    if (slider.gestureEnded(dir, mag))
    {
        warmLevel = warmTarget;
        coolLevel = coolTarget;
        saveState();
    }

    // =========================================================
    // Button capacitive FSM
    // =========================================================
    bool touched = isButtonTouched();

    if (touched && !buttonActive)
    {
        buttonActive = true;
        buttonHandled = false;
        buttonPressTime = now;
        lastInputTime = now;
    }

    if (!touched && buttonActive)
    {
        uint32_t pressDuration = now - buttonPressTime;

        if (!buttonHandled && pressDuration <= BUTTON_TAP_MAX_MS)
        {
            // ---- Short tap → toggle lamp (via ramp) ----
            lampOn = !lampOn;
            powerTargetOn = lampOn;
            saveState();
        }

        buttonActive = false;
        buttonHandled = false;
    }

    if (touched && buttonActive && !buttonHandled)
    {
        uint32_t heldTime = now - buttonPressTime;

        if (heldTime >= BUTTON_HOLD_MIN_MS)
        {
            // ---- Tap + Hold → cycle mode ----
            currentMode = static_cast<ControlMode>((currentMode + 1) % 3);

            // Start mode-change feedback
            modeFeedbackActive = true;
            modeFeedbackStart  = millis();
            modeFeedbackMode   = currentMode;

            // Save current state
            savedWarmLevel = warmLevel;
            savedCoolLevel = coolLevel;

            // Reset slider reference
            warmTarget = warmLevel;
            coolTarget = coolLevel;

            buttonHandled = true;
            lastInputTime = now;
        }
    }

    // =========================================================
    // Smooth channel levels
    // =========================================================
    warmLevel += (warmTarget - warmLevel) * SMOOTHING_ALPHA;
    coolLevel += (coolTarget - coolLevel) * SMOOTHING_ALPHA;

    // =========================================================
    // Power ramp (THIS WAS MISSING)
    // =========================================================
    float pTarget = powerTargetOn ? 1.0f : 0.0f;
    powerLevel += (pTarget - powerLevel) * POWER_RAMP_ALPHA;

    if (powerLevel < 0.001f) powerLevel = 0.0f;
    if (powerLevel > 0.999f) powerLevel = 1.0f;

    // =========================================================
    // Apply outputs ONCE (single writer)
    // =========================================================
    applyLEDs();   // must multiply channels by powerLevel internally

    // =========================================================
    // Inactivity → Light Sleep (only when OFF)
    // =========================================================
    if (!lampOn && (now - lastInputTime > INACTIVITY_MS))
    {
        uint32_t threshold =
            (uint32_t)(buttonBaseline * (1.0f - BUTTON_DROP_PERCENT));

        // Configure touch wake
        touchAttachInterrupt(BUTTON_PIN, dummyTouchISR, threshold);

        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
        esp_sleep_enable_touchpad_wakeup();

        Serial.println("[SLEEP] Entering light sleep (lamp OFF)");
        esp_light_sleep_start();

        // ---- Resume here after wake ----
        Serial.println("[SLEEP] Woke from light sleep");

        lastInputTime = millis();
    }



    delay(10);
}

// ================= TEMP TASK =================
void tempTask(void *param)
{
    for (;;)
    {
        // Only bother monitoring aggressively when lamp is ON.
        // (Still safe if you monitor always; this reduces pointless OneWire traffic.)
        if (lampOn)
        {
            tempSensor.requestTemperatures();
            float t = tempSensor.getTempCByIndex(0);

            // Serial.println(t);
            // Handle sensor missing (DEVICE_DISCONNECTED_C is usually -127)
            lastTempC = t;

            if (t == DEVICE_DISCONNECTED_C)
            {
                // Fail-safe choice: enable thermal override if sensor missing
                thermalOverrideActive = true;
            }
            else
            {
                // Hysteresis around TEMP_LIMIT_C
                if (!thermalOverrideActive && t > TEMP_LIMIT_C)
                    thermalOverrideActive = true;
                else if (thermalOverrideActive && t < (TEMP_LIMIT_C - TEMP_HYST_C))
                    thermalOverrideActive = false;
            }

            // Update pulse phase continuously while overriding
            if (thermalOverrideActive)
            {
                float p = thermalPulsePhase;
                p += 0.10f;                 // pulse speed
                if (p > 100000.0f) p = 0.0f;
                thermalPulsePhase = p;
            }
        }
        else
        {
            // When lamp is OFF, no need for thermal override
            thermalOverrideActive = false;
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ================= HELPERS =================
void applyLEDs()
{
    unsigned long now = millis();

    // ================= Mode-change feedback =================
    if (modeFeedbackActive)
    {
        uint32_t elapsed = now - modeFeedbackStart;

        // Phase 1: pre-delay (0.5 s)
        if (elapsed < FEEDBACK_PRE_DELAY)
        {
            if (modeFeedbackMode == MODE_WARM)
            {
                leds.set(NCV78723Driver::CH1, clamp01(savedWarmLevel) * powerLevel);
                leds.set(NCV78723Driver::CH2, 0.0f);
            }
            else if (modeFeedbackMode == MODE_COOL)
            {
                leds.set(NCV78723Driver::CH1, 0.0f);
                leds.set(NCV78723Driver::CH2, clamp01(savedCoolLevel) * powerLevel);
            }
            else // MODE_GLOBAL
            {
                leds.set(NCV78723Driver::CH1, 0.0f);
                leds.set(NCV78723Driver::CH2, 0.0f);
            }
            return;
        }

        // Phase 2: pulse window (1 second, 2 pulses)
        elapsed -= FEEDBACK_PRE_DELAY;
        if (elapsed < FEEDBACK_PULSE_TIME)
        {
            // Two pulses in 1 second => 4 edges => 250 ms per half-cycle
            bool on = ((elapsed / 250) % 2) == 0;

            float v = on ? (FEEDBACK_PWM / 4095.0f) : 0.0f;

            if (modeFeedbackMode == MODE_WARM)
            {
                leds.set(NCV78723Driver::CH1, v * powerLevel);
                leds.set(NCV78723Driver::CH2, 0.0f);
            }
            else if (modeFeedbackMode == MODE_COOL)
            {
                leds.set(NCV78723Driver::CH1, 0.0f);
                leds.set(NCV78723Driver::CH2, v * powerLevel);
            }
            else // MODE_GLOBAL
            {
                leds.set(NCV78723Driver::CH1, v * powerLevel);
                leds.set(NCV78723Driver::CH2, v * powerLevel);
            }
            return;
        }

        // Phase 3: post-delay (0.5 s)
        elapsed -= FEEDBACK_PULSE_TIME;
        if (elapsed < FEEDBACK_POST_DELAY)
        {
            leds.set(NCV78723Driver::CH1, clamp01(savedWarmLevel) * powerLevel);
            leds.set(NCV78723Driver::CH2, clamp01(savedCoolLevel) * powerLevel);
            return;
        }

        // Done → restore normal operation
        modeFeedbackActive = false;
    }

    // Thermal override still wins
    if (thermalOverrideActive)
    {
        // 1 Hz pulse: one full sine cycle per second
        float t = millis() * 0.001f;              // seconds
        // float dim = 0.05f + 0.05f * sinf(TWO_PI * 1.0f * t);

        float s = 0.5f + 0.5f * sinf(TWO_PI * t);
        float dim = 0.03f + 0.07f * s * s;


        leds.set(NCV78723Driver::CH2, 0.0f);
        leds.set(NCV78723Driver::CH1, clamp01(dim) * powerLevel);
        return;
    }


    // Normal operation with power ramp applied
    leds.set(NCV78723Driver::CH1, clamp01(warmLevel) * powerLevel);
    leds.set(NCV78723Driver::CH2, clamp01(coolLevel) * powerLevel);
}


void saveState()
{
    persisted.warm = clamp01(warmLevel);
    persisted.cool = clamp01(coolLevel);
    persisted.on   = lampOn;

    EEPROM.put(0, persisted);
    EEPROM.commit();
}

float clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

bool isButtonTouched()
{
    float v = touchRead(BUTTON_PIN);
    float threshold = buttonBaseline * (1.0f - BUTTON_DROP_PERCENT);
    return (v < threshold);
}
