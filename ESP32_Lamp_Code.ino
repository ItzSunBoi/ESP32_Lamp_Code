#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>

#include "NCV78723Driver.h"
#include "cap_slider.h"

// ---------------- Pins ----------------
#define TEMP_SENSOR_PIN 26
#define BUTTON_PIN      27

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
static constexpr float SLIDER_MULTIPLIER = 500.0f;
static constexpr float SMOOTHING_ALPHA  = 0.15f;
static constexpr float TEMP_LIMIT_C     = 75.0f;
static constexpr uint32_t INACTIVITY_MS = 30000;

// -------- Button capacitive state --------
static float buttonBaseline = 0.0f;
static bool  buttonActive   = false;
static bool  buttonHandled  = false;
static unsigned long buttonPressTime = 0;

// Timing (ms)
static constexpr uint32_t BUTTON_TAP_MAX_MS   = 300;
static constexpr uint32_t BUTTON_HOLD_MIN_MS  = 400;

// Capacitive threshold (same model as slider)
static constexpr float BUTTON_DROP_PERCENT = 0.20f;

// ---------------- EEPROM layout ----------------
struct PersistState {
    float warm;
    float cool;
    bool on;
};

PersistState persisted;

// ---------------- Forward declarations ----------------
void tempTask(void *param);
void applyLEDs();
void saveState();
float clamp01(float v);

// ================= SETUP =================
void setup()
{
    Serial.begin(115200);

    EEPROM.begin(sizeof(PersistState));
    EEPROM.get(0, persisted);

    // Validate EEPROM
    if (isnan(persisted.warm) || isnan(persisted.cool)) {
        persisted.warm = 0.5f;
        persisted.cool = 0.5f;
        persisted.on   = true;
    }

    warmLevel = warmTarget = persisted.warm;
    coolLevel = coolTarget = persisted.cool;
    lampOn    = persisted.on;

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
}

// ================= MAIN LOOP (Core 1) =================
void loop()
{
    uint8_t dir;
    float mag;

    // ---- Slider processing ----
    if (slider.read(dir, mag))
    {
        float delta = mag * SLIDER_MULTIPLIER / 4095.0f;
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

        lastInputTime = millis();
    }

    // ---- Gesture end → commit ----
    if (slider.gestureEnded(dir, mag))
    {
        warmLevel = warmTarget;
        coolLevel = coolTarget;
        saveState();
    }

    // ---- Smooth LED update ----
    warmLevel += (warmTarget - warmLevel) * SMOOTHING_ALPHA;
    coolLevel += (coolTarget - coolLevel) * SMOOTHING_ALPHA;

    applyLEDs();

    // ================= Button Capacitive FSM =================
    bool touched = isButtonTouched();

    if (touched && !buttonActive)
    {
        // Button just pressed
        buttonActive = true;
        buttonHandled = false;
        buttonPressTime = millis();
        lastInputTime = millis();
    }

    if (!touched && buttonActive)
    {
        // Button released
        uint32_t pressDuration = millis() - buttonPressTime;

        if (!buttonHandled && pressDuration <= BUTTON_TAP_MAX_MS)
        {
            // ---- Short tap → toggle lamp ----
            lampOn = !lampOn;
            applyLEDs();
            saveState();
        }

        buttonActive = false;
        buttonHandled = false;
    }

    if (touched && buttonActive && !buttonHandled)
    {
        uint32_t heldTime = millis() - buttonPressTime;

        if (heldTime >= BUTTON_HOLD_MIN_MS)
        {
            // ---- Tap + Hold → cycle mode ----
            currentMode = static_cast<ControlMode>((currentMode + 1) % 3);

            // Optional: reset slider reference to avoid jumps
            warmTarget = warmLevel;
            coolTarget = coolLevel;

            buttonHandled = true;
            lastInputTime = millis();
        }
    }

    // ================= Inactivity → Light Sleep =================
    if (!lampOn && (millis() - lastInputTime > INACTIVITY_MS))
    {
        // Configure wake source: capacitive touch only
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
        tempSensor.requestTemperatures();
        float tempC = tempSensor.getTempCByIndex(0);

        if (tempC > TEMP_LIMIT_C)
        {
            // Thermal override
            leds.set(NCV78723Driver::CH2, 0.0f);

            static float pulse = 0.0f;
            pulse += 0.02f;
            float dim = 0.05f + 0.05f * sin(pulse);

            leds.set(NCV78723Driver::CH1, dim);
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

// ================= HELPERS =================
void applyLEDs()
{
    if (!lampOn)
    {
        leds.set(NCV78723Driver::CH1, 0.0f);
        leds.set(NCV78723Driver::CH2, 0.0f);
        return;
    }

    leds.set(NCV78723Driver::CH1, warmLevel);
    leds.set(NCV78723Driver::CH2, coolLevel);
}

void saveState()
{
    persisted.warm = warmLevel;
    persisted.cool = coolLevel;
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

static void enterLightSleepIfAllowed()
{
    // Only sleep if lamp is OFF
    if (lampOn)
        return;

    // Configure wake source: capacitive touch only
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_sleep_enable_touchpad_wakeup();

    // NOTE:
    // Touch threshold already configured by touchRead baseline logic
    // We do NOT change LED state here

    Serial.println("[SLEEP] Entering light sleep (lamp OFF)");

    esp_light_sleep_start();

    // Execution resumes here after wake
    Serial.println("[SLEEP] Woke from light sleep");
}
