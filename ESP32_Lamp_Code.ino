#include "NCV78723Driver.h"
#include "cap_slider.h"
#include <OneWire.h>
#include <DallasTemperature.h>

#define Temp_Sensor 26  
#define Button_Pin 27

const int touchPins[] = {32, 33, 27, 14, 2, 15, 13};
CapSlider slider(touchPins, 7);

NCV78723Driver leds;

OneWire oneWire(Temp_Sensor);
DallasTemperature tempsensor(&oneWire);

void setup()
{
    Serial.begin(115200);
    slider.begin();
    leds.begin();
    tempsensor.begin();

    pinMode(25, OUTPUT);
    digitalWrite(25, HIGH);
}



void loop()
{
    sensors.requestTemperatures();  // Request temperature from sensor
    float tempC = sensors.getTempCByIndex(0); // Read first sensor

    // leds.set(NCV78723Driver::CH1, 0.25f);
    // leds.set(NCV78723Driver::CH2, 0.75f);
    // delay(1000);
    uint8_t dir;
    float mag;

    // Continuous movement
    slider.read(dir, mag);

    // Gesture finished (finger lifted)
    if (slider.gestureEnded(dir, mag))
    {
        Serial.print("GESTURE END: ");
        Serial.print(dir ? "RIGHT " : "LEFT ");
        Serial.println(mag, 3);
    }
}
