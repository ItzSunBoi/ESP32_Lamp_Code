#ifndef NCV78723_DRIVER_H
#define NCV78723_DRIVER_H

#include <Arduino.h>
#include <SPI.h>
#include "driver/ledc.h"

class NCV78723Driver
{
public:
    enum Channel : uint8_t {
        CH1 = 0,
        CH2 = 1
    };

    // Call once from setup()
    void begin();

    // Set output level (0.0 → 1.0)
    void set(Channel ch, float value);

    // Get last set value (0.0 → 1.0)
    float get(Channel ch) const;

private:
    // ---------- NCV78723 registers ----------
    enum Reg : uint8_t {
        REG_NOP         = 0x00,
        REG_BUCK1_CURR  = 0x01,
        REG_BUCK2_CURR  = 0x02,
        REG_TOFF        = 0x03,
        REG_BUCK_CTRL   = 0x04,
        REG_TSD_REC     = 0x05,
        REG_LEDSEL_DUR  = 0x06
    };

    // ---------- Pins ----------
    static constexpr int PIN_SCLK = 18;
    static constexpr int PIN_MISO = 19;
    static constexpr int PIN_MOSI = 23;
    static constexpr int PIN_CS   = 5;
    static constexpr int PIN_RSTB = 17;
    static constexpr int PIN_LED1 = 21;
    static constexpr int PIN_LED2 = 16;

    // ---------- LEDC ----------
    static constexpr ledc_mode_t      LEDC_MODE  = LEDC_HIGH_SPEED_MODE;
    static constexpr ledc_timer_t     LEDC_TIMER = LEDC_TIMER_0;
    static constexpr ledc_timer_bit_t LEDC_BITS  = LEDC_TIMER_12_BIT;
    static constexpr uint32_t         LEDC_FREQ  = 300;
    static constexpr ledc_channel_t   LEDC_CH1   = LEDC_CHANNEL_0;
    static constexpr ledc_channel_t   LEDC_CH2   = LEDC_CHANNEL_1;

    static constexpr uint32_t MAX_DUTY = (1u << LEDC_BITS) - 1;

    // ---------- SPI ----------
    SPIClass m_spi = SPIClass(VSPI);
    static const SPISettings SPI_CFG;

    // ---------- State ----------
    float m_values[2] = {0.0f, 0.0f};

    // ---------- Low-level helpers ----------
    static uint16_t makeWriteFrame(uint8_t addr4, uint16_t data10);
    static uint16_t pack_buck_curr(uint8_t isens, uint8_t vthr);
    static uint16_t pack_toff(uint8_t t1, uint8_t t2);
    static uint16_t pack_buck_ctrl(uint8_t fso, bool en1, bool en2);

    uint16_t spiXfer16(uint16_t tx);
    void wr(Reg r, uint16_t data10);
    void ledcWriteDuty(ledc_channel_t ch, uint32_t duty);
};

#endif
