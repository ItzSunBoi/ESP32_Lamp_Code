#ifndef NCV78723_DRIVER_H
#define NCV78723_DRIVER_H

#include <Arduino.h>
#include <SPI.h>

class NCV78723Driver
{
public:
    enum Channel : uint8_t {
        CH1 = 0,
        CH2 = 1
    };

    // Call once from setup()
    void begin();

    // Set output level (0.0 → 1.0) using ANALOG dimming (SPI current programming)
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

    // LEDCTRL pins (hold HIGH for analog dimming)
    static constexpr int PIN_LED1 = 21;
    static constexpr int PIN_LED2 = 16;

    // ---------- Analog dimming config ----------
    // Per your goal: ~600mA max. Use Range 3 (ISENS=0x02) and VTHR_MAX ~0x9E.
    static constexpr uint8_t ISENS_RANGE = 0x02; // BUCKx_ISENS_THR[1:0]
    static constexpr uint8_t VTHR_MAX    = 0x9E; // BUCKx_VTHR[7:0] @ ~600mA typ (Range 3)
    static constexpr uint8_t VTHR_MIN_ON = 0x00; // min current when "on" (can raise if you want)

    // ---------- SPI ----------
    SPIClass m_spi = SPIClass(VSPI);
    static const SPISettings SPI_CFG;

    // ---------- State ----------
    float m_values[2] = {0.0f, 0.0f};
    bool  m_enabled[2] = {false, false};

    // ---------- Low-level helpers ----------
    static uint16_t makeWriteFrame(uint8_t addr4, uint16_t data10);
    static uint16_t pack_buck_curr(uint8_t isens, uint8_t vthr);
    static uint16_t pack_toff(uint8_t t1, uint8_t t2);
    static uint16_t pack_buck_ctrl(uint8_t fso_md, bool en1, bool en2);

    uint16_t spiXfer16(uint16_t tx);
    void wr(Reg r, uint16_t data10);

    void setBuckEnabled(Channel ch, bool en);
    void setBuckCurrent(Channel ch, uint8_t vthr);
};

#endif
