#include "NCV78723Driver.h"

// SPI: MODE0, 1 MHz
const SPISettings NCV78723Driver::SPI_CFG(1000000, MSBFIRST, SPI_MODE0);

// ---------- SPI helpers ----------
static inline void csLow(int pin)
{
    digitalWrite(pin, LOW);
    delayMicroseconds(1);
}

static inline void csHigh(int pin)
{
    delayMicroseconds(1);
    digitalWrite(pin, HIGH);
    delayMicroseconds(2);
}

uint16_t NCV78723Driver::makeWriteFrame(uint8_t addr4, uint16_t data10)
{
    // WRITE: bit15=1, bits14:11=addr4, bit10=parity, bits9:0=data10
    uint16_t frame = (1u << 15) | ((addr4 & 0x0F) << 11) | (data10 & 0x03FF);
    uint16_t tmp = frame & ~(1u << 10);

    uint8_t parity = 0;
    for (int i = 0; i < 16; i++)
        parity ^= (tmp >> i) & 1u;

    if (parity == 0)
        frame |= (1u << 10);

    return frame;
}

uint16_t NCV78723Driver::spiXfer16(uint16_t tx)
{
    m_spi.beginTransaction(SPI_CFG);
    csLow(PIN_CS);
    uint16_t rx = m_spi.transfer16(tx);
    csHigh(PIN_CS);
    m_spi.endTransaction();
    return rx;
}

void NCV78723Driver::wr(Reg r, uint16_t data10)
{
    spiXfer16(makeWriteFrame((uint8_t)r, data10));
}

// ---------- Field packers ----------
uint16_t NCV78723Driver::pack_buck_curr(uint8_t isens, uint8_t vthr)
{
    // Table 9: bits9:8 = BUCKx_ISENS_THR[1:0], bits7:0 = BUCKx_VTHR[7:0]
    return ((isens & 0x03) << 8) | (vthr & 0xFF);
}

uint16_t NCV78723Driver::pack_toff(uint8_t t1, uint8_t t2)
{
    // Table 9: 0x03 = BUCK1_TOFF[4:0] (bits9:5) | BUCK2_TOFF[4:0] (bits4:0)
    return ((t1 & 0x1F) << 5) | (t2 & 0x1F);
}

uint16_t NCV78723Driver::pack_buck_ctrl(uint8_t fso_md, bool en1, bool en2)
{
    // Table 9: bits2:0 = FSO_MD[2:0], bit1=BUCK1_EN, bit0=BUCK2_EN
    return ((fso_md & 0x07) << 2) | ((en1 ? 1 : 0) << 1) | (en2 ? 1 : 0);
}

// ---------- Internal helpers ----------
void NCV78723Driver::setBuckEnabled(Channel ch, bool en)
{
    // Keep the other channel's state unchanged
    bool en1 = (ch == CH1) ? en : m_enabled[CH1];
    bool en2 = (ch == CH2) ? en : m_enabled[CH2];

    // FSO_MD = 0 is fine for now
    wr(REG_BUCK_CTRL, pack_buck_ctrl(0x0, en1, en2));

    m_enabled[ch] = en;
}

void NCV78723Driver::setBuckCurrent(Channel ch, uint8_t vthr)
{
    if (ch == CH1)
        wr(REG_BUCK1_CURR, pack_buck_curr(ISENS_RANGE, vthr));
    else
        wr(REG_BUCK2_CURR, pack_buck_curr(ISENS_RANGE, vthr));
}

// ---------- Public API ----------
void NCV78723Driver::begin()
{
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);

    pinMode(PIN_RSTB, OUTPUT);

    // For analog dimming, hold LEDCTRL pins HIGH (no external PWM gating)
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, HIGH);

    m_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

    // Reset NCV78723
    digitalWrite(PIN_RSTB, LOW);
    delay(5);
    digitalWrite(PIN_RSTB, HIGH);
    delay(5);

    // ---- Configure device ----

    // 0x05: bit9 BUCK1_TSD_AUT_RCVR_EN, bit8 BUCK2_TSD_AUT_RCVR_EN, bits7:0 THERMAL_WARNING_THR
    // Enable auto recovery for BOTH bucks, keep a sane warning threshold (0xB3).
    wr(REG_TSD_REC, (1u << 9) | (1u << 8) | 0xB3);

    // Keep your TOFF values
    wr(REG_TOFF, pack_toff(0x10, 0x10));

    // Enable both bucks, but start "off" via set(...,0) (we will disable channels at 0)
    m_enabled[CH1] = false;
    m_enabled[CH2] = false;
    wr(REG_BUCK_CTRL, pack_buck_ctrl(0x0, false, false));

    // LED selection duration (not used for analog dimming directly, but harmless)
    wr(REG_LEDSEL_DUR, (8 << 4) | 8);

    // Start outputs off
    set(CH1, 0.0f);
    set(CH2, 0.0f);
}

void NCV78723Driver::set(Channel ch, float value)
{
    value = constrain(value, 0.0f, 1.0f);
    m_values[ch] = value;

    // True off: disable the buck channel
    if (value <= 0.0f) {
        if (m_enabled[ch]) setBuckEnabled(ch, false);
        return;
    }

    // Ensure enabled when >0
    if (!m_enabled[ch]) {
        setBuckEnabled(ch, true);
    }

    // Map 0..1 to VTHR_MIN_ON..VTHR_MAX
    uint8_t vthr;
    if (VTHR_MAX >= VTHR_MIN_ON) {
        float span = (float)(VTHR_MAX - VTHR_MIN_ON);
        vthr = (uint8_t)(VTHR_MIN_ON + span * value + 0.5f);
    } else {
        vthr = VTHR_MAX;
    }

    if (vthr < VTHR_MIN_ON) vthr = VTHR_MIN_ON;
    if (vthr > VTHR_MAX)    vthr = VTHR_MAX;

    setBuckCurrent(ch, vthr);
}

float NCV78723Driver::get(Channel ch) const
{
    return m_values[ch];
}
