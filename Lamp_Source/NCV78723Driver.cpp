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

// Odd parity helper: set parity bit so total number of 1s is odd
static inline uint8_t oddParityBit16(uint16_t w, uint8_t parityBitPos)
{
    w &= ~(1u << parityBitPos);
    uint8_t ones = 0;
    for (int i = 0; i < 16; i++) ones += (w >> i) & 1u;
    return (ones % 2 == 0) ? 1 : 0;
}

// WRITE frame: bit15=1, bits14:11=addr[3:0], bit10 parity, bits9:0 data
uint16_t NCV78723Driver::makeWriteFrame(uint8_t addr4, uint16_t data10)
{
    uint16_t frame = (1u << 15) | ((uint16_t)(addr4 & 0x0F) << 11) | (data10 & 0x03FF);
    uint8_t par = oddParityBit16(frame, 10);
    if (par) frame |=  (1u << 10);
    else     frame &= ~(1u << 10);
    return frame;
}

// READ frame (Format 0 confirmed): bit15=0, bits14:10=addr[4:0], bit9 parity, bits8:0=0
uint16_t NCV78723Driver::makeReadFrame(uint8_t addr5)
{
    uint16_t frame = ((uint16_t)(addr5 & 0x1F) << 10);
    frame &= 0xFE00; // clear bits8:0

    uint8_t par = oddParityBit16(frame, 9);
    if (par) frame |=  (1u << 9);
    else     frame &= ~(1u << 9);

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

// Optional internal read (useful if you want to Serial.printf live status)
uint16_t NCV78723Driver::rd(uint8_t addr5)
{
    return spiXfer16(makeReadFrame(addr5));
}

// ---------- Field packers ----------
uint16_t NCV78723Driver::pack_buck_curr(uint8_t isens, uint8_t vthr)
{
    // 0x01/0x02: bits9:8 = ISENS_THR[1:0], bits7:0 = VTHR[7:0]
    return ((isens & 0x03) << 8) | (vthr & 0xFF);
}

uint16_t NCV78723Driver::pack_toff(uint8_t t1, uint8_t t2)
{
    // 0x03: BUCK1_TOFF[4:0] (bits9:5), BUCK2_TOFF[4:0] (bits4:0)
    return ((t1 & 0x1F) << 5) | (t2 & 0x1F);
}

uint16_t NCV78723Driver::pack_buck_ctrl(uint8_t fso_md, bool en1, bool en2)
{
    // 0x04: bits2:0=FSO_MD, bit1=BUCK1_EN, bit0=BUCK2_EN
    // Upper bits are N78723-2 only; writing them as 0 is safe.
    return ((fso_md & 0x07) << 2) | ((en1 ? 1 : 0) << 1) | (en2 ? 1 : 0);
}

// ---------- Internal control ----------
void NCV78723Driver::setBuckEnabled(Channel ch, bool en)
{
    bool en1 = (ch == CH1) ? en : m_enabled[CH1];
    bool en2 = (ch == CH2) ? en : m_enabled[CH2];

    wr(REG_BUCK_CTRL, pack_buck_ctrl(0x0, en1, en2));

    m_enabled[ch] = en;
    if (!en) m_lastVthr[ch] = 0xFF; // force re-write when re-enabled
}

void NCV78723Driver::setBuckCurrent(Channel ch, uint8_t vthr)
{
    if (vthr == m_lastVthr[ch]) return;

    m_lastVthr[ch] = vthr;

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

    // Analog dimming mode: LEDCTRL pins held HIGH (no PWM gating)
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

    // Configure protections / warning threshold:
    // 0x05: bit9 buck1 auto recover, bit8 buck2 auto recover, bits7:0 warning threshold
    wr(REG_TSD_REC, (1u << 9) | (1u << 8) | 0xB3);

    // Switching timing (keep your existing values unless you have a reason to change)
    wr(REG_TOFF, pack_toff(0x10, 0x10));

    // LED selection duration (harmless; keep)
    wr(REG_LEDSEL_DUR, (8 << 4) | 8);

    // Start both disabled; set() will enable as needed
    m_enabled[CH1] = false;
    m_enabled[CH2] = false;
    wr(REG_BUCK_CTRL, pack_buck_ctrl(0x0, false, false));

    // Start off
    set(CH1, 0.0f);
    set(CH2, 0.0f);
}

void NCV78723Driver::set(Channel ch, float value)
{
    value = constrain(value, 0.0f, 1.0f);
    m_values[ch] = value;

    // Off = disable channel
    if (value <= 0.0f) {
        if (m_enabled[ch]) setBuckEnabled(ch, false);
        return;
    }

    // Ensure enabled
    if (!m_enabled[ch]) setBuckEnabled(ch, true);

    // Map 0..1 to VTHR_MIN_ON..VTHR_MAX
    float span = (float)(VTHR_MAX - VTHR_MIN_ON);
    uint8_t vthr = (uint8_t)(VTHR_MIN_ON + span * value + 0.5f);

    if (vthr < VTHR_MIN_ON) vthr = VTHR_MIN_ON;
    if (vthr > VTHR_MAX)    vthr = VTHR_MAX;

    setBuckCurrent(ch, vthr);
}

float NCV78723Driver::get(Channel ch) const
{
    return m_values[ch];
}
