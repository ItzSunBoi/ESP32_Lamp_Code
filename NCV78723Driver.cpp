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
    return ((isens & 0x03) << 8) | (vthr & 0xFF);
}

uint16_t NCV78723Driver::pack_toff(uint8_t t1, uint8_t t2)
{
    return ((t1 & 0x1F) << 5) | (t2 & 0x1F);
}

uint16_t NCV78723Driver::pack_buck_ctrl(uint8_t fso, bool en1, bool en2)
{
    return ((fso & 0x07) << 2) | ((en1 ? 1 : 0) << 1) | (en2 ? 1 : 0);
}

// ---------- LEDC ----------
void NCV78723Driver::ledcWriteDuty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

// ---------- Public API ----------
void NCV78723Driver::begin()
{
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_RSTB, OUTPUT);

    m_spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

    ledc_timer_config_t t = {};
    t.speed_mode = LEDC_MODE;
    t.timer_num = LEDC_TIMER;
    t.duty_resolution = LEDC_BITS;
    t.freq_hz = LEDC_FREQ;
    t.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&t);

    ledc_channel_config_t c1 = {};
    c1.gpio_num = PIN_LED1;
    c1.speed_mode = LEDC_MODE;
    c1.channel = LEDC_CH1;
    c1.timer_sel = LEDC_TIMER;
    ledc_channel_config(&c1);

    ledc_channel_config_t c2 = {};
    c2.gpio_num = PIN_LED2;
    c2.speed_mode = LEDC_MODE;
    c2.channel = LEDC_CH2;
    c2.timer_sel = LEDC_TIMER;
    ledc_channel_config(&c2);

    // Reset NCV78723
    digitalWrite(PIN_RSTB, LOW);
    delay(5);
    digitalWrite(PIN_RSTB, HIGH);
    delay(5);

    // Configure device
    wr(REG_BUCK1_CURR, pack_buck_curr(0x01, 0x70));
    wr(REG_BUCK2_CURR, pack_buck_curr(0x01, 0x70));
    wr(REG_TOFF,       pack_toff(0x10, 0x10));
    wr(REG_BUCK_CTRL,  pack_buck_ctrl(0x0, true, true));
    wr(REG_LEDSEL_DUR, 0x000);
}

void NCV78723Driver::set(Channel ch, float value)
{
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;

    m_values[ch] = value;

    uint32_t duty = (uint32_t)(value * MAX_DUTY);

    if (ch == CH1)
        ledcWriteDuty(LEDC_CH1, duty);
    else
        ledcWriteDuty(LEDC_CH2, duty);
}

float NCV78723Driver::get(Channel ch) const
{
    return m_values[ch];
}
