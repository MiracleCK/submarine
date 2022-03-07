#include "DS18_Sensor.h"
#include <AP_Math/AP_Math.h>

#define DS18_DEGUG 0

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#define PIN_MODE(pin, mode) do{}while(0)
#define PIN_LOW(pin) do{}while(0)
#define PIN_HIGH(pin) do{}while(0)
#define READ_PIN(pin) 1
#else
#define PIN_MODE(pin, mode) palSetLineMode(pin, mode);
#define PIN_LOW(pin) palWriteLine(pin, 0)
#define PIN_HIGH(pin) palWriteLine(pin, 1)
#define READ_PIN(pin) palReadLine(pin)
#endif

#define BIT_BAND_ADDR(ptr) ((uint32_t *)(0x22000000 + (((uint32_t) ptr) - 0x20000000) * 32))

#define ROM_SKIP 0xCC
#define ROM_READ 0x33
#define CMD_CONVERT 0x44
#define CMD_READ_PAD 0xBE
#define CMD_WRITE_PAD 0x4E
#define CMD_LOAD_PAD 0xB8
#define CMD_STORE_PAD 0x48

extern const AP_HAL::HAL &hal;

static DS18_Sensor *_singleton = nullptr;

static GPTConfig tim6_cfg = {
        1000000, //1MHz
        NULL,
        0,
        0
};

extern "C" {
    static void tm_cb(GPTDriver *gptp)
    {
        _singleton->timer_cb(gptp);
    }
}

DS18_Sensor::DS18_Sensor(uint32_t ch_cnt, const ioline_t *channels)
{
    _singleton = this;
    if (ch_cnt > MAX_CHANNEL)
        ch_cnt = MAX_CHANNEL;
    channel_count = ch_cnt;

    bit_index = 0;
    conv_bit_band = BIT_BAND_ADDR(cmds);
    read_bit_band = conv_bit_band + 2 * 8;

    cmds[0] = ROM_SKIP;
    cmds[1] = CMD_CONVERT;
    cmds[2] = ROM_SKIP;
    cmds[3] = CMD_READ_PAD;

    for (int i = 0; i < ch_cnt; ++i)
    {
        pins[i] = channels[i];
        PIN_MODE(pins[i], PAL_MODE_OUTPUT_OPENDRAIN);
        PIN_HIGH(pins[i]);
        buf_bit_bands[i] = BIT_BAND_ADDR(channel_bufs[i]);
    }
    tim6_cfg.callback = tm_cb;
    gptStart(&GPTD6, &tim6_cfg);

    state = STATE_IDLE;
}

void DS18_Sensor::update()
{
    if (state == STATE_IDLE && channel_count > 0)
    {
        bit_index = 0;
        state = STATE_RESET_1;
        tick1 = 0;
        //for (int i = 0; i < channel_count; ++i)
            //temperature[i] = INVALID_TEMPERATURE;

        gptStopTimer(&GPTD6);
        gptStartOneShot(&GPTD6, 20); //20us
    }
}

void DS18_Sensor::timer_cb(GPTDriver *gptp)
{
    switch (state)
    {
        case STATE_RESET_1:
            if (tick1 == 0)
            {
                tick1 = 1;
                all_pin_low();
                gptStartOneShot(&GPTD6, 500); //low for 500us
            } else if (tick1 == 1)
            {
                tick1 = 2;
                all_pin_high();
                gptStartOneShot(&GPTD6, 40); //delay 40us
            } else
            {
                tick1 = 0;
                if (sum_all_pin_in() == channel_count)
                {
                    //没有应答
                    state = STATE_IDLE;
#if DS18_DEGUG
                    hal.shell->printf("No Ans\r\n");
#endif
                    return;
                }
                state = STATE_CONVERT;
                bit_index = 0;
                gptStartOneShot(&GPTD6, 460); //delay 40us
            }
            break;
        case STATE_CONVERT:
            if (write_bit(conv_bit_band[bit_index]))
            {
                if (++bit_index == 2 * 8)
                {
                    state = STATE_WAIT;
                    tick1 = 0;
                    bit_index = 0;
                }
            }
            break;
        case STATE_WAIT:
            if (tick1 == 0)
            {
                tick1 = 1;
                all_pin_high();
                gptStartOneShot(&GPTD6, 10000); //high for 10ms
            } else if (tick1 == 1)
            {
                tick1 = 2;
                all_pin_low();
                gptStartOneShot(&GPTD6, 5); //low for 5us
            } else if (tick1 == 2)
            {
                tick1 = 3;
                all_pin_high();
                gptStartOneShot(&GPTD6, 5); //high for 5us
            } else
            {
                tick1 = 0;
                if (sum_all_pin_in() < channel_count)
                {
                    bit_index++;
                    if (bit_index < 80) //800ms timeout
                    {
                        gptStartOneShot(&GPTD6, 5); //high for 5us
                        break;
                    }
                }
                else
                {
#if DS18_DEGUG
                    hal.shell->printf("Wait %d\r\n", bit_index);
#endif
                }
                bit_index = 0;
                state = STATE_RESET_2;
                gptStartOneShot(&GPTD6, 5); //high for 5us
            }
            break;
        case STATE_RESET_2:
            if (tick1 == 0)
            {
                tick1 = 1;
                all_pin_low();
                gptStartOneShot(&GPTD6, 500); //low for 500us
            } else if (tick1 == 1)
            {
                tick1 = 0;
                state = STATE_READ_CMD;
                bit_index = 0;
                all_pin_high();
                gptStartOneShot(&GPTD6, 500); //high for 500us
            }
            break;
        case STATE_READ_CMD:
            if (write_bit(read_bit_band[bit_index]))
            {
                if (++bit_index == 2 * 8)
                {
                    state = STATE_READ_DATA;
                    tick1 = 0;
                    bit_index = 0;
                }
            }
            break;
        case STATE_READ_DATA:
            if (tick1 == 0)
            {
                tick1 = 1;
                all_pin_low();
                gptStartOneShot(&GPTD6, 5); //low for 5us
            } else if (tick1 == 1)
            {
                tick1 = 2;
                all_pin_high();
                gptStartOneShot(&GPTD6, 5); //high for 5us
            } else if (tick1 == 2)
            {
                tick1 = 3;
                all_pin_load(bit_index);
                gptStartOneShot(&GPTD6, 55); //high for 55us
            } else
            {
                tick1 = 0;
                ++bit_index;
                if (bit_index >= 8 * 5)
                {
                    state = STATE_IDLE;
                    bit_index = 0;

                    for (int i = 0; i < channel_count; ++i)
                    {
                        int16_t *p = (int16_t *)channel_bufs[i];
                        float t = (*p)/16.0f;
                        if (t < -30 || t > 80)
                            return;
                        if (temperature[i] > INVALID_TEMPERATURE) {
                            float dt = fabsf(temperature[i] - t);
                            if (is_equal(t, -0.0625f) && dt > 1)
                                return;
                            if (dt < 3)
                                temperature[i] = t;
                            else {
                                temperature[i] = 0.05f*t + 0.95f*temperature[i];
                                hal.shell->printf("Bad temp: %f\r\n", t);
                            }
                            // else reject
                        }
                        else if(t < 85 && t > -30)
                        {
                            temperature[i] = t;
                        }
#if DS18_DEGUG
                        for (int j = 0; j < 5; ++j)
                        {
                            hal.shell->printf("%X ", channel_bufs[i][j]);
                        }
                        hal.shell->printf("%.3f\r\n", temperature[i]);
#endif
                    }
                } else
                {
                    //next bit
                    gptStartOneShot(&GPTD6, 5); //high for 5us
                }
            }

            break;
    }
}

void DS18_Sensor::all_pin_low(void)
{
    for (int i = 0; i < channel_count; ++i)
        PIN_LOW(pins[i]);
}

void DS18_Sensor::all_pin_high(void)
{
    for (int i = 0; i < channel_count; ++i)
        PIN_HIGH(pins[i]);
}

void DS18_Sensor::all_pin_load(uint32_t bit_offset)
{
    for (int i = 0; i < channel_count; ++i)
    {
        buf_bit_bands[i][bit_offset] = READ_PIN(pins[i]);
    }
}

uint32_t DS18_Sensor::sum_all_pin_in(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < channel_count; ++i)
    {
        sum += READ_PIN(pins[i]);
    }
    return sum;
}

bool DS18_Sensor::write_bit(uint32_t bit)
{
    if (tick1 == 0)
    {
        all_pin_low();
        if (bit)
        {
            gptStartOneShot(&GPTD6, 5); //low for 5us
            tick1 = 1;
        } else
        {
            gptStartOneShot(&GPTD6, 60); //low for 60us
            tick1 = 2;
        }
        return false;
    }

    if (tick1 == 1)
    {
        gptStartOneShot(&GPTD6, 60); //high for 60us
    } else
    {
        gptStartOneShot(&GPTD6, 5); //high for 5us
    }
    tick1 = 0;
    all_pin_high();
    return true;
}