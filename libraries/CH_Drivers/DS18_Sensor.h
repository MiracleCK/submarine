#pragma once
#include <stdint.h>
#include <AP_HAL/AP_HAL.h>

/*
 * DS18B20 digital temperature sensor
 * Driver is implemented with Timer interrupts
 * Create by Yinlanshan 21-8-12
 */

#define INVALID_TEMPERATURE -100.0f

#define MAX_CHANNEL 4
#define CHANNEL_BUF_SIZE 8

#define STATE_IDLE          0
#define STATE_RESET_1       1
#define STATE_CONVERT       2
#define STATE_WAIT          3
#define STATE_RESET_2       4
#define STATE_READ_CMD      5
#define STATE_READ_DATA     6


class DS18_Sensor
{
public:
    DS18_Sensor(uint32_t channel_count, const ioline_t *channels);
    void update();
    void timer_cb(GPTDriver *gptp);
    uint32_t sample_time(void)
    {
        return sample_ts;
    }
    float read(uint32_t channel)
    {
        if (channel >= MAX_CHANNEL)
            return INVALID_TEMPERATURE;
        return temperature[channel];
    }
private:
    ioline_t pins[MAX_CHANNEL];
    uint8_t channel_count = 0;
    uint32_t sample_ts = 0;
    float temperature[MAX_CHANNEL] = {INVALID_TEMPERATURE}; // degrees C
    uint8_t state = STATE_IDLE;
    uint8_t tick1;
    uint8_t bit_index = 0;

    uint8_t cmds[4];
    uint8_t channel_bufs[MAX_CHANNEL][CHANNEL_BUF_SIZE];
    uint32_t *conv_bit_band, *read_bit_band;
    uint32_t *buf_bit_bands[MAX_CHANNEL];

    void all_pin_low(void);
    void all_pin_high(void);
    void  all_pin_load(uint32_t bit_offset);
    uint32_t sum_all_pin_in(void);
    bool write_bit(uint32_t bit);
};
