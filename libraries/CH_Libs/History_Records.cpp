#include "History_Records.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>

#define RECORD_SIZE (sizeof(history_record_t))
#define MAX_RECORD_COUNT 150
#define RECORD_MIN_ADDR HAL_STORAGE_SIZE
#define RECORD_INFO_ADDR (RECORD_MIN_ADDR + MAX_RECORD_COUNT*RECORD_SIZE)


extern const AP_HAL::HAL &hal;

void History_Records::start_washing_task(int8_t mode)
{
    AP_HAL::Storage *st = hal.storage;
    uint16_t info;
    st->read_block((void *)&info, RECORD_INFO_ADDR, 2);
    //info MSB 0x8000 indicates record ring is full
    uint16_t index = info&0x7FFF;

    //Bad index, reset it to 0
    if (index >= MAX_RECORD_COUNT)
        index = 0;

    history_record_t record = {0};
    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix);
    record.timestamp = (uint32_t)(time_unix/1000000);
    record.mode = mode;
    record.state = HISTORY_RECORD_STATE_INCOMPLETE;

    ++index;
    uint16_t addr = RECORD_INFO_ADDR - (index*RECORD_SIZE);
    st->write_block(addr, (void *)&record, RECORD_SIZE);
    if (index >= MAX_RECORD_COUNT)
    {
        info = 0x8000; //info MSB 0x8000 indicates record ring is full
    }
    else
    {
        if (info&0x8000)
            info = 0x8000 | index;
        else
            info = index;
    }
    st->write_block(RECORD_INFO_ADDR, (void *)&info, 2);
}

void History_Records::update_washing_task(int8_t state, uint8_t duration)
{
    AP_HAL::Storage *st = hal.storage;
    uint16_t info;
    st->read_block((void *)&info, RECORD_INFO_ADDR, 2);
    //info MSB 0x8000 indicates record ring is full
    uint16_t index = info&0x7FFF;

    //Bad index, reset it to 0
    if (index >= MAX_RECORD_COUNT)
        index = 0;

    uint16_t addr = RECORD_INFO_ADDR - (index*RECORD_SIZE);
    if (index == 0)
    {
        //record ring is full, the previous record is at min addr
        if (info&0x8000)
            addr = RECORD_MIN_ADDR;
        else
            //no records to update
            return;
    }
    history_record_t record;
    record.state = state;
    record.duration = duration;
    int32_t offset = (uint8_t *)&record.state - (uint8_t *)&record;
    st->write_block(addr + offset, (void *)(&(record.state)), 2);
}


void History_Records::read_washing_records(uint16_t offset, uint16_t *total, uint16_t *len, uint8_t *content)
{
    AP_HAL::Storage *st = hal.storage;
    uint16_t info;
    st->read_block((void *)&info, RECORD_INFO_ADDR, 2);
    //info MSB 0x8000 indicates record ring is full
    int16_t index = info&0x7FFF;

    //Bad index, reset it to 0
    if (index >= MAX_RECORD_COUNT)
        index = 0;

    uint16_t max = index;
    if (info&0x8000)
        max = MAX_RECORD_COUNT;
    *total = max;
    if (offset >= max)
    {
        *len = 0;
        return;
    }

    uint16_t request_n = (*len) / RECORD_SIZE;
    uint16_t n = MIN(max - offset, request_n);
    uint16_t size = n*RECORD_SIZE;
    *len = size;

    index = index - offset;
    uint16_t addr;
    if (index <= 0)
        addr = RECORD_MIN_ADDR - (index*RECORD_SIZE);
    else
        addr = RECORD_INFO_ADDR - (index*RECORD_SIZE);

    uint16_t end = addr + size;
    if (end > RECORD_INFO_ADDR)
    {
        st->read_block(content, addr, RECORD_INFO_ADDR - addr);
        st->read_block(content, addr, end - RECORD_INFO_ADDR);
    }
    else
    {
        st->read_block(content, addr, size);
    }
}
