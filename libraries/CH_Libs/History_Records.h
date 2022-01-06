//
// Created by Yin lanshan on 2022/1/6.
//

#ifndef CH_HISTORY_RECORDS_H
#define CH_HISTORY_RECORDS_H

#include <stdint.h>
#define HISTORY_RECORD_STATE_INCOMPLETE 0
#define HISTORY_RECORD_STATE_DONE       1
#define HISTORY_RECORD_STATE_ERROR      2
#define HISTORY_RECORD_STATE_FULL       3

#define HISTORY_RECORD size;
typedef struct
{
    uint32_t timestamp;
    uint8_t mode;
    uint8_t state;
    uint8_t duration;
    uint8_t reserved;
} history_record_t;

class History_Records
{
public:
    /**
     * save washing task record
     * @param mode task mode.
     */
    static void start_washing_task(int8_t mode);

    /**
     * Update the last task state
     * @param state task new state
     * @param duration task running duration in deci hours
     */
    static void update_washing_task(int8_t state, uint8_t duration);

    /**
     * Read washing records
     * @param offset  first record offset
     * @param total output: total record count
     * @param len input as content buffer max length; output as content length
     * @param content records content buffer
     */
    static void read_washing_records(uint16_t offset, uint16_t *total, uint16_t *len, uint8_t *content);
};

#endif //CH_HISTORY_RECORDS_H
