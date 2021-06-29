#pragma once
#include <stdint.h>

class CH_WaterDetector {

public:
    void init(void);
    void update(void);
    uint8_t read(void);
};
