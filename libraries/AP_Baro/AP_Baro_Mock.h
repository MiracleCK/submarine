#pragma once

#include "AP_Baro_Backend.h"

#if defined(HAL_BARO_ALLOW_INIT_NO_BARO)

class AP_Baro_Mock : public AP_Baro_Backend {
public:
    AP_Baro_Mock(AP_Baro &);

    void update() override;

protected:

    void update_healthy_flag(uint8_t instance) override { _frontend.sensors[instance].healthy = true; }

private:
    uint8_t _instance;
};

#endif
