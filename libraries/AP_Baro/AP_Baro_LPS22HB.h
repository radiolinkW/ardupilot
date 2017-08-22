#pragma once

#include "AP_Baro_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter2p.h>

class AP_Baro_LPS22HB : public AP_Baro_Backend
{
public:
    void update();

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:
    AP_Baro_LPS22HB(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
    virtual ~AP_Baro_LPS22HB(void) {};

    bool _init();

    uint16_t _read_prom_word(uint8_t word);
    uint32_t _read_adc();

    void _timer();

    uint8_t _register_read(uint8_t reg);
     void _register_write(uint8_t reg, uint8_t val);
     void _register_modify(uint8_t reg, uint8_t clearbits, uint8_t setbits);
     void _dump_registers();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t _instance;

    int accum_count;
    float _pressure;
    float _temperature;
    bool _has_sample;
    LowPassFilter2pFloat barofilter{75,1};
};
