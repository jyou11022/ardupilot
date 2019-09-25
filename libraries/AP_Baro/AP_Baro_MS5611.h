#pragma once

#include "AP_Baro_Backend.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_MS5611_I2C_ADDR
#define HAL_BARO_MS5611_I2C_ADDR 0x77
#endif

#ifndef HAL_BARO_MS5837_I2C_ADDR
#define HAL_BARO_MS5837_I2C_ADDR 0x76
#endif

// #define MS5837_02BA_OFFSET -11000
// #define MS5837_02BA_C3 22750
// #define MS5837_02BA_C1 42870

class AP_Baro_MS56XX : public AP_Baro_Backend
{
public:
    void update();

    enum MS56XX_TYPE {
        BARO_MS5611 = 0,
        BARO_MS5607 = 1,
        BARO_MS5637 = 2,
        BARO_MS5837_30 = 3,
        BARO_MS5837_02 = 4
    };

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum MS56XX_TYPE ms56xx_type = BARO_MS5611);

    static uint16_t crc4(uint16_t *data);
    
private:
    /*
     * Update @accum and @count with the new sample in @val, taking into
     * account a maximum number of samples given by @max_count; in case
     * maximum number is reached, @accum and @count are updated appropriately
     */
    static void _update_and_wrap_accumulator(uint32_t *accum, uint32_t val,
                                             uint8_t *count, uint8_t max_count);

    AP_Baro_MS56XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum MS56XX_TYPE ms56xx_type);

    uint8_t _addr_cmd_convert_temperature;
    uint8_t _addr_cmd_convert_pressure;

    bool _init();

    void _calculate_5611();
    void _calculate_5607();
    void _calculate_5637();
    void _calculate_5837_30();
    void _calculate_5837_02();
    bool _read_prom_5611(uint16_t prom[8]);
    bool _read_prom_5637(uint16_t prom[8]);

    uint16_t _read_prom_word(uint8_t word);
    uint32_t _read_adc();

    void _timer();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    /* Shared values between thread sampling the HW and main thread */
    struct {
        uint32_t s_D1;
        uint32_t s_D2;
        uint8_t d1_count;
        uint8_t d2_count;
    } _accum;

    uint8_t _state;
    uint8_t _instance;

    /* Last compensated values from accumulated sample */
    float _D1, _D2;

    // Internal calibration registers
    struct {
        uint16_t c1, c2, c3, c4, c5, c6;
    } _cal_reg;

    bool _discard_next;

    enum MS56XX_TYPE _ms56xx_type;
};
