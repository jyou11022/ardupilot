#pragma once
#include "OpticalFlow_backend.h"

#define UAVCAN_START_NODE_ID 40

#if HAL_WITH_UAVCAN

#include <AP_UAVCAN/AP_UAVCAN.h>


class MeasurementCb;

class AP_OpticalFlow_HereFlow : public OpticalFlow_backend {
public:
    AP_OpticalFlow_HereFlow(OpticalFlow &flow, uint8_t instance);

    void init() override {}

    void update() override;

    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    static void handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb);

    static AP_OpticalFlow_HereFlow* get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id);

private:

    Vector2f flowRate, bodyRate;
    uint8_t surface_quality;
    float integral_time;
    bool new_data;

    uint8_t _node_id;
    uint8_t _instance;

    // AP_OpticalFlow_HereFlow* _driver;
    AP_UAVCAN* _ap_uavcan;
    void _push_state(void);

};
#endif //HAL_WITH_UAVCAN
