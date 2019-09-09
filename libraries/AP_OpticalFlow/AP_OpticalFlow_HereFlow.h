#pragma once
#include "OpticalFlow_backend.h"

#include "OpticalFlow.h"

#if HAL_WITH_UAVCAN

#include <AP_UAVCAN/AP_UAVCAN.h>

class MeasurementCb;

class AP_OpticalFlow_HereFlow : public OpticalFlow_backend {
public:
    AP_OpticalFlow_HereFlow(OpticalFlow &flow);
    ~AP_OpticalFlow_HereFlow() override;

    static OpticalFlow_backend *probe(OpticalFlow &flow);

    void init() override {}

    void update() override;

//    static void subscribe_msgs(AP_UAVCAN* ap_uavcan);

    //HereFlow: callback for UAVCAN messages
    virtual void handle_flow_msg(Vector2f flowRate, Vector2f bodyRate, uint8_t  surface_quality, float integral_time) override;

    bool register_uavcan_flow(uint8_t mgr, uint8_t node);

//    static void handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb);

private:

    Vector2f _flowRate, _bodyRate;
    uint8_t _surface_quality;
    float _integral_time;
    bool new_data;
    static uint8_t _node_id;

    uint8_t _manager;

//    static AP_OpticalFlow_HereFlow* _driver;
//    static AP_UAVCAN* _ap_uavcan;
    void _push_state(void);

    bool _initialized;

    AP_HAL::Semaphore *_sem_flow;

};
#endif //HAL_WITH_UAVCAN
