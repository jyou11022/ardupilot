#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_OpticalFlow_HereFlow.h"
#include "OpticalFlow.h"

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

//#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
//#include <AP_UAVCAN/AP_UAVCAN.h>

//#include <com/hex/equipment/flow/Measurement.hpp>

extern const AP_HAL::HAL& hal;

#define debug_flow_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)

uint8_t AP_OpticalFlow_HereFlow::_node_id = 40;
//#define debug_flow_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { hal.console->printf(fmt, ##args); }} while (0)

//UAVCAN Frontend Registry Binder
//UC_REGISTRY_BINDER(MeasurementCb, com::hex::equipment::flow::Measurement);

//uint8_t AP_OpticalFlow_HereFlow::_node_id = 0;
//AP_OpticalFlow_HereFlow* AP_OpticalFlow_HereFlow::_driver = nullptr;
//AP_UAVCAN* AP_OpticalFlow_HereFlow::_ap_uavcan = nullptr;
/*
  constructor - registers instance at top Flow driver
 */
AP_OpticalFlow_HereFlow::AP_OpticalFlow_HereFlow(OpticalFlow &flow) :
    OpticalFlow_backend(flow)
{
    _sem_flow = hal.util->new_semaphore();

    // if (_driver) {
    //     AP_HAL::panic("Only one instance of Flow supported!");
    // }
}

AP_OpticalFlow_HereFlow::~AP_OpticalFlow_HereFlow()
{
    if (!_initialized) {
        return;
    }
    
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(_manager);
    if (ap_uavcan == nullptr) {
        return;
    }
    
    ap_uavcan->remove_flow_listener(this);
    delete _sem_flow;
    
    debug_flow_uavcan(2, "AP_OpticalFlow_HereFlow destructed\n\r");
}

OpticalFlow_backend *AP_OpticalFlow_HereFlow::probe(OpticalFlow &flow)
{
    if (AP_BoardConfig_CAN::get_can_num_ifaces() == 0) {
        return nullptr;
    }

    AP_OpticalFlow_HereFlow *sensor;

    for (uint8_t i = 0; i < MAX_NUMBER_OF_CAN_DRIVERS; i++) {
        AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(i);
        if (ap_uavcan == nullptr) {
            continue;
        }
        uint8_t freeflow = ap_uavcan->find_smallest_free_flow_node();
        if (freeflow == UINT8_MAX) {
            continue;
        }
        sensor = new AP_OpticalFlow_HereFlow(flow);
        
        if (sensor->register_uavcan_flow(i, freeflow)) {
            debug_flow_uavcan(2, "AP_OpticalFlow_HereFlow probed, drv: %d, node: %d\n\r", i, freeflow);
            return sensor;
        } else {
            delete sensor;
        }
    }

    return nullptr;
}


//links the HereFlow messages to the backend
// void AP_OpticalFlow_HereFlow::subscribe_msgs(AP_UAVCAN* ap_uavcan)
// {
//     if (ap_uavcan == nullptr) {
//         return;
//     }

//     auto* node = ap_uavcan->get_node();

//     uavcan::Subscriber<com::hex::equipment::flow::Measurement, MeasurementCb> *measurement_listener;
//     measurement_listener = new uavcan::Subscriber<com::hex::equipment::flow::Measurement, MeasurementCb>(*node);
//     // Register method to handle incoming HereFlow measurement
//     const int measurement_listener_res = measurement_listener->start(MeasurementCb(ap_uavcan, &handle_measurement));
//     if (measurement_listener_res < 0) {
//         AP_HAL::panic("UAVCAN Flow subscriber start problem\n\r");
//         return;
//     }
// }

//updates driver states based on received HereFlow messages
// void AP_OpticalFlow_HereFlow::handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb)
// {
//     if (_driver == nullptr) {
//         return;
//     }
//     //protect from data coming from duplicate sensors,
//     //as we only handle one Here Flow at a time as of now
//     if (_ap_uavcan == nullptr) {
//         _ap_uavcan = ap_uavcan;
//         _node_id = node_id;
//     }

//     if (_ap_uavcan == ap_uavcan && _node_id == node_id) {
//         WITH_SEMAPHORE(_driver->_sem);
//         _driver->new_data = true;
//         _driver->flowRate = Vector2f(cb.msg->flow_integral[0], cb.msg->flow_integral[1]);
//         _driver->bodyRate = Vector2f(cb.msg->rate_gyro_integral[0], cb.msg->rate_gyro_integral[1]);
//         _driver->integral_time = cb.msg->integration_interval;
//         _driver->surface_quality = cb.msg->quality;
//     }
// }


void AP_OpticalFlow_HereFlow::handle_flow_msg(Vector2f flowRate, Vector2f bodyRate, uint8_t  surface_quality, float integral_time)
{
    if (_sem_flow->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _flowRate = flowRate;
        _bodyRate = bodyRate;
        _surface_quality = surface_quality;
        _integral_time = integral_time;
        new_data = true;
        //_last_timestamp = AP_HAL::micros64();
        _sem_flow->give();
    }
}

bool AP_OpticalFlow_HereFlow::register_uavcan_flow(uint8_t mgr, uint8_t node)
{
    AP_UAVCAN *ap_uavcan = AP_UAVCAN::get_uavcan(mgr);
    if (ap_uavcan == nullptr) {
        return false;
    }
    _manager = mgr;

    if (ap_uavcan->register_flow_listener_to_node(this, node)) {
        //_instance = _frontend.register_sensor();
        debug_flow_uavcan(2, "AP_OpticalFlow_HereFlow loaded\n\r");

        _initialized = true;

        return true;
    }

    return false;
}


void AP_OpticalFlow_HereFlow::update()
{
    _push_state();
}

// Read the sensor
void AP_OpticalFlow_HereFlow::_push_state(void)
{
    if (_sem_flow->take(HAL_SEMAPHORE_BLOCK_FOREVER) && !new_data) {
        struct OpticalFlow::OpticalFlow_state state {};
        const Vector2f flowScaler = _flowScaler();
        //setup scaling based on parameters
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
        float integralToRate = 1.0f / _integral_time;
        //Convert to Raw Flow measurement to Flow Rate measurement
        state.device_id = _node_id;
        state.flowRate = Vector2f(_flowRate.x * flowScaleFactorX,
                                    _flowRate.y * flowScaleFactorY) * integralToRate;
        state.bodyRate = _bodyRate * integralToRate;
        state.surface_quality = _surface_quality;
        _applyYaw(state.flowRate);
        _applyYaw(state.bodyRate);
        // hal.console->printf("DRV: %u %f %f\n", state.surface_quality, flowRate.length(), bodyRate.length());
        _update_frontend(state);
        new_data = false;
        _sem_flow->give();
    }
}

#endif // HAL_WITH_UAVCAN
