#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include "AP_OpticalFlow_HereFlow.h"

#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <stdio.h> // debug

#include <com/hex/equipment/flow/Measurement.hpp>

extern const AP_HAL::HAL& hal;

#define debug_flow_uavcan(level_debug, can_driver, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(can_driver)) { hal.console->printf(fmt, ##args); }} while (0)



//UAVCAN Frontend Registry Binder
UC_REGISTRY_BINDER(MeasurementCb, com::hex::equipment::flow::Measurement);

//uint8_t AP_OpticalFlow_HereFlow::_node_id = 0;



// AP_OpticalFlow_HereFlow* AP_OpticalFlow_HereFlow::_driver = nullptr;
// AP_UAVCAN* AP_OpticalFlow_HereFlow::_ap_uavcan = nullptr;

/*
  constructor - registers instance at top Flow driver
 */
AP_OpticalFlow_HereFlow::AP_OpticalFlow_HereFlow(OpticalFlow &flow) :

    OpticalFlow_backend(flow)
{
    //this->_instance = instance;
    //this->_node_id = instance+UAVCAN_START_NODE_ID;
    //printf("Found HereFlow\n"); //debug
//     if (_driver) {
//         AP_HAL::panic("Only one instance of Flow supported!");
//     }
//     _driver = this;
//     _driver->_node_id = instance;
}

//Method to find the backend relating to the node id
AP_OpticalFlow_HereFlow* AP_OpticalFlow_HereFlow::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id)
{
    //printf("Node ID: %d\n", node_id);
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    AP_OpticalFlow_HereFlow* driver = nullptr;
    // for (uint8_t i = 0; i < OPTICALFLOW_MAX_INSTANCES; i++) {
    //     if (AP::OpticalFlow()._type == OpticalFlowType::UAVCAN) {
    driver = (AP_OpticalFlow_HereFlow*)AP::opticalflow()->backend[get_instance(node_id)];
    //printf("Are we Here?\n"); //debug

    //Double check if the driver was initialised as UAVCAN Type
    if (driver != nullptr) {
        if (driver->_node_id == node_id && 
            driver->_ap_uavcan == ap_uavcan) {
            return driver;
        } else {
            //we found a possible duplicate addressed sensor
            //we return nothing in such scenario
            return nullptr;
        }
    }

    AP::opticalflow()->backend[get_instance(node_id)] = new AP_OpticalFlow_HereFlow(*AP::opticalflow());
    //driver = (AP_OpticalFlow_HereFlow*)AP::opticalflow()->backend[get_instance(node_id)];
    // if (driver == nullptr) {
    //     return driver;
    // }
    //if (driver->_ap_uavcan == nullptr) {
    driver->_ap_uavcan = ap_uavcan;
    driver->_node_id = node_id;
    driver->_instance = get_instance(node_id);
    printf("Created HereFlow Instance %i \n",get_instance(node_id));
    //}

    return driver;
}



//links the HereFlow messages to the backend
void AP_OpticalFlow_HereFlow::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<com::hex::equipment::flow::Measurement, MeasurementCb> *measurement_listener;
    measurement_listener = new uavcan::Subscriber<com::hex::equipment::flow::Measurement, MeasurementCb>(*node);
    // Register method to handle incoming HereFlow measurement
    const int measurement_listener_res = measurement_listener->start(MeasurementCb(ap_uavcan, &handle_measurement));
    if (measurement_listener_res < 0) {
        AP_HAL::panic("UAVCAN Flow subscriber start problem\n\r");
        return;
    }
}

//updates driver states based on received HereFlow messages
void AP_OpticalFlow_HereFlow::handle_measurement(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MeasurementCb &cb)
{
    //("Are we Here1?\n"); //debug
    //fetch the matching uavcan driver, node id and sensor id backend instance
    AP_OpticalFlow_HereFlow* driver = get_uavcan_backend(ap_uavcan, node_id);
    if (driver == nullptr) {
        //printf("Are we Heres5?\n"); //debug
        return;
    }

    // if (_driver == nullptr) {
    //     return;
    // }
    //protect from data coming from duplicate sensors,
    //as we only handle one Here Flow at a time as of now
    // if (driver->_ap_uavcan == nullptr) {
    //     driver->_ap_uavcan = ap_uavcan;
    //     driver->_node_id = node_id;
    // }
    // if (driver->_ap_uavcan == ap_uavcan && driver->_node_id == node_id) {
    //     WITH_SEMAPHORE(driver->_sem);
    //     driver->new_data = true;
    //     driver->flowRate = Vector2f(cb.msg->flow_integral[0], cb.msg->flow_integral[1]);
    //     driver->bodyRate = Vector2f(cb.msg->rate_gyro_integral[0], cb.msg->rate_gyro_integral[1]);
    //     driver->integral_time = cb.msg->integration_interval;
    //     driver->surface_quality = cb.msg->quality;
    //     //printf("DRV: %u %f %f\n", cb.msg->quality, cb.msg->flow_integral[0], cb.msg->flow_integral[1]);
    // }
    WITH_SEMAPHORE(driver->_sem);
    driver->new_data = true;
    driver->flowRate = Vector2f(cb.msg->flow_integral[0], cb.msg->flow_integral[1]);
    driver->bodyRate = Vector2f(cb.msg->rate_gyro_integral[0], cb.msg->rate_gyro_integral[1]);
    driver->integral_time = cb.msg->integration_interval;
    driver->surface_quality = cb.msg->quality;
    //printf("DRV: %u %f %f %i\n", driver->surface_quality, driver->flowRate.length(), driver->bodyRate.length(),driver->_instance);
}

void AP_OpticalFlow_HereFlow::update()
{
    _push_state();
}

// Read the sensor
void AP_OpticalFlow_HereFlow::_push_state(void)
{

    //printf("Are we Heres3?\n"); //debug
    WITH_SEMAPHORE(_sem);
    if (!new_data) {
    //     //printf("Are we Heres5?\n"); //debug
        return;
    // }
    }
    struct OpticalFlow::OpticalFlow_state state;
    const Vector2f flowScaler = _flowScaler();
    //setup scaling based on parameters
    float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
    float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
    float integralToRate = 1.0f / integral_time;
    //Convert to Raw Flow measurement to Flow Rate measurement
    state.flowRate = Vector2f(flowRate.x * flowScaleFactorX,
                                flowRate.y * flowScaleFactorY) * integralToRate;
    state.bodyRate = bodyRate * integralToRate;
    state.surface_quality = surface_quality;
    _applyYaw(state.flowRate);
    _applyYaw(state.bodyRate);
    //printf("DRV: %u %f %f\n", state.surface_quality, flowRate.length(), bodyRate.length());
    //hal.console->printf("DRV: %u %f %f\n", state.surface_quality, flowRate.length(), bodyRate.length());
    _update_frontend2(state,_instance);
    //printf("DRV2: %u %f %f %i\n", state.surface_quality, flowRate.length(), bodyRate.length(),_instance);
    new_data = false;
}

#endif // HAL_WITH_UAVCAN

// uint8_t get_instance(uint8_t node_id)
// {
//     return node_id - HEREFLOW_INIT_NODE_ID;
// }