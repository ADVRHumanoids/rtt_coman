#ifndef _IMU_SENSOR_H_
#define _IMU_SENSOR_H_

#include <force_torque_sensor.h>
#include <rst-rt/robot/IMU.hpp>
#include <imu_3DM-GX3-25.h>

typedef sensorFeedback<rstrt::robot::IMU> imu_data;

class imu_sensor{
public:
    imu_sensor(RTT::DataFlowInterface& ports);

    void sense();
    std::string getFrame(){ return _imu_frame;}
private:
    imu_data _imu_data;
    /**
     * @brief _imu_frame is the frame where the force/torque is measured
     */
    std::string _imu_frame;
    RTT::DataFlowInterface& _ports;

#if __XENO__
    boost::shared_ptr<rt_imu_3DM_GX3_25>   _imu;
#else
    boost::shared_ptr<nrt_imu_3DM_GX3_25>  _imu;
#endif


};


#endif
