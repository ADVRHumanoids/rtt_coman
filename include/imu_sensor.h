#ifndef _IMU_SENSOR_H_
#define _IMU_SENSOR_H_

#include <force_torque_sensor.h>
#include <rst-rt/robot/IMU.hpp>
#include <imu_3DM-GX3-25.h>

typedef sensorFeedback<rstrt::robot::IMU> imu_data;

class imu_sensor{
public:
    imu_sensor(const std::string& frame, RTT::DataFlowInterface& ports);

    void sense();
    std::string getFrame(){ return _imu_frame;}
    bool isInited(){ return _is_inited;}
private:
    boost::shared_ptr<imu_data> _imu_data;
    /**
     * @brief _imu_frame is the frame where the force/torque is measured
     */
    std::string _imu_frame;
    RTT::DataFlowInterface& _ports;

    data_3DM_GX3_t raw_data;

    bool _is_inited;

#if __XENO__
    boost::shared_ptr<rt_imu_3DM_GX3_25>   _imu;
#else
    boost::shared_ptr<nrt_imu_3DM_GX3_25>  _imu;
#endif

    void setFeedback();

    void fillMsg()
    {
        float acc[3];
        float angRate[3];
        float quat[4];

        _imu->get_Quaternion(quat, 0);
        _imu->get_Acc_Ang(acc, angRate, 0);

        _imu_data->sensor_feedback.rotation(0) = quat[0];
        _imu_data->sensor_feedback.rotation(1) = quat[1];
        _imu_data->sensor_feedback.rotation(2) = quat[2];
        _imu_data->sensor_feedback.rotation(3) = quat[3];

        _imu_data->sensor_feedback.linearAcceleration(0) = acc[0];
        _imu_data->sensor_feedback.linearAcceleration(1) = acc[1];
        _imu_data->sensor_feedback.linearAcceleration(2) = acc[2];

        _imu_data->sensor_feedback.angularVelocity(0) = angRate[0];
        _imu_data->sensor_feedback.angularVelocity(1) = angRate[1];
        _imu_data->sensor_feedback.angularVelocity(2) = angRate[2];
    }


};


#endif
