#include <imu_sensor.h>

imu_sensor::imu_sensor(const std::string &frame, RTT::DataFlowInterface &ports):
    _ports(ports),
    _imu_frame(frame),
    _is_inited(false)
{
    RTT::log(RTT::Info)<<"Creating IMU sensor on link... "<<frame<<RTT::endlog();
#if __XENO__
        _imu.reset(new rt_imu_3DM_GX3_25("rtser0", 115200, CMD_QUATERNION));
#else
        _imu.reset(new nrt_imu_3DM_GX3_25("/dev/ttyACM0", 115200, CMD_QUATERNION));
#endif

        setFeedback();
    RTT::log(RTT::Info)<<"...IMU created"<<frame<<RTT::endlog();
        _is_inited = true;
}

void imu_sensor::setFeedback()
{
    _imu_data.reset(new imu_data);
    _imu_data->orocos_port.setName(_imu_frame+"_SensorFeedback");
    _imu_data->orocos_port.doc("Linear acceleration, Angular rate and Orientation measured from IMU sensor.");
    _ports.addPort(_imu_data->orocos_port);

    _imu_data->sensor_feedback = rstrt::robot::IMU();
    fillMsg();
    _imu_data->orocos_port.setDataSample(_imu_data->sensor_feedback);
}

void imu_sensor::sense()
{
    if(_imu_data)
    {

        _imu->get_raw_data_sync(raw_data);
        fillMsg();

        if (_imu_data->orocos_port.connected())
            _imu_data->orocos_port.write(_imu_data->sensor_feedback);
    }
}
