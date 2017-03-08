#include <imu_sensor.h>

imu_sensor::imu_sensor(RTT::DataFlowInterface &ports):
    _ports(ports)
{
#if __XENO__
        _imu.reset(new rt_imu_3DM_GX3_25("rtser0", 115200, CMD_QUATERNION));
#else
        _imu.reset(new nrt_imu_3DM_GX3_25("/dev/ttyACM0", 115200, CMD_QUATERNION));
#endif
}
