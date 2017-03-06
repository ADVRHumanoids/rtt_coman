#ifndef _FORCE_TORQUE_SENSOR_H_
#define _FORCE_TORQUE_SENSOR_H_

#include <XBotCoreModel.h>
#include <rtt/Port.hpp>
#include <rst-rt/dynamics/Wrench.hpp>
#include <Boards_ctrl_ext.h>

class force_torqueID
{
public:
    /**
     * @brief force_torqueID map ft ID with boards ID
     * @param i 0 = COMAN_NO_FOREARMS, 1 = COMAN_FOREARMS
     */
    force_torqueID(const std::string& force_torque_name):
        ft_name(force_torque_name)
    {
        l_arm = {37};
        r_arm = {36};

        r_leg = {24};
        l_leg = {25};

        if(ft_name.compare("l_arm_ft") == 0)
            boards_id = l_arm;
        if(ft_name.compare("l_leg_ft") == 0)
            boards_id = l_leg;
        if(ft_name.compare("r_leg_ft") == 0)
            boards_id = r_leg;
        if(ft_name.compare("r_arm_ft") == 0)
            boards_id = r_arm;
    }

    std::vector<int> boards_id;

private:
      std::string ft_name;
      std::vector<int> r_leg;
      std::vector<int> l_leg;
      std::vector<int> r_arm;
      std::vector<int> l_arm;

};


template <class T> class sensorFeedback {
public:
    sensorFeedback() {
    }

    ~sensorFeedback() {

    }

    T sensor_feedback;
    RTT::OutputPort<T> orocos_port;

};

typedef sensorFeedback<rstrt::dynamics::Wrench> wrench;

class force_torque_sensor{
public:
    force_torque_sensor(const std::string& joint_srdf, RTT::DataFlowInterface& ports,
                        boost::shared_ptr<urdf::ModelInterface const> urdf_model,
                        ts_bc_data_t* boards_data);

    bool isInited(){ return _inited;}
    void sense();
    std::string getFrame(){ return _force_torque_frame;}

    bool setMeasurementDirection(const std::vector<int>& directions);
private:
    /**
     * @brief _force_torque_frame is the frame where the force/torque is measured
     */
    std::string _force_torque_frame;

    boost::shared_ptr<wrench> _wrench_measured;
    boost::shared_ptr<force_torqueID> _ft_ID;
    RTT::DataFlowInterface& _ports;
    bool _inited;
    ts_bc_data_t* _boards_data;
    std::vector<int> _measurement_direction;

    /**
     * @brief pairFrameToSensor check if a sensor is associated to a given frame
     * @param joint_srdf is the joint name specified in the srdf under the group "force_torque_sensors"
     * @param sensor
     * @return true if the sensor is in the specified frame
     */
    bool pairFrameToSensor(const std::string& joint_srdf,
                           boost::shared_ptr<urdf::ModelInterface const> urdf_model);

    void setFeedback();

    void fillMsg()
    {
        _wrench_measured->sensor_feedback.forces[0] = _measurement_direction[0]*
                (_boards_data[_ft_ID->boards_id[0]-1].raw_bc_data.ft_bc_data.fx)/1000.;
        _wrench_measured->sensor_feedback.forces[1] = _measurement_direction[1]*
                (_boards_data[_ft_ID->boards_id[0]-1].raw_bc_data.ft_bc_data.fy)/1000.;
        _wrench_measured->sensor_feedback.forces[2] = _measurement_direction[2]*
                (_boards_data[_ft_ID->boards_id[0]-1].raw_bc_data.ft_bc_data.fz)/1000.;
        _wrench_measured->sensor_feedback.torques[0] = _measurement_direction[3]*
                (_boards_data[_ft_ID->boards_id[0]-1].raw_bc_data.ft_bc_data.tx)/1000.;
        _wrench_measured->sensor_feedback.torques[1] = _measurement_direction[4]*
                (_boards_data[_ft_ID->boards_id[0]-1].raw_bc_data.ft_bc_data.ty)/1000.;
        _wrench_measured->sensor_feedback.torques[2] = _measurement_direction[5]*
                (_boards_data[_ft_ID->boards_id[0]-1].raw_bc_data.ft_bc_data.tz)/1000.;

    }

};

#endif
