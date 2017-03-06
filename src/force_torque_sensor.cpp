#include <force_torque_sensor.h>

force_torque_sensor::force_torque_sensor(const std::string &joint_srdf,
                                         RTT::DataFlowInterface &ports,
                                         boost::shared_ptr<const urdf::ModelInterface> urdf_model,
                                         ts_bc_data_t* boards_data):
    _inited(false),
    _force_torque_frame(""),
    _ports(ports),
    _boards_data(boards_data)
{
    RTT::log(RTT::Info) << "Creating Force/Torque Sensor "<<RTT::endlog();

    if(pairFrameToSensor(joint_srdf, urdf_model))
    {
        _ft_ID.reset(new force_torqueID(_force_torque_frame));
        RTT::log(RTT::Info)<<"Board ID: "<<_ft_ID->boards_id[0]<<RTT::endlog();

        setFeedback();
        _inited = true;
    }
    else
        RTT::log(RTT::Error)<<joint_srdf<<" can not be associated to any FT sensor"<<RTT::endlog();


}

bool force_torque_sensor::pairFrameToSensor(const std::string& joint_srdf,
                                            boost::shared_ptr<urdf::ModelInterface const> urdf_model)
{
    boost::shared_ptr<const urdf::Joint> ft_joint = urdf_model->getJoint(joint_srdf);
    if(ft_joint == NULL)
        return false;

    _force_torque_frame = ft_joint->child_link_name;
    RTT::log(RTT::Info)<<"Sensor of type force_torque specified in joint "<<joint_srdf<<" is associated to link "<<
                         _force_torque_frame<<RTT::endlog();
    return true;
}

void force_torque_sensor::setFeedback()
{
    _wrench_measured.reset(new wrench);
    _wrench_measured->orocos_port.setName(_force_torque_frame+"_SensorFeedback");
    _wrench_measured->orocos_port.doc("Wrench measured from force/torque sensor.");
    _ports.addPort(_wrench_measured->orocos_port);

    _wrench_measured->sensor_feedback = rstrt::dynamics::Wrench();
    fillMsg();
    _wrench_measured->orocos_port.setDataSample(_wrench_measured->sensor_feedback);
}

void force_torque_sensor::sense()
{
    if(_wrench_measured)
    {

        fillMsg();

        if (_wrench_measured->orocos_port.connected())
            _wrench_measured->orocos_port.write(_wrench_measured->sensor_feedback);
    }
}
