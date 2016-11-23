#include <test/motor_controller_test.h>

motor_controller_test::motor_controller_test(const std::string &name):
    RTT::TaskContext(name),
    _urdf_path(""),
    _srdf_path(""),
    _dt_ms(0.005),
    _robot_name("")
{
    this->setActivity(new RTT::Activity(1, _dt_ms));

    this->addOperation("loadURDFAndSRDF", &motor_controller_test::loadURDFAndSRDF,
                this, RTT::ClientThread);
    this->addOperation("attachToRobot", &motor_controller_test::attachToRobot,
                this, RTT::ClientThread);

    this->addOperation("startTrj", &motor_controller_test::startTrj,
                this, RTT::ClientThread);
    this->addOperation("stopTrj", &motor_controller_test::stopTrj,
                this, RTT::ClientThread);

    _urdf_model.reset(new urdf::Model());
}

bool motor_controller_test::loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path)
{
    _urdf_path = URDF_path;
    _srdf_path = SRDF_path;

    RTT::log(RTT::Info)<<"URDF path: "<<_urdf_path<<RTT::endlog();
    RTT::log(RTT::Info)<<"SRDF path: "<<_srdf_path<<RTT::endlog();

    bool models_loaded = _urdf_model->initFile(_urdf_path);

    if(models_loaded)
    {
        RTT::log(RTT::Info)<<"Model name: "<<_urdf_model->getName()<<RTT::endlog();

        std::map<std::string, boost::shared_ptr<urdf::Joint> > joint_map = _urdf_model->joints_;
        std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it;
        RTT::log(RTT::Info)<<"Joints: "<<RTT::endlog();
        for(it = joint_map.begin(); it != joint_map.end(); it++)
        {
            if(it->second->type != urdf::Joint::FIXED){
                _joint_list.push_back(it->first);
                RTT::log(RTT::Info)<<"  "<<_joint_list.back()<<RTT::endlog();}
        }
        RTT::log(RTT::Info)<<"Total number of joints is "<<_joint_list.size()<<RTT::endlog();
    }


    getJointLimits(_map_joint_limimts);

    return models_loaded;
}

bool motor_controller_test::attachToRobot(const std::string &robot_name)
{
    _robot_name = robot_name;
    RTT::log(RTT::Info)<<"Robot name: "<<_robot_name<<RTT::endlog();

    RTT::TaskContext* task_ptr = this->getPeer(robot_name);
    if(!task_ptr){
        RTT::log(RTT::Error)<<"Can not getPeer("<<robot_name<<")"<<RTT::endlog();
        return false;}

    RTT::log(RTT::Info)<<"Found Peer "<<robot_name<<RTT::endlog();

    RTT::OperationCaller<std::map<std::string, std::vector<std::string> >(void) > getKinematicChainsAndJoints
        = task_ptr->getOperation("getKinematicChainsAndJoints");

    _map_kin_chains_joints = getKinematicChainsAndJoints();

    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string kin_chain_name = it->first;
        std::vector<std::string> joint_names = it->second;

        _kinematic_chains_feedback_ports[kin_chain_name] =
            boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> >(
                        new RTT::InputPort<rstrt::robot::JointState>(
                            kin_chain_name+"_"+"JointFeedback"));
        this->addPort(*(_kinematic_chains_feedback_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointFeedback port");
        _kinematic_chains_feedback_ports.at(kin_chain_name)->connectTo(
                    task_ptr->ports()->getPort(kin_chain_name+"_"+"JointFeedback"));

        rstrt::robot::JointState tmp(joint_names.size());
        _kinematic_chains_joint_state_map[kin_chain_name] = tmp;
        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data feedback"<<RTT::endlog();



        _kinematic_chains_output_ports[kin_chain_name] =
                boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> >(
                            new RTT::OutputPort<rstrt::kinematics::JointAngles>(
                                kin_chain_name+"_"+"JointPositionCtrl"));
        this->addPort(*(_kinematic_chains_output_ports.at(kin_chain_name))).
                doc(kin_chain_name+"_"+"JointPositionCtrl port");
        _kinematic_chains_output_ports.at(kin_chain_name)->connectTo(
                    task_ptr->ports()->getPort(kin_chain_name+"_"+"JointPositionCtrl"));

        rstrt::kinematics::JointAngles tmp2(joint_names.size());
        _kinematic_chains_desired_joint_state_map[kin_chain_name] = tmp2;
        RTT::log(RTT::Info)<<"Added "<<kin_chain_name<<" port and data input"<<RTT::endlog();

        _map_chain_trj_time[kin_chain_name] = 0.0;
        _map_chain_start_trj[kin_chain_name] = false;
        _map_chain_q0[kin_chain_name] = rstrt::robot::JointState(joint_names.size());
    }

    return true;
}

bool motor_controller_test::configureHook()
{

}

bool motor_controller_test::startHook()
{

}

void motor_controller_test::updateHook()
{
    std::map<std::string, bool>::iterator it;
    for(it = _map_chain_start_trj.begin(); it != _map_chain_start_trj.end(); it++)
    {
        if(it->second)
        {
            _map_chain_trj_time.at(it->first) += _dt_ms;
            double amplitude = 10.*M_PI/180.;
            double period = 0.1;

            rstrt::kinematics::JointAngles previous = _kinematic_chains_desired_joint_state_map.at(it->first);

            _kinematic_chains_desired_joint_state_map.at(it->first) = sin_traj(_map_chain_q0.at(it->first), amplitude, _map_chain_trj_time.at(it->first), period);


            for(unsigned int i = 0; i < previous.angles.rows(); ++i)
            {
                std::pair<double, double> min_max = _map_joint_limimts.at(_map_kin_chains_joints.at(it->first)[i]);

                if(_kinematic_chains_desired_joint_state_map.at(it->first).angles[i] <= min_max.first ||
                   _kinematic_chains_desired_joint_state_map.at(it->first).angles[i] >= min_max.second){
                    _kinematic_chains_desired_joint_state_map.at(it->first).angles[i] =
                            previous.angles[i];

                    RTT::log(RTT::Warning)<<"Joint "<<_map_kin_chains_joints.at(it->first)[i]<<" is not moving, it reaches joint limits"<<RTT::endlog();
                }
            }


            _kinematic_chains_output_ports.at(it->first)->write(_kinematic_chains_desired_joint_state_map.at(it->first));
        }
    }
}

void motor_controller_test::stopHook()
{

}

void motor_controller_test::getJointLimits(std::map<std::string, std::pair<double, double>>& map)
{
    std::map<std::string, std::vector<std::string> >::iterator it;
    for(it = _map_kin_chains_joints.begin(); it != _map_kin_chains_joints.end(); it++)
    {
        std::string chain_name = it->first;
        std::vector<std::string> joint_names = it->second;
        RTT::log(RTT::Info)<<"Joint Limits for chain "<<chain_name<<RTT::endlog();
        for(unsigned int i = 0; i < joint_names.size(); ++i)
        {
            boost::shared_ptr<const urdf::Joint> joint = _urdf_model->getJoint(joint_names[i]);
            double lower = joint->limits->lower;
            double upper = joint->limits->upper;

            map[joint_names[i]] = std::pair<double,double>(lower, upper);

            RTT::log(RTT::Info)<<"  "<<joint_names[i]<<" ["<<lower<<", "<<upper<<"]"<<RTT::endlog();
        }
    }
}

bool motor_controller_test::startTrj(const std::string &chain_name)
{
    if ( _map_kin_chains_joints.find(chain_name) == _map_kin_chains_joints.end() )
        return false;

    RTT::FlowStatus fs = _kinematic_chains_feedback_ports.at(chain_name)->read(
        _kinematic_chains_joint_state_map.at(chain_name));

    for(unsigned int i = 0; i < _kinematic_chains_joint_state_map.at(chain_name).angles.rows(); ++i)
        if(std::isnan(_kinematic_chains_joint_state_map.at(chain_name).angles[i]))
            return false;

    _map_chain_q0[chain_name] = _kinematic_chains_joint_state_map.at(chain_name);
    _map_chain_trj_time.at(chain_name) = 0.0;
    _map_chain_start_trj.at(chain_name) = true;


    return true;
}

bool motor_controller_test::stopTrj(const std::string &chain_name)
{
    if ( _map_kin_chains_joints.find(chain_name) == _map_kin_chains_joints.end() )
        return false;

    _map_chain_start_trj.at(chain_name) = false;
}

ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(motor_controller_test)
