#include <kinematic_chain.h>

using namespace cogimon;

KinematicChain::KinematicChain(const std::string& chain_name, const std::vector<std::string> &joint_names,
                               RTT::DataFlowInterface& ports):
    _kinematic_chain_name(chain_name),
    _ports(ports),
    _current_control_mode(std::string(ControlModes::JointPositionCtrl)),
    _joint_names(joint_names)
{
    RTT::log(RTT::Info) << "Creating Kinematic Chain " << chain_name << RTT::endlog();

    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        _map_joint_name_scoped_name.insert(std::pair<std::string, std::string>(_joint_names[i], " "));

    RTT::log(RTT::Info) << "Joints: " << RTT::endlog();
    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        RTT::log(RTT::Info) << "    " << _joint_names[i] << RTT::endlog();
}

std::vector<RTT::base::PortInterface*> KinematicChain::getAssociatedPorts() {
    return _inner_ports;
}

bool KinematicChain::initKinematicChain(const cogimon::gains& gains_)
{
    _gains.reset(new cogimon::gains(gains_));
    std::vector<std::string> controllers = _gains->map_controllers[_kinematic_chain_name];
    if(controllers.empty())
    {
        RTT::log(RTT::Error)<<"No controllers are available!"<<RTT::endlog();
        return false;
    }
    else
    {
        RTT::log(RTT::Info)<<"Available controllers for "<<_kinematic_chain_name<<" are:"<<RTT::endlog();
        for(unsigned int i = 0; i < controllers.size(); ++i)
            RTT::log(RTT::Info)<<"  "<<controllers[i]<<RTT::endlog();
    }

    setJointNamesAndIndices();
    _number_of_dofs = _joint_names.size();

    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointPositionCtrl)) != controllers.end()){
        if(!setController(std::string(ControlModes::JointPositionCtrl)))
            return false;
        else
            RTT::log(RTT::Info)<<std::string(ControlModes::JointPositionCtrl)<<" activated!"<<RTT::endlog();
    }
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointImpedanceCtrl)) != controllers.end()){
        if(!setController(std::string(ControlModes::JointImpedanceCtrl)))
            return false;
        else
            RTT::log(RTT::Info)<<std::string(ControlModes::JointImpedanceCtrl)<<" activated!"<<RTT::endlog();
    }
    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointTorqueCtrl)) != controllers.end()){
        if(!setController(std::string(ControlModes::JointTorqueCtrl)))
            return false;
        else
            RTT::log(RTT::Info)<<std::string(ControlModes::JointTorqueCtrl)<<" activated!"<<RTT::endlog();}


    setFeedBack(); //We consider, for now, that the full feedback is available
    RTT::log(RTT::Info)<<"Full feedback activated!"<<RTT::endlog();

    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointPositionCtrl)) != controllers.end()){
        setInitialPosition();
    }

    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointImpedanceCtrl)) != controllers.end()){
        setInitialImpedance();
        RTT::log(RTT::Info)<<"Initial Impedance set!"<<RTT::endlog();
    }

    return true;
}

std::vector<std::string> KinematicChain::getJointNames()
{
    return _joint_names;
}

std::vector<std::string> KinematicChain::getControllersAvailable()
{
    return _controllers_name;
}

std::string KinematicChain::getKinematicChainName()
{
    return _kinematic_chain_name;
}

unsigned int KinematicChain::getNumberOfDOFs()
{
    return _number_of_dofs;
}

std::string KinematicChain::getCurrentControlMode()
{
    return _current_control_mode;
}

void KinematicChain::setFeedBack()
{
        full_feedback.reset(new full_fbk);
        full_feedback->orocos_port.setName(_kinematic_chain_name+"_JointFeedback");
        full_feedback->orocos_port.doc("Output for Joint-fb from Gazebo to Orocos world. Contains joint-position, -velocity and -torque.");
        _ports.addPort(full_feedback->orocos_port);
        // TODO needs to be solved better
        _inner_ports.push_back(_ports.getPort(full_feedback->orocos_port.getName()));

        full_feedback->joint_feedback = JointState(_number_of_dofs);
        full_feedback->orocos_port.setDataSample(full_feedback->joint_feedback);
}

bool KinematicChain::setController(const std::string& controller_type)
{
    if(controller_type == ControlModes::JointPositionCtrl){
        position_controller.reset(new position_ctrl);
        position_controller->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointPositionCtrl);
        position_controller->orocos_port.doc("Input for JointPosition-cmds from Orocos to Robolli.");
        _ports.addPort(position_controller->orocos_port);
        // TODO needs to be solved better
        _inner_ports.push_back(_ports.getPort(position_controller->orocos_port.getName()));

        position_controller->joint_cmd = JointAngles(_number_of_dofs);
        position_controller->joint_cmd.angles.setZero();

        //ROBOLLI????
    }
    else if(controller_type == ControlModes::JointImpedanceCtrl){
        impedance_controller.reset(new impedance_ctrl);
        impedance_controller->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointImpedanceCtrl);
        impedance_controller->orocos_port.doc("Input for JointImpedance-cmds from Orocos to Robolli.");
        _ports.addPort(impedance_controller->orocos_port);
        // TODO needs to be solved better
        _inner_ports.push_back(_ports.getPort(impedance_controller->orocos_port.getName()));

        impedance_controller->joint_cmd = JointImpedance(_number_of_dofs);
        impedance_controller->joint_cmd.stiffness.setZero();
        impedance_controller->joint_cmd.damping.setZero();
    }
    else if(controller_type == ControlModes::JointTorqueCtrl)
    {
        torque_controller.reset(new torque_ctrl);
        torque_controller->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointTorqueCtrl);
        torque_controller->orocos_port.doc("Input for JointTorque-cmds from Orocos to Robolli.");
        _ports.addPort(torque_controller->orocos_port);
        // TODO needs to be solved better
        _inner_ports.push_back(_ports.getPort(torque_controller->orocos_port.getName()));

        torque_controller->joint_cmd = JointTorques(_number_of_dofs);
        torque_controller->joint_cmd.torques.setZero();
    }
    else{
        RTT::log(RTT::Error) << "Control Mode: " << controller_type << " is not available!" << RTT::endlog();
        return false;}
    _controllers_name.push_back(controller_type);
    return true;
}

//This method has to be implemented to sync kinematic chains joints
// to Robolli joints
bool KinematicChain::setJointNamesAndIndices()
{

    return true;
}

void KinematicChain::setInitialPosition()
{
    position_controller->orocos_port.clear();
    //Here I have to read from robolli actual position of the joints
    //for(unsigned int i = 0; i < _joint_names.size(); ++i)
        //position_controller->joint_cmd.angles[i] = _model->GetJoint(_joint_names[i])->GetAngle(0).Radian();

    position_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

//Here we use the vaue given by robolli
void KinematicChain::setInitialImpedance()
{
    RTT::log(RTT::Info)<<_kinematic_chain_name<<" impedance:"<<RTT::endlog();
    for(unsigned int i = 0; i < _joint_names.size(); ++i){
//        impedance_controller->joint_cmd.stiffness[i] = impedance.stiffness;
//        impedance_controller->joint_cmd.damping[i] = impedance.damping;
//        RTT::log(RTT::Info)<<"  "<<impedance.joint_name<<" stiffness: "<<impedance.stiffness<<" damping: "<<impedance.damping<<RTT::endlog();
    }
    impedance_controller->joint_cmd_fs = RTT::FlowStatus::NewData;
}

bool KinematicChain::setControlMode(const std::string &controlMode)
{
    if(controlMode != ControlModes::JointPositionCtrl &&
       controlMode != ControlModes::JointTorqueCtrl   &&
       controlMode != ControlModes::JointImpedanceCtrl ){
        RTT::log(RTT::Warning) << "Control Mode " << controlMode << " does not exist!" << RTT::endlog();
        return false;
    }

    if(!(std::find(_controllers_name.begin(), _controllers_name.end(), controlMode) != _controllers_name.end())){
        RTT::log(RTT::Warning) << "Control Mode " << controlMode << " is not available!" << RTT::endlog();
        return false;
    }

    if(controlMode == ControlModes::JointPositionCtrl){

            setInitialPosition();
    }
    else if(controlMode == ControlModes::JointTorqueCtrl || controlMode == ControlModes::JointImpedanceCtrl){

    }

    _current_control_mode = controlMode;
    return true;
}

void KinematicChain::sense()
{
    ///ROBOLLI IMPLEMENTATION!
//	if (full_feedback) {
//		for (unsigned int i = 0; i < _number_of_dofs; ++i)
//			full_feedback->joint_feedback.angles(i) = _model->GetJoint(
//					_joint_names[i])->GetAngle(0).Radian();

//		for (unsigned int i = 0; i < _number_of_dofs; ++i)
//			full_feedback->joint_feedback.velocities(i) = _model->GetJoint(
//					_joint_names[i])->GetVelocity(0);

//		for (unsigned int i = 0; i < _number_of_dofs; ++i) {
//			gazebo::physics::JointWrench w =
//					_model->GetJoint(_joint_names[i])->GetForceTorque(0u);
//			gazebo::math::Vector3 a =
//					_model->GetJoint(_joint_names[i])->GetLocalAxis(0u);
//			full_feedback->joint_feedback.torques(i) = a.Dot(w.body1Torque);
//		}

//		if (full_feedback->orocos_port.connected())
//			full_feedback->orocos_port.write(full_feedback->joint_feedback);
//	}
}

void KinematicChain::getCommand()
{
    if(_current_control_mode == ControlModes::JointTorqueCtrl)
        torque_controller->joint_cmd_fs = torque_controller->orocos_port.readNewest(
                    torque_controller->joint_cmd);
    else if(_current_control_mode == ControlModes::JointPositionCtrl)
        position_controller->joint_cmd_fs = position_controller->orocos_port.readNewest(
                    position_controller->joint_cmd);
    else if(_current_control_mode == ControlModes::JointImpedanceCtrl){
        position_controller->joint_cmd_fs = position_controller->orocos_port.readNewest(
                    position_controller->joint_cmd);
        impedance_controller->joint_cmd_fs = impedance_controller->orocos_port.readNewest(
                    impedance_controller->joint_cmd);
        torque_controller->joint_cmd_fs = torque_controller->orocos_port.readNewest(
                    torque_controller->joint_cmd);
    }
}

void KinematicChain::move()
{
    ///NEEDS ROBOLLI IMPLEMENTATION

//    if(_current_control_mode == ControlModes::JointPositionCtrl){
//        std::vector<std::string> joint_scoped_names = getJointScopedNames();
//        for(unsigned int i = 0; i < joint_scoped_names.size(); ++i)
//            _gazebo_position_joint_controller->SetPositionTarget(joint_scoped_names[i], position_controller->joint_cmd.angles(i));
//        _gazebo_position_joint_controller->Update();
//    }
//    else if(_current_control_mode == ControlModes::JointTorqueCtrl){
//        for(unsigned int i = 0; i < _joint_names.size(); ++i)
//            _model->GetJoint(_joint_names[i])->SetForce(0, torque_controller->joint_cmd.torques(i));
//    }
//    else if(_current_control_mode == ControlModes::JointImpedanceCtrl){
//        for(unsigned int i = 0; i < _joint_names.size(); ++i){
//            double q = full_feedback->joint_feedback.angles[i];
//            double qd = position_controller->joint_cmd.angles[i];
//            double Kd = impedance_controller->joint_cmd.stiffness[i];
//            double qdot = full_feedback->joint_feedback.velocities[i];
//            double Dd = impedance_controller->joint_cmd.damping[i];
//            double tauoff = torque_controller->joint_cmd.torques[i];
//            double tau = -Kd*(q-qd)-Dd*qdot+tauoff;
//            _model->GetJoint(_joint_names[i])->SetForce(0, tau);
//        }
//    }
}

std::string KinematicChain::printKinematicChainInformation()
{
    std::stringstream joint_names_stream;
    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        joint_names_stream << _joint_names[i] << " ";

    std::vector<std::string> controller_names = getControllersAvailable();
    std::stringstream controller_names_stream;
    for(unsigned int i = 0; i < controller_names.size(); ++i)
        controller_names_stream << controller_names[i] << " ";

    std::stringstream info;
    info << "Kinematic Chain: " << _kinematic_chain_name << std::endl;
    info << "    Number of DOFs: " << _number_of_dofs << std::endl;
    info << "    Joints:  [" << joint_names_stream.str() << "]" << std::endl;
    info << "    Control Modes:  [ " << controller_names_stream.str() << "]" << std::endl;
    info << "    Current Control Mode: " << _current_control_mode << std::endl;

    return info.str();
}
