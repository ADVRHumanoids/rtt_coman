/*
 * Copyright (C) 2016 Cogimon
 * Author: Enrico Mingo Hoffman
 * email:  enrico.mingo@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <kinematic_chain.h>

using namespace cogimon;

KinematicChain::KinematicChain(const std::string& chain_name, const std::vector<std::string> &joint_names,
                               RTT::DataFlowInterface& ports, ts_bc_data_t* boards_data,
                               boost::shared_ptr<Boards_ctrl_ext>& boards):
    _kinematic_chain_name(chain_name),
    _ports(ports),
    _current_control_mode(""),
    _joint_names(joint_names),
    _boards_data(boards_data),
    _boards(boards)
{
    RTT::log(RTT::Info) << "Creating Kinematic Chain " << chain_name << RTT::endlog();


    RTT::log(RTT::Info) << "Joints: " << RTT::endlog();
    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        RTT::log(RTT::Info) << "    " << _joint_names[i] << RTT::endlog();

    //HARDCODED STUFFS...SORRY...
    if(chain_name.compare("left_arm") == 0 && _joint_names.size() == 4)
    {
        RTT::log(RTT::Info) <<"COMAN_NO_FOREARMS, left_arm"<< RTT::endlog();
        _boardsID.reset(new boardsID(0, chain_name));
    }
    else if(chain_name.compare("right_arm") == 0 && _joint_names.size() == 4)
    {
        RTT::log(RTT::Info) <<"COMAN_NO_FOREARMS, right_arm"<< RTT::endlog();
        _boardsID.reset(new boardsID(0, chain_name));
    }
    else
        _boardsID.reset(new boardsID(1, chain_name));

    RTT::log(RTT::Info)<<"Boards ID: "<<RTT::endlog();
    for(unsigned int i = 0; i < _boardsID->boards_id.size(); ++i)
        RTT::log(RTT::Info) << "    " << _boardsID->boards_id[i] << RTT::endlog();

    _number_of_dofs = _joint_names.size();
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

//    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointPositionCtrl)) != controllers.end()){
//        setInitialPosition();
//    }

//    if(std::find(controllers.begin(), controllers.end(), std::string(ControlModes::JointImpedanceCtrl)) != controllers.end()){
//        setInitialImpedance();
//        RTT::log(RTT::Info)<<"Initial Impedance set!"<<RTT::endlog();
//    }

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
        full_feedback->orocos_port.doc("Output for Joint-fb from Robolli to Orocos world. Contains joint-position, -velocity and -torque.");
        _ports.addPort(full_feedback->orocos_port);

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

        position_controller->joint_cmd = JointAngles(_number_of_dofs);
        position_controller->joint_cmd.angles.setZero();

        voltage_offset.reset(new position_ctrl);
        voltage_offset->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointPositionCtrl+"_VoltageOffset");
        voltage_offset->orocos_port.doc("Input for voltageOffest-cmds from Orocos to Robolli.");
        _ports.addPort(voltage_offset->orocos_port);

        voltage_offset->joint_cmd = JointAngles(_number_of_dofs);
        voltage_offset->joint_cmd.angles.setZero();
        //ROBOLLI????
    }
    else if(controller_type == ControlModes::JointImpedanceCtrl){
        impedance_controller.reset(new impedance_ctrl);
        impedance_controller->orocos_port.setName(_kinematic_chain_name+"_"+ControlModes::JointImpedanceCtrl);
        impedance_controller->orocos_port.doc("Input for JointImpedance-cmds from Orocos to Robolli.");
        _ports.addPort(impedance_controller->orocos_port);

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

        torque_controller->joint_cmd = JointTorques(_number_of_dofs);
        torque_controller->joint_cmd.torques.setZero();
    }
    else{
        RTT::log(RTT::Error) << "Control Mode: " << controller_type << " is not available!" << RTT::endlog();
        return false;}
    _controllers_name.push_back(controller_type);
    return true;
}

void KinematicChain::setInitialPosition()
{
    position_controller->orocos_port.clear();
    //Here I have to read from robolli actual position of the joints
    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        position_controller->joint_cmd.angles[i] = getPosition(_joint_names[i]);

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

std::vector<int> KinematicChain::getBoardsID()
{
    return _boardsID->boards_id;
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
            for(unsigned int i = 0; i < _boardsID->boards_id.size(); ++i){
                _boards->start_stop_single_control(
                            _boardsID->boards_id[i], false);
                _boards->start_stop_single_control(
                            _boardsID->boards_id[i], true);
            }
            setInitialPosition();
    }
    else if(controlMode == ControlModes::JointImpedanceCtrl){
        for(unsigned int i = 0; i < _joint_names.size(); ++i){
            if(!setImpedanceCtrl(_joint_names[i])){
                RTT::log(RTT::Error)<<"Can not set Impedance Ctrl to joint "<<_joint_names[i]<<RTT::endlog();
                return false;}
        }
    }
    else if(controlMode == ControlModes::JointTorqueCtrl){

    }

    _current_control_mode = controlMode;
    return true;
}

void KinematicChain::sense()
{
    if (full_feedback) {
        for (unsigned int i = 0; i < _number_of_dofs; ++i){
            full_feedback->joint_feedback.angles(i) = getPosition(_joint_names[i]);
            full_feedback->joint_feedback.velocities(i) = getVelocity(_joint_names[i]);
            full_feedback->joint_feedback.torques(i) = getTorque(_joint_names[i]);}


        if (full_feedback->orocos_port.connected())
            full_feedback->orocos_port.write(full_feedback->joint_feedback);
    }
}

void KinematicChain::getCommand()
{
    if(_current_control_mode == ControlModes::JointTorqueCtrl)
        torque_controller->joint_cmd_fs = torque_controller->orocos_port.readNewest(
                    torque_controller->joint_cmd);
    else if(_current_control_mode == ControlModes::JointPositionCtrl){
        position_controller->joint_cmd_fs = position_controller->orocos_port.readNewest(
                    position_controller->joint_cmd);
        voltage_offset->joint_cmd_fs = voltage_offset->orocos_port.readNewest(
                    voltage_offset->joint_cmd);
    }
    else if(_current_control_mode == ControlModes::JointImpedanceCtrl){
        position_controller->joint_cmd_fs = position_controller->orocos_port.readNewest(
                    position_controller->joint_cmd);
        impedance_controller->joint_cmd_fs = impedance_controller->orocos_port.readNewest(
                    impedance_controller->joint_cmd);
        torque_controller->joint_cmd_fs = torque_controller->orocos_port.readNewest(
                    torque_controller->joint_cmd);
    }
}

void KinematicChain::move(int* _tx_position_desired_mRAD, short* _tx_voltage_desired_mV)
{
        if(_current_control_mode == ControlModes::JointPositionCtrl){
            for(unsigned int i = 0; i < _boardsID->boards_id.size(); ++i){
                _tx_position_desired_mRAD[_boardsID->boards_id[i]-1] =
                        (int)(RAD2mRAD(position_controller->joint_cmd.angles[i]-
                                 _boardsID->offsets[_boardsID->boards_id[i]]));
                _tx_voltage_desired_mV[_boardsID->boards_id[i]-1] =
                            (short)(V2mV(voltage_offset->joint_cmd.angles[i]));

            }
        }
}

//void KinematicChain::move()
//{
//    std::vector<int> _tx_position_desired_mRAD(_number_of_dofs);
//    if(_current_control_mode == ControlModes::JointPositionCtrl){
//        for(unsigned int i = 0; i < _boardsID->boards_id.size(); ++i){
//            _tx_position_desired_mRAD[i] =
//                    (int)(RAD2mRAD(position_controller->joint_cmd.angles[i]-
//                             _boardsID->offsets[_boardsID->boards_id[i]]));
//            RTT::log(RTT::Info)<<"position_controller->joint_cmd.angles "<<i<<" "<<position_controller->joint_cmd.angles[i]<<RTT::endlog();

//            RTT::log(RTT::Info)<<"_tx_position_desired_mRAD "<<i<<" "<<_tx_position_desired_mRAD[i]<<RTT::endlog();

//            _boards->set_position_group(
//                std::vector<uint8_t>(_boardsID->boards_id.begin(),_boardsID->boards_id.end()).data(),
//                        _tx_position_desired_mRAD.data(), _boardsID->boards_id.size());
//        }
//        RTT::log(RTT::Info)<<RTT::endlog();
//    }


////    if(_current_control_mode == ControlModes::JointPositionCtrl){
////        std::vector<std::string> joint_scoped_names = getJointScopedNames();
////        for(unsigned int i = 0; i < joint_scoped_names.size(); ++i)
////            _gazebo_position_joint_controller->SetPositionTarget(joint_scoped_names[i], position_controller->joint_cmd.angles(i));
////        _gazebo_position_joint_controller->Update();
////    }
////    else if(_current_control_mode == ControlModes::JointTorqueCtrl){
////        for(unsigned int i = 0; i < _joint_names.size(); ++i)
////            _model->GetJoint(_joint_names[i])->SetForce(0, torque_controller->joint_cmd.torques(i));
////    }
////    else if(_current_control_mode == ControlModes::JointImpedanceCtrl){
////        for(unsigned int i = 0; i < _joint_names.size(); ++i){
////            double q = full_feedback->joint_feedback.angles[i];
////            double qd = position_controller->joint_cmd.angles[i];
////            double Kd = impedance_controller->joint_cmd.stiffness[i];
////            double qdot = full_feedback->joint_feedback.velocities[i];
////            double Dd = impedance_controller->joint_cmd.damping[i];
////            double tauoff = torque_controller->joint_cmd.torques[i];
////            double tau = -Kd*(q-qd)-Dd*qdot+tauoff;
////            _model->GetJoint(_joint_names[i])->SetForce(0, tau);
////        }
////    }
//}

std::string KinematicChain::printKinematicChainInformation()
{
    std::stringstream joint_names_stream;
    for(unsigned int i = 0; i < _joint_names.size(); ++i)
        joint_names_stream << _joint_names[i] << " ";

    std::vector<std::string> controller_names = getControllersAvailable();
    std::stringstream controller_names_stream;
    for(unsigned int i = 0; i < controller_names.size(); ++i)
        controller_names_stream << controller_names[i] << " ";

    std::vector<int> boards_id = _boardsID->boards_id;
    std::stringstream boards_id_stream;
    for(unsigned int i = 0; i < boards_id.size(); ++i)
        boards_id_stream << boards_id[i] << " ";

    std::stringstream info;
    info << "Kinematic Chain: " << _kinematic_chain_name << std::endl;
    info << "    Number of DOFs: " << _number_of_dofs << std::endl;
    info << "    Joints:  [" << joint_names_stream.str() << "]" << std::endl;
    info << "    Control Modes:  [" << controller_names_stream.str() << "]" << std::endl;
    info << "    Current Control Mode: " << _current_control_mode << std::endl;
    info << "    Boards id: [" << boards_id_stream.str() <<"]" << std::endl;


    return info.str();
}

bool KinematicChain::setImpedanceCtrl(const int ID, const bool pure_torque)
{
    //Steps:
    //1) Set PID Gains to 0 and Torque Gains to 0
    if(setPID(ID, 0, 0, 0) && setPIDTorque(ID, 0, 0, 0))
    {
        McBoard* mc_board =  _boards->get_mc_board(ID);
        _boards->start_stop_single_control(ID, false);

        //4)Set bites 10 to 1 in configuration2 mask
        int tries = 0;
        while(mc_board->getItem(GET_MOTOR_CONFIG2, NULL, 0, REPLY_MOTOR_CONFIG2,
                          &(mc_board->_motor_config2),
                          sizeof(mc_board->_motor_config2)) != 0)
        {
            tries++;
            if(tries == 10)
                return false;
        }
        short int param = mc_board->_motor_config2;
        param |= 0x0400;
        if(mc_board->setItem(SET_MOTOR_CONFIG2, &param, sizeof(param)))
            return false;


        //3)Set bites 14, 11 and 1 to 1 in configuration mask
        tries = 0;
        while(mc_board->getItem(GET_MOTOR_CONFIG, NULL, 0, REPLY_MOTOR_CONFIG,
                          &(mc_board->_motor_config),
                          sizeof(mc_board->_motor_config)) != 0)
        {
            tries++;
            if(tries == 10)
                return false;
        }
        param = mc_board->_motor_config;
        param |= 0x4802;
        if(mc_board->setItem(SET_MOTOR_CONFIG, &param, sizeof(param)))
            return false;

        //2)Set Torque ON/OFF flag to 1
        mc_board->_torque_on_off = 1;
        mc_board->setItem(SET_TORQUE_ON_OFF, &(mc_board->_torque_on_off),
                          mc_board->_torque_on_off);

        //5)Set PID gains to (5000, 0, 500) and Torque gains to (445, 22, 0)
        if(!pure_torque){
            if(setPID(ID, 5000, 0, 500) && setPIDTorque(ID, 445, 22, 0)){
                _boards->start_stop_single_control(ID, true);
                return true;}
        }
        else{
            if(setPIDTorque(ID, 445, 22, 0))
                return true;
        }
    }
    return false;
}
