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

#include <rtt_coman.hpp>
#include <rtt/Operation.hpp>
#include <string>
#include <fstream>
#include <streambuf>


using namespace cogimon;
using namespace RTT;
using namespace RTT::os;
using namespace Eigen;

rtt_coman::rtt_coman(const std::string &name):
    TaskContext(name),
    is_configured(false),
    _models_loaded(false),
    _yaml_is_loaded(false),
    is_controlled(false)
{
//    this->addOperation("setControlMode", &rtt_coman::setControlMode,
//                this, RTT::ClientThread);

    this->addOperation("getKinematicChains", &rtt_coman::getKinematiChains,
                this, RTT::ClientThread);

    this->addOperation("getKinematicChainsAndJoints", &rtt_coman::getKinematiChainsAndJoints,
                this, RTT::ClientThread);

    this->addOperation("printKinematicChainInformation", &rtt_coman::printKinematicChainInformation,
                this, RTT::ClientThread);

    this->addOperation("getControlMode", &rtt_coman::getControlMode,
                this, RTT::ClientThread);

    this->addOperation("getAvailableControlMode", &rtt_coman::getControlAvailableMode,
                this, RTT::ClientThread);

    this->addOperation("loadYAML", &rtt_coman::loadYAML,
                this, RTT::ClientThread);

    this->addOperation("loadURDFAndSRDF", &rtt_coman::loadURDFAndSRDF,
                this, RTT::ClientThread);

    this->addOperation("setOffSet", &rtt_coman::setOffSet,
                this, RTT::ClientThread);

    this->addOperation("setPID", &rtt_coman::setPID,
                this, RTT::ClientThread);
}

void rtt_coman::setOffSet(const std::string& joint_name, const double offset)
{
    if(is_configured && !isRunning()){
        std::map<std::string, std::vector<std::string> > map_kin_chain_joints = getKinematiChainsAndJoints();
        std::map<std::string, std::vector<std::string> >::iterator it;
        for(it = map_kin_chain_joints.begin(); it != map_kin_chain_joints.end(); it++)
        {
            if (std::find(it->second.begin(), it->second.end(),joint_name)!=it->second.end())
                kinematic_chains[it->first]->setOffSet(joint_name, offset);
        }
    }
    if(!is_configured)
        RTT::log(RTT::Warning)<<"Component has to be configured before setting Offsets!"<<RTT::endlog();
    if(isRunning())
        RTT::log(RTT::Error)<<"Can NOT set Offsets when component is running!"<<RTT::endlog();
}

bool rtt_coman::loadYAML(const std::string &YAML_path)
{
    if(!_yaml_is_loaded){
        _boards.reset(new Boards_ctrl_ext(YAML_path.c_str()));

        _yaml_is_loaded = true;
    }
    else
        RTT::log(RTT::Info)<<"YAML config file have been already loaded!"<<RTT::endlog();

    return _yaml_is_loaded;
}

bool rtt_coman::loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path)
{
    if(_yaml_is_loaded){
        if(!_models_loaded)
        {
            std::string _urdf_path = URDF_path;
            std::string _srdf_path = SRDF_path;

            RTT::log(RTT::Info)<<"URDF path: "<<_urdf_path<<RTT::endlog();
            RTT::log(RTT::Info)<<"SRDF path: "<<_srdf_path<<RTT::endlog();

            _models_loaded = _xbotcore_model.init(_urdf_path, _srdf_path);
            _models_loaded = _models_loaded && gains.initFile(_srdf_path);

            for(unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i){
                RTT::log(RTT::Info)<<"chain #"<<i<<" "<<_xbotcore_model.get_chain_names()[i]<<RTT::endlog();
                std::vector<std::string> enabled_joints_in_chain_i;
                _xbotcore_model.get_enabled_joints_in_chain(_xbotcore_model.get_chain_names()[i], enabled_joints_in_chain_i);
                for(unsigned int j = 0; j < enabled_joints_in_chain_i.size(); ++j)
                    RTT::log(RTT::Info)<<"  "<<enabled_joints_in_chain_i[j]<<RTT::endlog();
            }
        }
        else
            RTT::log(RTT::Info)<<"URDF and SRDF have been already loaded!"<<RTT::endlog();
    }
    else
        RTT::log(RTT::Error) << "YAML models has not been passed. Call loadYAML(YAML_path)" << RTT::endlog();

    return _models_loaded;
}

bool rtt_coman::configureHook() {
    if(_yaml_is_loaded && _models_loaded && !is_configured)
    {
        _boards->init();

        int xbotcoremodel_number_of_joints = 0;
        for(unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i){
            std::string chain = _xbotcore_model.get_chain_names()[i];
            std::vector<std::string> enabled;
            _xbotcore_model.get_enabled_joints_in_chain(chain, enabled);
            xbotcoremodel_number_of_joints += enabled.size();
        }


        RTT::log(RTT::Info) << "Model name "<< _xbotcore_model.getName() << RTT::endlog();
        RTT::log(RTT::Info) << "Model has " << xbotcoremodel_number_of_joints << " joints" << RTT::endlog();
        RTT::log(RTT::Info) << "Robot has " << _boards->getMCSNum()<< " boards" << RTT::endlog();

        _boards->get_bc_data(_ts_bc_data); //IMPORTANT: this has to be done before the kinematic chains creation!

        for(unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i){
            std::string chain_name = _xbotcore_model.get_chain_names()[i];
            std::vector<std::string> enabled_joints_in_chain;
            _xbotcore_model.get_enabled_joints_in_chain(chain_name,enabled_joints_in_chain);

            kinematic_chains.insert(std::pair<std::string, boost::shared_ptr<KinematicChain>>(
                chain_name, boost::shared_ptr<KinematicChain>(
                    new KinematicChain(chain_name, enabled_joints_in_chain, *(this->ports()), _ts_bc_data,
                                        _boards))));

            RTT::log(RTT::Info)<<kinematic_chains[chain_name]->printKinematicChainInformation()<<RTT::endlog();
        }
        RTT::log(RTT::Info) << "Kinematic Chains map created!" << RTT::endlog();

        for(std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it = kinematic_chains.begin();
            it != kinematic_chains.end(); it++){
            if(!(it->second->initKinematicChain(gains.Gains))){
                RTT::log(RTT::Warning) << "Problem Init Kinematic Chain" <<
                    it->second->getKinematicChainName() << RTT::endlog();
                return false;
            }
        }
        RTT::log(RTT::Info) << "Kinematic Chains Initialized!" << RTT::endlog();

        RTT::log(RTT::Warning) << "Done configuring component" << RTT::endlog();
    }

    if (!_models_loaded)
        RTT::log(RTT::Error) << "URDF and SRDF models has not been passed. Call loadURDFAndSRDF(URDF_path, SRDF_path)" << RTT::endlog();

    if (!_yaml_is_loaded)
        RTT::log(RTT::Error) << "YAML config has not been passed. Call loadYAML(YAML_path)" << RTT::endlog();

    is_configured = true;
    return is_configured;
}

void rtt_coman::updateHook(){
    if (!is_configured && !isRunning())
        return;

    std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
    // GET COMMANDS:
    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->getCommand();

    // SENSE:
    _boards->get_bc_data(_ts_bc_data);
    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
        it->second->sense();

    // MOVE:
    if(is_controlled)
    {
        for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++)
            it->second->move(_tx_position_desired_mRAD);

        _boards->set_position(_tx_position_desired_mRAD,
                              sizeof(int)*MAX_MC_BOARDS);

    }

}

std::map<std::string, std::vector<std::string> > rtt_coman::getKinematiChainsAndJoints()
{
    std::map<std::string, std::vector<std::string> > kinematic_chains_and_joints_map;
    std::vector<std::string> chain_names = getKinematiChains();
    for(unsigned int i = 0; i < chain_names.size(); ++i){
        kinematic_chains_and_joints_map.insert(std::pair<std::string, std::vector<std::string>>(
            chain_names[i], kinematic_chains[chain_names[i]]->getJointNames()));
    }
    return kinematic_chains_and_joints_map;
}

std::vector<std::string> rtt_coman::getKinematiChains()
{
    std::vector<std::string> chains;
    for(std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it = kinematic_chains.begin();
        it != kinematic_chains.end(); it++)
        chains.push_back(it->second->getKinematicChainName());
    return chains;
}

std::vector<std::string> rtt_coman::getControlAvailableMode(const std::string& kinematic_chain)
{
    std::vector<std::string> control_modes;

    std::vector<std::string> chain_names = getKinematiChains();
    if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
        log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
        control_modes.push_back("");}
    else
        control_modes = kinematic_chains[kinematic_chain]->getControllersAvailable();
    return control_modes;
}

std::string rtt_coman::getControlMode(const std::string& kinematic_chain)
{
    std::vector<std::string> chain_names = getKinematiChains();
        if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
            log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
            return "";}

    return kinematic_chains[kinematic_chain]->getCurrentControlMode();
}

std::string rtt_coman::printKinematicChainInformation(const std::string& kinematic_chain)
{
    std::vector<std::string> chain_names = getKinematiChains();
    if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
        log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
        return "";}

    return kinematic_chains[kinematic_chain]->printKinematicChainInformation();
}

bool rtt_coman::startHook()
{
    std::map<std::string, boost::shared_ptr<KinematicChain>>::iterator it;
    int success = 0;

    for(it = kinematic_chains.begin(); it != kinematic_chains.end(); it++){
        std::vector<int> boards_id = it->second->getBoardsID();

        _boards->get_bc_data(_ts_bc_data);
        it->second->sense();

        it->second->setControlMode(ControlModes::JointPositionCtrl);
        it->second->move(_tx_position_desired_mRAD);

        for(unsigned int i = 0; i < boards_id.size(); ++i){
            success += _boards->start_stop_single_control(boards_id[i], true); //for now only position control
            RTT::log(RTT::Info)<<"Started board "<<boards_id[i]<<RTT::endlog();
            usleep(1e4);}
    }

    if(success == 0){
        is_controlled = true;
        return true;}
    return false;
}

bool rtt_coman::setControlMode(const std::string& kinematic_chain, const std::string& controlMode)
{
    std::vector<std::string> chain_names = getKinematiChains();
    if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
        log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
        return false;}

    _boards->get_bc_data(_ts_bc_data);
    kinematic_chains[kinematic_chain]->sense();

    bool a =  kinematic_chains[kinematic_chain]->setControlMode(controlMode);
    if(a){
        is_controlled = true;
        //change control mode in robolli, qui ci va una move?
    }
    else
        is_controlled = false;

    return is_controlled;
}

bool rtt_coman::setPID(const std::string &kinematic_chain, const std::vector<int> &P,
                       const std::vector<int> &I, const std::vector<int> &D)
{
    std::vector<std::string> chain_names = getKinematiChains();
    if(!(std::find(chain_names.begin(), chain_names.end(), kinematic_chain) != chain_names.end())){
        log(Warning) << "Kinematic Chain " << kinematic_chain << " is not available!" << endlog();
        return false;}

    std::vector<std::string> joint_names = kinematic_chains[kinematic_chain]->getJointNames();
    bool a = true;
    for(unsigned int i = 0; i < joint_names.size(); ++i)
        a = a && kinematic_chains[kinematic_chain]->setPID(joint_names[i], P[i], I[i], D[i]);
    return a;
}

void rtt_coman::stopHook()
{
    std::vector<std::string> chain_names = getKinematiChains();
    for(unsigned int i = 0; i < chain_names.size(); ++i)
    {
        std::vector<int> boards_id = kinematic_chains[chain_names[i]]->getBoardsID();
        for(unsigned int j = 0; j < boards_id.size(); ++j)
            _boards->start_stop_single_control(boards_id[j]-1, false);
    }
}

ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(cogimon::robotSim)
ORO_LIST_COMPONENT_TYPE(cogimon::rtt_coman)
