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
    _yaml_is_loaded(false)
{
//    this->addOperation("setControlMode", &rtt_coman::setControlMode,
//                this, RTT::ClientThread);

//    this->addOperation("getKinematicChains", &rtt_coman::getKinematiChains,
//                this, RTT::ClientThread);

//    this->addOperation("getKinematicChainsAndJoints", &rtt_coman::getKinematiChainsAndJoints,
//                this, RTT::ClientThread);

//    this->addOperation("printKinematicChainInformation", &rtt_coman::printKinematicChainInformation,
//                this, RTT::ClientThread);

//    this->addOperation("getControlMode", &rtt_coman::getControlMode,
//                this, RTT::ClientThread);

//    this->addOperation("getAvailableControlMode", &rtt_coman::getControlAvailableMode,
//                this, RTT::ClientThread);

    this->addOperation("loadYAML", &rtt_coman::loadYAML,
                this, RTT::ClientThread);

    this->addOperation("loadURDFAndSRDF", &rtt_coman::loadURDFAndSRDF,
                this, RTT::ClientThread);
}

bool rtt_coman::loadYAML(const std::string &YAML_path)
{
    if(!_yaml_is_loaded){
        //_boards.reset(new Boards_ctrl_ext(YAML_path.c_str()));

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
        //_boards->init();

        int xbotcoremodel_number_of_joints = 0;
        for(unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i){
            std::string chain = _xbotcore_model.get_chain_names()[i];
            std::vector<std::string> enabled;
            _xbotcore_model.get_enabled_joints_in_chain(chain, enabled);
            xbotcoremodel_number_of_joints += enabled.size();
        }

        RTT::log(RTT::Info) << "Model name "<< _xbotcore_model.getName() << RTT::endlog();
        RTT::log(RTT::Info) << "Model has " << xbotcoremodel_number_of_joints << " joints" << RTT::endlog();
//        RTT::log(RTT::Info) << "Robot has " << _boards->getActiveNum() << " boards" << RTT::endlog();

//        if(_boards->getActiveNum() != _xbotcore_model.get_joint_num())
//            return is_configured;



        for(unsigned int i = 0; i < _xbotcore_model.get_chain_names().size(); ++i){
            std::string chain_name = _xbotcore_model.get_chain_names()[i];
            std::vector<std::string> enabled_joints_in_chain;
            _xbotcore_model.get_enabled_joints_in_chain(chain_name,enabled_joints_in_chain);

            kinematic_chains.insert(std::pair<std::string, boost::shared_ptr<KinematicChain>>(
                chain_name, boost::shared_ptr<KinematicChain>(
                    new KinematicChain(chain_name, enabled_joints_in_chain, *(this->ports())))));
        }
        RTT::log(RTT::Info) << "Kinematic Chains map created!" << RTT::endlog();
    }

    if (!_models_loaded)
        RTT::log(RTT::Error) << "URDF and SRDF models has not been passed. Call loadURDFAndSRDF(URDF_path, SRDF_path)" << RTT::endlog();

    if (!_yaml_is_loaded)
        RTT::log(RTT::Error) << "YAML config has not been passed. Call loadYAML(YAML_path)" << RTT::endlog();

    return is_configured;
}

void rtt_coman::updateHook(){

}

ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(cogimon::robotSim)
ORO_LIST_COMPONENT_TYPE(cogimon::rtt_coman)
