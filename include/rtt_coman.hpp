#ifndef _RTT_COMAN_HPP_
#define _RTT_COMAN_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/Component.hpp>
#include <rtt/os/Semaphore.hpp>

#include <Eigen/Dense>

#include <vector>

#include <Eigen/Core>
#include <time.h>
#include <rtt/os/TimeService.hpp>
#include <sstream>
#include <rtt/Logger.hpp>

#include <thread>
#include <memory>


#include <control_modes.h>
#include <kinematic_chain.h>
#include <boost/shared_ptr.hpp>

#include <srdfdom_advr/model.h>
#include <urdf/model.h>
#include <XBotCoreModel.h>
#include <parser.h>

#include <Boards_ctrl_ext.h>
#include <utils.h>

namespace cogimon {

class rtt_coman: public RTT::TaskContext {
public:
    rtt_coman(std::string const& name);
    bool configureHook();
    void updateHook();
    virtual ~rtt_coman() {}

protected:
    //bool setControlMode(const std::string& kinematic_chain, const std::string& controlMode);
    std::vector<std::string> getKinematiChains();
    std::string getControlMode(const std::string& kinematic_chain);
    std::vector<std::string> getControlAvailableMode(const std::string& kinematic_chain);
    std::string printKinematicChainInformation(const std::string& kinematic_chain);
    bool loadYAML(const std::string& YAML_path);
    bool loadURDFAndSRDF(const std::string& URDF_path, const std::string& SRDF_path);
    std::map<std::string, std::vector<std::string> > getKinematiChainsAndJoints();
    void setOffSet(const std::string& joint_name, const double offset);




    std::map<std::string, boost::shared_ptr<KinematicChain>> kinematic_chains;

    XBot::XBotCoreModel _xbotcore_model;
    bool _models_loaded;
    bool _yaml_is_loaded;

    gain_parser gains;


private:
    bool is_configured;

    boost::shared_ptr<Boards_ctrl_ext> _boards;

    ts_bc_data_t   _ts_bc_data[MAX_DSP_BOARDS];
};

}
#endif
