#ifndef _RTT_COMAN_KINEMATIC_CHAIN_H_
#define _RTT_COMAN_KINEMATIC_CHAIN_H_

// RST-RT includes
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rst-rt/robot/JointState.hpp>

#include <control_modes.h>
#include <parser.h>


using namespace rstrt::kinematics;
using namespace rstrt::dynamics;
using namespace rstrt::robot;


typedef cogimon::jointCtrl<JointAngles> position_ctrl;
typedef cogimon::jointCtrl<JointImpedance> impedance_ctrl;
typedef cogimon::jointCtrl<JointTorques> torque_ctrl;

typedef cogimon::jointFeedback<JointState> full_fbk;


class KinematicChain {
public:
    KinematicChain(const std::string& chain_name, const std::vector<std::string>& joint_names,
                   RTT::DataFlowInterface& ports);
    ~KinematicChain(){}

    std::string getKinematicChainName();
    unsigned int getNumberOfDOFs();
    std::string getCurrentControlMode();
    std::vector<std::string> getJointNames();
    std::vector<std::string> getControllersAvailable();
    bool initKinematicChain(const cogimon::gains& gains_);
    bool setControlMode(const std::string& controlMode);
    void sense();
    void getCommand();
    void move();
    std::string printKinematicChainInformation();
    std::vector<RTT::base::PortInterface*> getAssociatedPorts();

    boost::shared_ptr<position_ctrl> position_controller;
    boost::shared_ptr<impedance_ctrl> impedance_controller;
    boost::shared_ptr<torque_ctrl> torque_controller;

    boost::shared_ptr<full_fbk> full_feedback;

private:
    std::string _kinematic_chain_name;
    std::vector<std::string> _controllers_name;
    unsigned int _number_of_dofs;
    RTT::DataFlowInterface& _ports;

    std::vector<RTT::base::PortInterface*> _inner_ports;

    std::string _current_control_mode;
    std::vector<std::string> _joint_names;
    std::map<std::string, std::string> _map_joint_name_scoped_name;


    bool setController(const std::string& controller_type);
    void setFeedBack();
    bool setJointNamesAndIndices();
    void setInitialPosition();
    void setInitialImpedance();

    boost::shared_ptr<cogimon::gains> _gains;
};



#endif
