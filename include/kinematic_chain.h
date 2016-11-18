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

class boardsID
{
public:
    /**
     * @brief boardsID map joint ID with boards ID
     * @param i 0 = COMAN_NO_FOREARMS, 1 = COMAN_FOREARMS
     */
    boardsID(const unsigned int i, const std::string& kinematic_chain_name):
        kin_chain_name(kinematic_chain_name)
    {
        if(0){
            r_leg = std::vector<int>{  4,  6,  7,  8,  9, 10};
            l_leg = std::vector<int>{  5, 11, 12, 13, 14, 15};
            waist = std::vector<int>{  1,  2, 3};
            r_arm = std::vector<int>{ 16, 17, 18 ,19};
            l_arm = std::vector<int>{ 20, 21, 22, 23};
            neck  = std::vector<int>{ }; //{ 24, 25};
        }
        else if(1){
            r_leg = std::vector<int>{  4,  6,  7,  8,  9, 10};
            l_leg = std::vector<int>{  5, 11, 12, 13, 14, 15};
            waist = std::vector<int>{  1,  2, 3};
            r_arm = std::vector<int>{ 16, 17, 18 ,19};
            l_arm = std::vector<int>{ 20, 21, 22, 23};
            neck  = std::vector<int>{ }; //{ 24, 25};
        }
    }

    std::vector<int> getBoardsID()
    {
        if(kin_chain_name.compare("left_arm") == 0)
            return l_arm;
        if(kin_chain_name.compare("left_leg") == 0)
            return l_leg;
        if(kin_chain_name.compare("right_leg") == 0)
            return r_leg;
        if(kin_chain_name.compare("right_arm") == 0)
            return r_arm;
        if(kin_chain_name.compare("torso") == 0)
            return waist;
    }

private:
    std::vector<int> r_leg;
    std::vector<int> l_leg;
    std::vector<int> waist;
    std::vector<int> r_arm;
    std::vector<int> l_arm;
    std::vector<int> neck;

    std::string kin_chain_name;
};

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
    boost::shared_ptr<boardsID> _boardsID;
};



#endif
