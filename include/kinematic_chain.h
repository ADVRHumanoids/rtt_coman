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

#include <Boards_ctrl_ext.h>
#include <utils.h>


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
        if(0){ //NO_FOREARMS
            r_arm = std::vector<int>{ 16, 17, 18 ,19};
            l_arm = std::vector<int>{ 20, 21, 22, 23};
        }
        else if(1){ //FOREARMS
            r_arm = std::vector<int>{ 16, 17, 18 ,19};
            l_arm = std::vector<int>{ 20, 21, 22, 23};

        }
        r_leg = std::vector<int>{  4,  6,  7,  8,  10, 9};
        l_leg = std::vector<int>{  5, 11, 12, 13, 15, 14};
        waist = std::vector<int>{  3,  2, 1};
        neck  = std::vector<int>{ }; //{ 24, 25};

        int s = r_leg.size() + l_leg.size() + waist.size() + neck.size() + r_arm.size() + l_arm.size();
        for(unsigned int i = 0; i < s; ++i)
            offsets.push_back(0.0);

        if(kin_chain_name.compare("left_arm") == 0)
            boards_id = l_arm;
        if(kin_chain_name.compare("left_leg") == 0)
            boards_id = l_leg;
        if(kin_chain_name.compare("right_leg") == 0)
            boards_id = r_leg;
        if(kin_chain_name.compare("right_arm") == 0)
            boards_id = r_arm;
        if(kin_chain_name.compare("torso") == 0)
            boards_id = waist;

    }

    std::vector<int> boards_id;
    std::vector<double> offsets;

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
                   RTT::DataFlowInterface& ports,
                   ts_bc_data_t* boards_data,
                   boost::shared_ptr<Boards_ctrl_ext>& boards);
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
    void move(int *_tx_position_desired_mRAD);
    std::string printKinematicChainInformation();
    std::vector<RTT::base::PortInterface*> getAssociatedPorts();
    std::vector<int> getBoardsID();

    boost::shared_ptr<position_ctrl> position_controller;
    boost::shared_ptr<impedance_ctrl> impedance_controller;
    boost::shared_ptr<torque_ctrl> torque_controller;

    boost::shared_ptr<full_fbk> full_feedback;

    void setOffSet(const std::string& joint_name, const double offset)
    {
        int pos = std::find(_joint_names.begin(), _joint_names.end(), joint_name) - _joint_names.begin();
        setOffSet(_boardsID->boards_id[pos], offset);
    }

private:
    std::string _kinematic_chain_name;
    std::vector<std::string> _controllers_name;
    unsigned int _number_of_dofs;
    RTT::DataFlowInterface& _ports;

    std::vector<RTT::base::PortInterface*> _inner_ports;

    std::string _current_control_mode;
    std::vector<std::string> _joint_names;


    bool setController(const std::string& controller_type);
    void setFeedBack();
    void setInitialPosition();
    void setInitialImpedance();

    boost::shared_ptr<cogimon::gains> _gains;
    boost::shared_ptr<boardsID> _boardsID;
    ts_bc_data_t* _boards_data;
    boost::shared_ptr<Boards_ctrl_ext> _boards;

    double getPosition(const int ID)
    {
        return mRAD2RAD(_boards_data[ID-1].raw_bc_data.mc_bc_data.Position)+_boardsID->offsets[ID];
    }

    double getPosition(const std::string& joint_name)
    {
        int pos = std::find(_joint_names.begin(), _joint_names.end(), joint_name) - _joint_names.begin();
        return getPosition(_boardsID->boards_id[pos]);
    }

    double getVelocity(const int ID)
    {
        return mRAD2RAD(_boards_data[ID-1].raw_bc_data.mc_bc_data.Velocity);
    }

    double getVelocity(const std::string& joint_name)
    {
        int pos = std::find(_joint_names.begin(), _joint_names.end(), joint_name) - _joint_names.begin();
        return getVelocity(_boardsID->boards_id[pos]);
    }

    double getTorque(const int ID)
    {
        return _boards_data[ID-1].raw_bc_data.mc_bc_data.Torque/1000.0;
    }

    double getTorque(const std::string& joint_name)
    {
        int pos = std::find(_joint_names.begin(), _joint_names.end(), joint_name) - _joint_names.begin();
        return getTorque(_boardsID->boards_id[pos]);
    }

    void setOffSet(const int ID, const double offset)
    {
        _boardsID->offsets[ID] = offset;
    }



};



#endif
