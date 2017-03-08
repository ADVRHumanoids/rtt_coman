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

#ifndef MOTOR_CONTROLLER_TEST_H
#define MOTOR_CONTROLLER_TEST_H

#include <rtt/RTT.hpp>
#include <string>
#include <rst-rt/robot/JointState.hpp>
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/dynamics/JointImpedance.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/Port.hpp>
#include <XBotCoreModel.h>
#include <urdf/model.h>

class motor_controller_test: public RTT::TaskContext {
public:
    motor_controller_test(std::string const & name);

    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();

private:
    std::string _urdf_path;
    std::string _srdf_path;
    boost::shared_ptr<urdf::ModelInterface const> _urdf_model;
    std::vector<std::string> _joint_list;
    double _dt_ms;
    std::string _robot_name;
    std::map<std::string, std::vector<std::string> > _map_kin_chains_joints;

    std::map<std::string, double> _map_chain_trj_time;
    std::map<std::string, rstrt::robot::JointState> _map_chain_q0;
    std::map<std::string, bool> _map_chain_start_trj;
    std::map<std::string, bool> _map_chain_start_voltage_trj;
    std::map<std::string, bool> _map_chain_start_torque_trj;
    std::map<std::string, bool> _map_chain_start_impedance_trj;

    std::map<std::string, boost::shared_ptr<RTT::InputPort<rstrt::robot::JointState> > > _kinematic_chains_feedback_ports;
    std::map<std::string, rstrt::robot::JointState> _kinematic_chains_joint_state_map;

    std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > > _kinematic_chains_output_ports;
    std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::kinematics::JointAngles> > > _kinematic_chains_output_voltage_ports;
    std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::dynamics::JointTorques> > > _kinematic_chains_output_torques_ports;
    std::map<std::string, boost::shared_ptr<RTT::OutputPort<rstrt::dynamics::JointImpedance> > > _kinematic_chains_output_impedance_ports;
    std::map<std::string, rstrt::kinematics::JointAngles> _kinematic_chains_desired_joint_state_map;
    std::map<std::string, rstrt::kinematics::JointAngles> _kinematic_chains_desired_joint_voltage_offset_map;
    std::map<std::string, rstrt::dynamics::JointTorques> _kinematic_chains_desired_joint_torque_map;
    std::map<std::string, rstrt::dynamics::JointImpedance> _kinematic_chains_desired_joint_impedance_map;


    std::map<std::string, std::pair<double, double>> _map_joint_limimts;

    bool loadURDFAndSRDF(const std::string &URDF_path, const std::string &SRDF_path);
    bool attachToRobot(const std::string& robot_name);
    bool startTrj(const std::string& chain_name);
    bool stopTrj(const std::string& chain_name);

    bool startVoltageOffset(const std::string& chain_name, const std::vector<double>& offset);
    bool stopVoltageOffset(const std::string& chain_name);

    bool startImpedanceConst(const std::string& chain_name, const std::vector<double>& stiffness,
                             const std::vector<double>& damping);
    bool stopImpedanceConst(const std::string& chain_name);

    bool startTorqueTrj(const std::string& chain_name);
    bool stopTorqueTrj(const std::string& chain_name);

    double sin_traj(double q0, double amplitude, double t, double period){
        return q0 + amplitude*std::sin(t/(period*M_PI));
    }

    rstrt::kinematics::JointAngles sin_traj(const rstrt::robot::JointState& q0, double amplitude, double t, double period)
    {
        rstrt::kinematics::JointAngles tmp(q0.angles.rows());
        for(unsigned int i = 0; i < q0.angles.rows(); ++i)
            tmp.angles[i] = sin_traj(q0.angles[i], amplitude, t, period);
        return tmp;
    }

    void getJointLimits(std::map<std::string, std::pair<double, double>>& map);
};

#endif
