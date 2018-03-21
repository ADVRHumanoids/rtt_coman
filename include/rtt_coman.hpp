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

#include <force_torque_sensor.h>

namespace cogimon {
/**
 * @brief The rtt_coman class  creates an OROCOS component which permits to control the humanoid robot COMAN
 */

class rtt_coman: public RTT::TaskContext {
public:
    /**
     * @brief rtt_coman
     * @param name is the name used for the OROCOS task
     */
    rtt_coman(std::string const& name);

    /**
     * @brief configureHook here all the ports related to kinematic chains and sensors are created, furthermore the robolli thread is started
     * @return
     */
    bool configureHook();

    /**
     * @brief updateHook sensors are read from robolli and broadcasted to OROCOS, commands from OROCOS are sent to robolli
     */
    void updateHook();

    /**
     * @brief startHook here we assume the robots start in PositionCtrl and we force this control mode, we also starts all the boards
     * @return
     */
    bool startHook();

    /**
     * @brief stopHook here we stops all the boards
     */
    void stopHook();

    virtual ~rtt_coman() {}

protected:
    /**
     * @brief setControlMode set a controlMode for a particular kinematic chain, it is provided as an OROCOS Operation
     * @param kinematic_chain string which identify the kineamtic chain (from the SRDF)
     * @param controlMode one possible control mode (listed in control_modes.h)
     * @return true if succeed (the failure may be caused by wrong kineamtic chains name as well as errors from the librobolli).
     * NOTE: passing from impedance to position will not change control mode (will remain in impedance)
     */
    bool setControlMode(const std::string& kinematic_chain, const std::string& controlMode);

    /**
     * @brief getKinematiChains return a vector with all the kinematic chains which the robot is constituted, it is provided as an OROCOS Operation
     * @return a vector of string with the names of the kinematic chains
     */
    std::vector<std::string> getKinematiChains();

    /**
     * @brief getControlMode return a string with the actual control mode of the kinematic chain, it is provided as an OROCOS Operation
     * @param kinematic_chain which you want to know the control mode
     * @return "" if the kinematic chain does not exists
     */
    std::string getControlMode(const std::string& kinematic_chain);

    /**
     * @brief getControlAvailableMode return the control modes available for that kinematic chain, it is provided as an OROCOS Operation
     * @param kinematic_chain which you want to know the control modes
     * @return "" if the kinematic chain does not exists
     */
    std::vector<std::string> getControlAvailableMode(const std::string& kinematic_chain);

    /**
     * @brief printKinematicChainInformation print some information about the kinematic chain, it is provided as an OROCOS Operation
     * @param kinematic_chain
     * @return
     */
    std::string printKinematicChainInformation(const std::string& kinematic_chain);

    /**
     * @brief loadYAML loads the YALM file needed by librobolli, it is provided as an OROCOS Operation
     * @param YAML_path
     * @return true if it is a valid YAML
     */
    bool loadYAML(const std::string& YAML_path);

    /**
     * @brief loadURDFAndSRDF uses URDF and SRDF to create the kinematic_chains structures, it is provided as an OROCOS Operation
     * @param URDF_path
     * @param SRDF_path
     * @return
     */
    bool loadURDFAndSRDF(const std::string& URDF_path, const std::string& SRDF_path);

    /**
     * @brief getKinematiChainsAndJoints return a map between kinematic chains and joints, this is usefull to know which joints there are in a
     * kinematic chain, it is provided as an OROCOS Operation
     * @return the map kinematic chain --> joints
     */
    std::map<std::string, std::vector<std::string> > getKinematiChainsAndJoints();

    /**
     * @brief setOffSet permits to set a fixed position offset for a joint will be taken into account when reading/writing joints positions, it is provided as an OROCOS Operation
     * @param joint_name name of the joint
     * @param offset value
     */
    void setOffSet(const std::string& joint_name, const double offset);

    /**
     * @brief setPID gains for the position control
     * NOTE: To have VoltageControl sets P = 0., I = 0. and D = 0. and use the offset port, it is provided as an OROCOS Operation
     * @param kinematic_chain which to set the PID gains
     * @param P
     * @param I
     * @param D
     * @return false if not in position mode or something goes wrong inside robolli
     */
    bool setPID(const std::string& kinematic_chain, const std::vector<int>& P,
                const std::vector<int>& I, const std::vector<int>& D);

    /**
     * @brief getForceTorqueSensorsFrames returns the frames of the ft sensors in a vector, it is provided as an OROCOS Operation
     * @return vector of strings
     */
    std::vector<std::string> getForceTorqueSensorsFrames();

    /**
     * @brief setForceTorqueMeasurementDirection set manually direction of force/torques for the measured wrenches
     * @param force_torque_frame which force torque sensor to change
     * @param directions vector of +1 or -1
     * @return false if force_torque_frame does not exist
     */
    bool setForceTorqueMeasurementDirection(const std::string& force_torque_frame,
                                            const std::vector<int>& directions);

    /**
     * @brief getCtrlBoardIndices retrieves IDs of control board used internally by robolli
     * @param kinematic_chain
     * @return empty vector if errors happens
     */
    std::vector<int> getCtrlBoardIndices(const std::string& kinematic_chain);

    /**
     * @brief getFTBoardIndex retrieves Force/Torque sensors index
     * @param force_torque_frame
     * @return -1 if error happens
     */
    int getFTBoardIndex(const std::string& force_torque_frame);
//    ///DEPRECATED
//    bool setImpedance(const std::string& kinematic_chain, const std::vector<int>& P,
//                const std::vector<int>& D);


    /**
     * @brief kinematic_chains map between kinematic chains name and structures
     */
    std::map<std::string, boost::shared_ptr<KinematicChain>> kinematic_chains;

    /**
     * @brief force_torque_sensors vector of force_torue_sensors structures
     */
    std::vector<force_torque_sensor> force_torque_sensors;

    /**
     * @brief _xbotcore_model
     */
    XBot::XBotCoreModel _xbotcore_model;

    /**
     * @brief _models_loaded variable to check if model is loaded
     */
    bool _models_loaded;

    /**
     * @brief _yaml_is_loaded variable to check if robolli yaml is loaded
     */
    bool _yaml_is_loaded;

    /**
     * @brief gains this is used to parse gains and control types
     */
    gain_parser gains;


private:
    /**
     * @brief is_configured variable to check if component is configured
     */
    bool is_configured;

    /**
     * @brief is_controlled variable to check if component is controlled, when false the component will not send commands to robolli
     */
    bool is_controlled;

    /**
     * @brief _boards this data structure handle the ethernet communication (robolli)
     */
    boost::shared_ptr<Boards_ctrl_ext> _boards;

    ts_bc_data_t   _ts_bc_data[MAX_DSP_BOARDS];
    int _tx_position_desired_mRAD[MAX_DSP_BOARDS];
    short _tx_voltage_desired_mV[MAX_DSP_BOARDS];
    int _tx_torque_desired_mNm[MAX_DSP_BOARDS];
};

}
#endif
