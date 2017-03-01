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

namespace cogimon {

class rtt_coman: public RTT::TaskContext {
public:
    rtt_coman(std::string const& name);
    bool configureHook();
    void updateHook();
    bool startHook();
    void stopHook();

    virtual ~rtt_coman() {}

protected:
    bool setControlMode(const std::string& kinematic_chain, const std::string& controlMode);
    std::vector<std::string> getKinematiChains();
    std::string getControlMode(const std::string& kinematic_chain);
    std::vector<std::string> getControlAvailableMode(const std::string& kinematic_chain);
    std::string printKinematicChainInformation(const std::string& kinematic_chain);
    bool loadYAML(const std::string& YAML_path);
    bool loadURDFAndSRDF(const std::string& URDF_path, const std::string& SRDF_path);
    std::map<std::string, std::vector<std::string> > getKinematiChainsAndJoints();
    void setOffSet(const std::string& joint_name, const double offset);
    bool setPID(const std::string& kinematic_chain, const std::vector<int>& P,
                const std::vector<int>& I, const std::vector<int>& D);




    std::map<std::string, boost::shared_ptr<KinematicChain>> kinematic_chains;

    XBot::XBotCoreModel _xbotcore_model;
    bool _models_loaded;
    bool _yaml_is_loaded;

    gain_parser gains;


private:
    bool is_configured;
    bool is_controlled;

    boost::shared_ptr<Boards_ctrl_ext> _boards;

    ts_bc_data_t   _ts_bc_data[MAX_DSP_BOARDS];
    int _tx_position_desired_mRAD[MAX_DSP_BOARDS];
};

}
#endif
