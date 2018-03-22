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

#include <boost/shared_ptr.hpp>

#include <srdfdom_advr/model.h>
#include <urdf/model.h>
#include <XBotCoreModel.h>

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

};

}
#endif
