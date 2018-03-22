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
    TaskContext(name)
{
   

}


bool rtt_coman::configureHook() {

    setCpuAffinity(2);
    RTT::log(RTT::Info) << "---------------------------------------configureHook()" << RTT::endlog();
    return true;

}

void rtt_coman::updateHook(){
   
    RTT::log(RTT::Info) << "updateHook()" << RTT::endlog();
//     sleep(1);
    return;
}

bool rtt_coman::startHook()
{
    RTT::log(RTT::Info) << "*****************************************startHook()" << RTT::endlog();
    return true; 
}


void rtt_coman::stopHook()
{
    RTT::log(RTT::Info) << "stopHook()" << RTT::endlog();
}

ORO_CREATE_COMPONENT_LIBRARY()
//ORO_CREATE_COMPONENT(cogimon::robotSim)
ORO_LIST_COMPONENT_TYPE(cogimon::rtt_coman)
