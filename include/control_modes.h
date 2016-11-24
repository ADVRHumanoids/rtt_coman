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

#ifndef _CONTROL_MODES_RTT_COMAN_H_
#define _CONTROL_MODES_RTT_COMAN_H_

#include <string>
#include <rtt/Port.hpp>

///TODO: This structs are copy-paste of control_modes in rtt-gazebo-robot-sim,
/// maybe we can make a library then...

namespace cogimon {

    struct ControlModes{
            static constexpr const char* JointPositionCtrl = "JointPositionCtrl";
            static constexpr const char* JointTorqueCtrl = "JointTorqueCtrl";
            static constexpr const char* JointImpedanceCtrl = "JointImpedanceCtrl";
    };

    struct FeedbackModes {
        static constexpr const char* velocityFeedback = "JointVelocity";
        static constexpr const char* torqueFeedback = "JointTorque";
        static constexpr const char* positionFeedback = "JointPosition";
    };


    template <class T> class jointCtrl {
    public:
        jointCtrl(){

        }

        ~jointCtrl(){

        }

        RTT::InputPort<T> orocos_port;
        RTT::FlowStatus joint_cmd_fs;
        T joint_cmd;
    };


    template <class T> class jointFeedback {
    public:
        jointFeedback() {
        }

        ~jointFeedback() {

        }

        T joint_feedback;
        RTT::OutputPort<T> orocos_port;

    };
}
#endif
