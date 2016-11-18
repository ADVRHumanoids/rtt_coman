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
