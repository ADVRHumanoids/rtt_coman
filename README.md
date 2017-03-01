rtt_coman
=========

This package implements an OROCOS component for communicating with the COMAN Robot through the Robolli library. 

How it works:
-------------
The ``rtt_coman`` permits to read/write data to the DSPs boards from the OROCOS framework. Robolli is the intermediary between 
OROCOS and the DSPs: it basically open a thread that buffers DSPs written/read data.

How to use it:
-----------
Firs of all turn on COMAN and connect through the ethernet paying attention that the ```eth_iface``` in ```config/config.yaml```
corresponds to the interface that you are using to connect to COMAN. If not change the ```config.yaml``` file (you can
check the interface typing ```ifconfig``` in your terminal and looking at the ethernet card name).

Now you can run one of the OROCOS script, inside the ```orocos_script``` folder to start the ```rtt_coman``` component.
The script is very simple:
- In the first part we import OROCOS components and libraries:

```
import("eigen_typekit")
import("kdl_typekit")
import("rst-rt_typekit")
import("rtt_rsbcomm")
import("rtt_coman")
require("os")
```
- Then we load the rtt_component and we set its activity:

```
loadComponent("coman","cogimon::rtt_coman")

setActivity("coman",0.001,10,ORO_SCHED_OTHER)
```
for example to 1kHz at the priority of 10.
- We need to load the config YAML file and the model files (URDF and SRDF), then we call the ```configure()```. Note that these functions have to be called
in this order, if not the component will not start.

```
var string YAML_path = os.getenv("ROBOTOLOGY_ROOT") + "/robots/rtt_coman/config/config.yaml"
coman.loadYAML(YAML_path)

var string urdf_path = os.getenv("ROBOTOLOGY_ROOT") + "/robots/cogimon-gazebo-models/iit-coman/model.urdf"
var string srdf_path = os.getenv("ROBOTOLOGY_ROOT") + "/robots/cogimon-gazebo-models/iit-coman/coman.srdf"
coman.loadURDFAndSRDF(urdf_path, srdf_path)

coman.configure()
```
- If needed is possible to set joint offsets for the model, this operation has to be done before calling the ```start()```:

```
var double M_PI = 3.14159265359;
coman.setOffSet("LShLat", 90.0*M_PI/180.0)
coman.setOffSet("RShLat", -90.0*M_PI/180.0)
```
- Finally we start the component:

```
coman.start()
```

To stop the component just call the operation ```.stop()```, this will stop the motor control in the control boards.

Available Operations:
---------------------
- ***getAvailableControlMode("chain_name")*** return a vector of strings with the available control modes for that kinematic chain  
- ***getControlMode("chain_name")*** return a string with the actual running control mode for that kinematic chain
- ***getKinematicChains()*** return a vector of strings with the name of the kinematic chain for the robot
- ***getKinematicChainsAndJoints()*** return a map between kinematic chain names and joint names (oredered)
- ***loadYAML("yaml_path")*** load a ```config.yaml``` file
- ***loadURDFAndSRDF("urdf_path, srdf_path")*** load ```robot.urdf``` and ```robot.srdf``` files
- ***printKinematicChainInformation("kinematic_chains")*** prints to terminal some information related to kinematic chain
- ***setPID("kinematic_chains", P, I, D)*** set PID values for position Ctrl for a particular kinematic chain

Input Ports:
------------
The following set of ports is generated for each kinematic chain of the robot
- ***kinematic_chain_JointImpedanceCtrl*** set joints stiffness and damping for the impedance control
- ***kinematic_chain_JointPositionCtrl*** set joints position for position control and impedance control
- ***kinematic_chain_JointPositionCtrl_VoltageOffset*** set joints voltage offset for position control
- ***kinematic_chain_JointTorqueCtrl*** set joints torque for torque control

Output Ports:
------------
The following port is generated for each kinematic chain of the robot
- ***kinematic_chain_JointFeedback*** joints position, velcity and torque feedback

NOTE:
-----
Available controllers are (for now):
- ***Position Ctrl***
- ***Voltage Ctrl*** obtained setting the Position Ctrol mode, setting to 0 all the PIDs and sending values in the ```kinematic_chain_JointPositionCtrl_VoltageOffset``` port
