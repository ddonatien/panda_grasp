// File:          kdl_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <unistd.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace KDL;



double SimplifyAngle(double angle)
{
    angle = std::fmod(angle, (2.0 * M_PI));
    if( angle < -M_PI )
        angle += (2.0 * M_PI);
    else if( angle > M_PI )
        angle -= (2.0 * M_PI);
    return angle;
}

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  std::string arm_joints[] = {
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7"
  };

  std::string hand_joints[] = {
    "panda_finger_joint1",
    "panda_finger_joint2",
  };

  
  double min_pos[] = {
    -2.8973,
    -1.7628,
    -2.8973,
    -3.0718,
    -2.8973,
    -0.0175,
    -2.8973
  };
   
  double max_pos[] = {
    2.8973,
    1.7628,
    2.8973,
    -0.0698,
    2.8973,
    3.7525,
    2.8973
  };

  
  // Simple robot arm with two segments.
  Chain chain;
  // join1
  chain.addSegment(Segment(Joint(Joint::RotY),
    Frame(Rotation::RPY(0.0,0.0,0.0), Vector(0.0, 0.0, 0.0))));
  // joint2
  chain.addSegment(Segment(Joint(Joint::RotZ),
    Frame(Rotation::RPY(0.0,0.0,0.0), Vector(0.0, 0.333,0.0))));
  // joint3
  chain.addSegment(Segment(Joint(Joint::RotZ),
    Frame(Rotation::RPY(-M_PI/2,0.0,0.0), Vector(0.0, 0.0,0.0))));
  // joint4
  chain.addSegment(Segment(Joint(Joint::RotZ),
    Frame(Rotation::RPY(M_PI/2,0.0,0.0), Vector(0.0825, 0.0, 0.316))));
  // joint5
  chain.addSegment(Segment(Joint(Joint::RotZ),
    Frame(Rotation::RPY(M_PI/2,0.0,0.0), Vector(0.0, 0.0, 0.0))));
  // joint6
  chain.addSegment(Segment(Joint(Joint::RotZ),
    Frame(Rotation::RPY(-M_PI/2,0.0,0.0), Vector(-0.0825, 0.0, -0.384))));
  // joint7
  chain.addSegment(Segment(Joint(Joint::RotZ),
    Frame(Rotation::RPY(M_PI/2,0.0,0.0), Vector(0.088, 0.0, 0.0))));
 
  // Create solver based on kinematic chain
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
  JntArray q_min(chain.getNrOfJoints());
  JntArray q_max(chain.getNrOfJoints());

  for(unsigned i = 0; i < chain.getNrOfJoints(); i++) {
    q_min(i) = min_pos[i];
    q_max(i) = max_pos[i];
  }
 
  std::vector<Motor*> arm_motors;
  for (std::string name: arm_joints) {
    arm_motors.push_back(robot->getMotor(name));
  }

  std::vector<Motor*> hand_motors;
  for (std::string name: hand_joints) {
    hand_motors.push_back(robot->getMotor(name));
  }
  
  // Create the frame that will contain the results
  KDL::Frame cartpos;
  
  KDL::JntArray jointpositions = JntArray(chain.getNrOfJoints());
  
  std::cout<<"Aaaah"<<std::endl;  
  std::vector<PositionSensor*> arm_sensors;
  for (std::string name: hand_joints) {
    arm_sensors.push_back(robot->getPositionSensor(name + "_sensor"));
    robot->getPositionSensor(name + "_sensor")->enable(1000);
  }
  robot->step(timeStep);
  std::cout<<"Aaaah"<<std::endl;
  usleep(1000);
  std::cout<<"bbbb"<<std::endl;
  for(unsigned i = 0; i < chain.getNrOfJoints(); i++) {
    std::cout<<arm_sensors[i]->getValue()<<std::endl;
    jointpositions(i) = arm_sensors[i]->getValue();
  }
  std::cout<<"Aaaah"<<std::endl;
 
  // Calculate forward position kinematics
  bool kinematics_status = fksolver.JntToCart(jointpositions, cartpos);
  std::cout << kinematics_status <<std::endl;
  std::cout << cartpos <<std::endl;
 
  // Vector tmp(0.0, 0.0, 0.0);
  // Vector tr = cartpos * tmp;
  // std::cout << tr << std::endl;
  ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
  ChainIkSolverPos_NR_JL iksolver1(chain, q_min, q_max, fksolver, iksolver1v, 100, 1e-6);
 
  //Creation of jntarrays:
  JntArray q(chain.getNrOfJoints());
  JntArray q_init(chain.getNrOfJoints());
 
  //Set destination frame
  Frame F_dest = Frame(Vector(0.1, 0.9, 0.1));
  int ret = iksolver1.CartToJnt(q_init, F_dest, q);
  std::cout<< ret <<std::endl;


  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1 && ret > -5 ) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    for(unsigned i = 0; i < chain.getNrOfJoints(); i++) {
      arm_motors[i]->setPosition(::SimplifyAngle(q(i)));
    }

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

