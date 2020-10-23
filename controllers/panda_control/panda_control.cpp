#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  Motor *joints[7];
  char joint_names[7][7] = {"joint1", "joint2", "joint3", "joint4",
                            "joint5", "joint6", "joint7"};
  for (int i = 0; i < 4; i++) {
    joints[i] = robot->getMotor(joint_names[i]);
    joints[i]->setPosition(INFINITY);
    joints[i]->setVelocity(0.0);
  }
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();


  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    //joints[0]->setPosition(0.);    
    //joints[1]->setPosition(0.);
    //joints[2]->setPosition(0.);
    //joints[3]->setPosition(0.);
    //joints[4]->setPosition(0.);
    //joints[5]->setPosition(0.);
    //joints[6]->setPosition(0.);
    //joints[7]->setPosition(0.);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
