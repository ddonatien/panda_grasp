
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames_io.hpp>
#include <unistd.h>
#include <queue>

using namespace webots;
using namespace KDL;


class PandaArm {
  public: PandaArm(Robot* robot, int timeStep, Frame baseFrame) {
    this->baseFrame = baseFrame;
    // join1
    this->chain.addSegment(Segment(Joint(Joint::RotZ),
      Frame::DH_Craig1989(0., 0.0, 0.333, 0.0)));
    // joint2
    this->chain.addSegment(Segment(Joint(Joint::RotZ),
      Frame::DH_Craig1989(0.0, -M_PI/2, 0.0, 0.0)));
    // joint3
    this->chain.addSegment(Segment(Joint(Joint::RotZ),
      Frame::DH_Craig1989(0.0, M_PI/2, 0.316, 0.0)));
    // joint4
    this->chain.addSegment(Segment(Joint(Joint::RotZ),
      Frame::DH_Craig1989(-0.0825, M_PI/2, 0.0, 0.0)));
    // joint5
    this->chain.addSegment(Segment(Joint(Joint::RotZ),
      Frame::DH_Craig1989(0.0825, -M_PI/2, 0.384, 0.0)));
    // joint6
    this->chain.addSegment(Segment(Joint(Joint::RotZ),
      Frame::DH_Craig1989(0.0, M_PI/2, 0.0, 0.0)));
    // joint7
    this->chain.addSegment(Segment(Joint(Joint::RotZ),
      Frame::DH_Craig1989(-0.088, M_PI/2, 0.0, 0.0)));
    // eef
    this->chain.addSegment(Segment(Joint(Joint::None),
      Frame::DH_Craig1989(0.0, 0.0, 0.207, 0.0)));
      
    this->fksolver = new ChainFkSolverPos_recursive(chain);
    
    for(unsigned i = 0; i < chain.getNrOfJoints(); i++) {
      this->q_min(i) = min_pos[i];
      this->q_max(i) = max_pos[i];
      this->q_start(i) = init_pos[i];
    }
 
    for (std::string name: arm_joints) {
      this->arm_motors.push_back(robot->getMotor(name));
    }

    for (std::string name: hand_joints) {
      this->hand_motors.push_back(robot->getMotor(name));
    }
    
    for (std::string name: arm_joints) {
      this->arm_sensors.push_back(robot->getPositionSensor(name + "_sensor"));
      this->arm_sensors.back()->enable(timeStep);
    }

    this->iksolver1v = new ChainIkSolverVel_pinv(this->chain);
    this->iksolver1 = new ChainIkSolverPos_LMA(this->chain, 1E-3, 500, 1E-4);

    SetQGoal(this->q_start);
  }

  public: JntArray GetJointPositions() {
    JntArray pos(7);
    pos(0) = 0.0;
    for(unsigned i = 0; i < arm_sensors.size()-1; i++) {
      pos(i+1) = arm_sensors[i]->getValue();
    }
    return pos;
  }

  public: void OpenJaws(/*float force*/) {
    for(Motor* motor: hand_motors){
      std::cout<<motor->getMaxPosition()<<std::endl;
      std::cout<<motor->getMinPosition()<<std::endl;
      // motor->setAvailableForce(force);
      motor->setPosition(0.0);
    }
    return;
  }

  public: void CloseJaws(/*float force*/) {
    for(Motor* motor: hand_motors){
      std::cout<<motor->getMaxPosition()<<std::endl;
      std::cout<<motor->getMinPosition()<<std::endl;
      // motor->setAvailableForce(force);
      motor->setPosition(-0.04);
    }
    return;
  }

  public: bool GetPose(Frame* cartpos,int segmentNr=-1){
    KDL::JntArray jp = GetJointPositions();
    bool ok = fksolver->JntToCart(jp, *cartpos, segmentNr);
    std::cout<<"Cartpos"<<std::endl;
    std::cout<<*cartpos<<std::endl;
    *cartpos = this->baseFrame.Inverse()*(*cartpos);
    return ok;
  }

  public: int GetDest(Frame FDest, JntArray q_start, JntArray* tmp) {
    int code = iksolver1->CartToJnt(q_start, baseFrame*FDest, *tmp);
    return code;
  }

  public: JntArray GetStartPos() {
    return q_start;
  }

  public: bool IsMoving() {
    return moving;
  }

  public: void Update(){
    JntArray jp = GetJointPositions();
    if(!Equal(jp, qGoal, 1e-1)){
        moving = true;
    }
    else {
      moving = false;
      std::pair<int, Frame*> goal = goals.front();
      switch (goal.first)
      {
        case 0:
          {
            CloseJaws();
            goals.pop();
          }
          break;

        case 1:
          {
            OpenJaws();
            goals.pop();
          }
          break;

        case 2:
          {
            JntArray tmp(7);
            KDL::JntArray q_init(this->chain.getNrOfJoints());
            for(unsigned int i=0; i < chain.getNrOfJoints(); i++ ) {
              q_init(i) = (q_max(i)-q_min(i))/2;
            }
            int code = GetDest(*(goal.second), jp, &tmp);
            std::cout<<"Invese kinematics code : "<<code<<std::endl;
            std::cout<<tmp(0)<<" "<<tmp(1)<<" "<<tmp(2)<<" "<<tmp(3)<<" "<<tmp(4)<<" "<<tmp(5)<<" "<<tmp(6)<<" "<<tmp(6)<<" "<<std::endl;
            SetQGoal(tmp);
            goals.pop();
          }
          break;
  
        default:
          break;
      }
    }
  }

  private: void SetQGoal(JntArray q) {
    qGoal = JntArray(7);
    qGoal(0) = 0.0;
    for(unsigned i = 0; i < 7; i++) {
      arm_motors[i]->setPosition(q(i));
      if (i<6) qGoal(i+1) = q(i);
    }
    moving = true;
  }

  public: void AddGoal(int nature, Frame* dest) {
    goals.push(std::pair<int, Frame*>(nature, dest));
  }

  private:
    bool moving = false;
    ChainFkSolverPos_recursive *fksolver;
    ChainIkSolverVel_pinv *iksolver1v;
    ChainIkSolverPos_LMA *iksolver1;
    KDL::Frame baseFrame;
    std::vector<Motor*> arm_motors;
    std::vector<Motor*> hand_motors;
    std::vector<PositionSensor*> arm_sensors;
    std::queue<std::pair<int, Frame*>> goals;
    Chain chain; 
    JntArray qGoal = *(new JntArray(7));
    JntArray q_min = *(new JntArray(7));
    JntArray q_max = *(new JntArray(7));
    JntArray q_start = *(new JntArray(7));
    std::string arm_joints[7] = {
      "panda_joint1",
      "panda_joint2",
      "panda_joint3",
      "panda_joint4",
      "panda_joint5",
      "panda_joint6",
      "panda_joint7"
    };
    std::string hand_joints[2] = {
      "panda_finger_joint1",
      "panda_finger_joint2"
    };
    double min_pos[7] = {
      -2.8973,
      -1.7628,
      -2.8973,
      0.0698,
      -2.8973,
      -3.7525,
      -2.8973
    };
    double max_pos[7] = {
      2.8973,
      1.7628,
      2.8973,
      3.0718,
      2.8973,
      0.0175,
      2.8973
    };
    double init_pos[7] = {
      -1.19,
      -0.07,
      -0.75,
      1.63,
      -1.16,
      -2.1,
      2
    };
    double SimplifyAngle(double angle) {
        angle = std::fmod(angle, (2.0 * M_PI));
        if( angle < -M_PI )
            angle += (2.0 * M_PI);
        else if( angle > M_PI )
            angle -= (2.0 * M_PI);
        return angle;
    };
};

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  KDL::Frame cartpos;
  robot->step(timeStep);
  Frame base1 =Frame(Rotation::RPY(0.0, M_PI, 0.0)*Rotation::RPY(-M_PI/2, 0.0, 0.0));
  Frame base2 = Frame(Rotation::Quaternion(0.270598, 0.270597, 0.653281, 0.653282), Vector(-0.1, 0.5, 0.1)).Inverse();
  PandaArm arm = PandaArm(robot, timeStep, base1*base2);
   
  //Frame F_dest = Frame(Rotation(-0.76504, 0.209616, 0.608912, 0.435161, 0.865273, 0.248872, -0.474707, 0.455372, -0.753185)
  //                 ,Vector(0.367334, 0.399661, 0.611311));
  //arm.AddGoal(2, &F_dest);

  int u = 0;
  while (robot->step(timeStep) != -1) {
    arm.Update();
    if(!arm.IsMoving()){
      u++;
      if(!(u % 1000)){
      KDL::Frame effpos8;
      std::cout<<"Frame 8"<<std::endl;
      std::cout<<arm.GetPose(&effpos8, 8)<<std::endl;
      std::cout<<effpos8<<std::endl;
      }
    }
  };

  delete robot;
  return 0;
}