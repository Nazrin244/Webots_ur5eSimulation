// File:          myUR5.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 64
// All the webots classes are defined in the "webots" namespace
using namespace webots;

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
  Motor *claw1 = robot->getMotor("finger_1_joint_1");
  Motor *claw2 = robot->getMotor("finger_2_joint_1");
  Motor *claw3 = robot->getMotor("finger_middle_joint_1");
  Motor *shoulder = robot->getMotor("shoulder_lift_joint");
  Motor *elbow = robot->getMotor("elbow_joint");
  Motor *wrist1 = robot->getMotor("wrist_1_joint");
  Motor *wrist2 = robot->getMotor("wrist_2_joint");
  
  claw1->setPosition(INFINITY);
  claw2->setPosition(INFINITY);
  claw3->setPosition(INFINITY);
  shoulder->setPosition(INFINITY);
  elbow->setPosition(INFINITY);
  wrist1->setPosition(INFINITY);
  wrist2->setPosition(INFINITY);
  
  claw1->setVelocity(0.0);
  claw2->setVelocity(0.0);
  claw3->setVelocity(0.0);
  shoulder->setVelocity(0.0);
  elbow->setVelocity(0.0);
  wrist1->setVelocity(0.0);
  wrist2->setVelocity(0.0);
  
  // get the time step of the current world.
 // int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
   DistanceSensor *ds = robot->getDistanceSensor("distance sensor");
   ds->enable(TIME_STEP);
   int count = 0;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    count++;
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    double val = ds->getValue();
    std::cout<< "distance: " <<val<<" count: "<<count<<std::endl;

    // Process sensor data here.
    if(val<300){
     claw1->setVelocity(6.0);//close claw set velocity to 6
     claw2->setVelocity(6.0);
     claw3->setVelocity(6.0);
     }
          
     else if(count>30 && count>35){
     shoulder->setVelocity(-2.0);//set shoulder velocity to -2
    } 
    else if(count>40 && count>45){
      elbow->setVelocity(-2.0); //set elbow velocity to -2
      }
      else if(count>50 && count>60){
      wrist1->setVelocity(-2.0);//set wrist velocity to -2
      }
      else if (val<529){
     claw1->setVelocity(0.0);//open claw
     claw2->setVelocity(0.0);
     claw3->setVelocity(0.0);
     }
   else{
  claw1->setVelocity(0.0);
  claw2->setVelocity(0.0);
  claw3->setVelocity(0.0);
  
  shoulder->setVelocity(0.0);
  elbow->setVelocity(0.0);
  wrist1->setVelocity(0.0);
  wrist2->setVelocity(0.0);
     }
  };
  



  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
