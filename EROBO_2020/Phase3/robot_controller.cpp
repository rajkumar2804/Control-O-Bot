// File:          final_controller_phase3.cpp
// Date:          9th Nov, 2020
// Description:  Find shortest path in a maze
// Author:     Raj Kumar Gupta
// Modifications: None
// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/GPS.hpp>

// time in [ms] of a simulation step
#define TIME_STEP 32
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv){
  // create the Robot instance. 
    Robot *robot = new Robot();
    
    // Initialize Infra-Red sensors
    DistanceSensor *ir[4];
    char irNames[4][20] = {"ir_right", "ir_centre_left", "ir_centre_right", "ir_left"};
    for (int i = 0; i < 4; i++) {
      ir[i] = robot->getDistanceSensor(irNames[i]);
      ir[i]->enable(TIME_STEP);
  }
    // Initialize Distance Sensors
    DistanceSensor *ds[3];
    char dsNames[3][10] = {"ds_right", "ds_front","ds_left"};
    for (int i = 0; i < 3; i++) {
      ds[i] = robot->getDistanceSensor(dsNames[i]);
      ds[i]->enable(TIME_STEP);
  }
  
  // Initialize GPS module
  GPS *gps;
  gps=robot->getGPS("gps");
  gps->enable(TIME_STEP);
  
  // Initailize Inertial Measurement Unit
  InertialUnit *IMU;
  IMU=robot->getInertialUnit("imu");
  IMU->enable(TIME_STEP);

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  
   // Initialize 4 wheels of the bot
   Motor *wheels[4];
   char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
   for (int i = 0; i < 4; i++) {
     wheels[i] = robot->getMotor(wheels_names[i]);
     wheels[i]->setPosition(INFINITY);
     wheels[i]->setVelocity(0.0);
   }
   
   //********************************************************
   //            VARIABLES DECLARATION
   //******************************************************** 
   
    // Any intersection is called node
    bool node_detected = 0;
    
    // Line follower mode on/off
    bool line_follower_mode = 1;
    
    int reach_middle_of_node_counter = 0;
    bool reached_middle_of_node = 0;
    
    // Availability of paths at node
    bool left_path_available = 0;
    bool right_path_available = 0;
    bool front_path_available = 0;
    
    // Current movement
    bool going_straight = 0;
    bool turning_left = 0;
    bool turning_right = 0;
    
    // Orientation of bot
    float initial_theta = 0;
    float current_theta = 0;
    float diff_theta = 0;
    
    // Destination coordinates
    float x = -2.14396;
    float y = 0.043;
    float z = -1.48741;
    
    // Max velocity of bot
    int vel = 8;
    
    // Velocity of 4 wheels
    double leftspeed_wheel1 = 0;
    double rightspeed_wheel2 = 0;
    double leftspeed_wheel3= 0;
    double rightspeed_wheel4 = 0;
    
    //************************************************************************

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read Infra-Red sensors:
    double irValues[4];
    for (int i = 0; i < 4 ; i++)
      irValues[i] = ir[i]->getValue();
    
    // Read Distance sensors
    double dsValues[3];
    for (int i = 0; i < 3 ; i++)
      dsValues[i] = ds[i]->getValue();
    
    // Get current coordinates of bot from GPS  
    float curr_x = gps->getValues()[0];
    float curr_y = gps->getValues()[1];
    float curr_z = gps->getValues()[2];
  
    // Check if we need to turn right
    bool right_turn =
      irValues[1] <200 &&
      irValues[2]>250;
      
    // Check if we need to turn left      
    bool left_turn =
      irValues[1] >250 &&
      irValues[2] <200;
      
    // Check if we need to move forward    
    bool move_forward=
      irValues[1] >250 &&
      irValues[2] >250;

    // Check if left path is detected
    bool node_left_turn=
      (irValues[0] <450 &&
      irValues[1] >250 &&
      irValues[2] >250 &&
      irValues[3] >500) ||
      (irValues[0] >500 && (irValues[0] < 1000 || irValues[0] == 1000) &&
      irValues[1] >250 && irValues[1] < 1000 &&
      irValues[2] >250 && irValues[2] < 1000 &&
      irValues[3] >500 && (irValues[3] < 1000 ||irValues[0] == 1000));
      
    // Check if left path is detected     
    bool node_right_turn=
      (irValues[0] >500 &&
      irValues[1] >250 &&
      irValues[2] >250 &&
      irValues[3] <450) ||
      (irValues[0] >500 && (irValues[0] < 1000 || irValues[0] == 1000) &&
      irValues[1] >250 && irValues[1] < 1000 &&
      irValues[2] >250 && irValues[2] < 1000 &&
      irValues[3] >500 && (irValues[3] < 1000 ||irValues[0] == 1000));
      
    // If a node is detected     
    if(node_left_turn || node_right_turn){
            node_detected = 1;
            // Turn line follower mode OFF
            line_follower_mode = 0;
    }	 
	  
    if(right_turn){
        //Turn right
        leftspeed_wheel1=2.0;
        rightspeed_wheel2=1;
        leftspeed_wheel3=2.0;
        rightspeed_wheel4=1;
    } else if(left_turn){
        // Turn left
        leftspeed_wheel1=1;
        rightspeed_wheel2=2.0;
        leftspeed_wheel3=1;
        rightspeed_wheel4=2.0;
    } else if(move_forward){
        // Go forward
        leftspeed_wheel1=vel;
        rightspeed_wheel2=vel;
        leftspeed_wheel3=vel;
        rightspeed_wheel4=vel;
    } else {
        //Go forward
        leftspeed_wheel1=vel;
        rightspeed_wheel2=vel;
        leftspeed_wheel3=vel;
        rightspeed_wheel4=vel;
    }
    
    // When at the center of node, stop the bot
    if(node_detected && irValues[0] < 450 && irValues[3] < 450){ 
        wheels[0]->setVelocity(0);
        wheels[1]->setVelocity(0);
        wheels[2]->setVelocity(0);
        wheels[3]->setVelocity(0);
        node_detected = 0;
    } 
    
    
    if(!node_detected && !line_follower_mode && !reached_middle_of_node && !reach_middle_of_node_counter){
        reach_middle_of_node_counter = 10;
    }
    else if(line_follower_mode) {	
        wheels[0]->setVelocity(leftspeed_wheel1);
        wheels[1]->setVelocity(rightspeed_wheel2);
        wheels[2]->setVelocity(leftspeed_wheel3);
        wheels[3]->setVelocity(rightspeed_wheel4);
    }
    
    // Keep moving forward until not at the center of node
    if(reach_middle_of_node_counter > 0){
        reach_middle_of_node_counter--;
        wheels[0]->setVelocity(2);
        wheels[1]->setVelocity(2);
        wheels[2]->setVelocity(2);
        wheels[3]->setVelocity(2);
    }
    
    // Stop when at center of node and check if a path is available in front
    if(!node_detected && !line_follower_mode && !reach_middle_of_node_counter && !reached_middle_of_node){
        reached_middle_of_node = 1;
        wheels[0]->setVelocity(0);
        wheels[1]->setVelocity(0);
        wheels[2]->setVelocity(0);
        wheels[3]->setVelocity(0);

        // If front path is detected at node
        if(irValues[1] > 250 && irValues[2] >250){
            front_path_available = 1;
        }
        // Store the initial orientation of bot before turning
        initial_theta = IMU->getRollPitchYaw()[2];
    }
    
    // If left path is detected at node according to IR senosrs
    if(node_left_turn){
        left_path_available = 1;
    }

    // If right path is detected at node according to IR senosrs  
    if(node_right_turn){
        right_path_available = 1;
    }
    
    // If right path is not detected at node according to distance senosrs
    if(dsValues[0] < 1000){
        right_path_available = 0;
    }
    
    // If front path is not detected at node according to distance senosrs
    if(dsValues[1] < 1000){
        front_path_available = 0;
    }

    // If left path is not detected at node according to distance senosrs
    if(dsValues[2] < 1000){
        left_path_available = 0;
    }
    
     
    // Store the current orientation of bot
    current_theta = IMU->getRollPitchYaw()[2];
    
    // Compute difference in initial and current orientation of the bot
    if(initial_theta >3.12 && initial_theta < 3.14){
        diff_theta = abs(initial_theta - abs(current_theta));
    } else {
        diff_theta = abs(initial_theta - current_theta);
    }
    
     // Take necessary action when at the center of node	
     if(reached_middle_of_node){
         if(front_path_available){
	    wheels[0]->setVelocity(vel);
	    wheels[1]->setVelocity(vel);
	    wheels[2]->setVelocity(vel);
	    wheels[3]->setVelocity(vel);
	    going_straight = 1;	 
         } else if (right_path_available){
               turning_right = 1;
         } else if (left_path_available){
	    turning_left = 1;
         } else {
  	    wheels[0]->setVelocity(0);
	    wheels[1]->setVelocity(0);
	    wheels[2]->setVelocity(0);
	    wheels[3]->setVelocity(0);
         }
      }
      
    // If bot have not yet rotated 90 degree in the left  
    if(turning_left && diff_theta < 1.57){
        wheels[0]->setVelocity(-2);
        wheels[1]->setVelocity(2);
        wheels[2]->setVelocity(-2);
        wheels[3]->setVelocity(2);
    }
    
    // If bot have not yet rotated 90 degree in the left
    if(turning_right && diff_theta < 1.57){
        wheels[0]->setVelocity(2);
        wheels[1]->setVelocity(-2);
        wheels[2]->setVelocity(2);
        wheels[3]->setVelocity(-2);
    }
    
    // If bot have completed 90 degree rotation 
    if(((turning_left || turning_right) && diff_theta >= 1.57) || going_straight){
        wheels[0]->setVelocity(0);
        wheels[1]->setVelocity(0);
        wheels[2]->setVelocity(0);
        wheels[3]->setVelocity(0);
        
        // Restet variables to 0
        going_straight = 0;
        turning_left = 0;
        turning_right = 0;
        reached_middle_of_node = 0;
        front_path_available = 0;
        right_path_available = 0;
        left_path_available = 0;
        
        // Turn line follower mode ON
        line_follower_mode =1;
    } 
    
    // Check if reached the destination and stop the bot and turn line follower mode OFF
    if(curr_x < x+0.01 && curr_x > x-0.01 && curr_y < y+0.03 && curr_y > y-0.03 && curr_z < z+0.04 && curr_z > z-0.04){
        wheels[0]->setVelocity(0);
        wheels[1]->setVelocity(0);
        wheels[2]->setVelocity(0);
        wheels[3]->setVelocity(0);
        line_follower_mode = 0;
        node_detected = 0;
        std::cout<<"Destination Reached"<<std::endl;
    }
  }

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}