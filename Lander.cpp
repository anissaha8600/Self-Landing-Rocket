/*
	Lander Control simulation.

	Updated by F. Estrada for CSC C85, Oct. 2013
	Updated by Per Parker, Sep. 2015

	Learning goals:

	- To explore the implementation of control software
	  that is robust to malfunctions/failures.

	The exercise:

	- The program loads a terrain map from a .ppm file.
	  the map shows a red platform which is the location
	  a landing module should arrive at.
	- The control software has to navigate the lander
	  to this location and deposit the lander on the
	  ground considering:

	  * Maximum vertical speed should be less than 10 m/s at touchdown
	  * Maximum landing angle should be less than 15 degrees w.r.t vertical

	- Of course, touching any part of the terrain except
	  for the landing platform will result in destruction
	  of the lander

	This has been made into many videogames. The oldest one
	I know of being a C64 game called 1985 The Day After.
        There are older ones! (for bonus credit, find the oldest
        one and send me a description/picture plus info about the
        platform it ran on!)

	Your task:

	- These are the 'sensors' you have available to control
          the lander.

	  Velocity_X();  - Gives you the lander's horizontal velocity
	  Velocity_Y();	 - Gives you the lander's vertical velocity
	  Position_X();  - Gives you the lander's horizontal position (0 to 1024)
	  Position Y();  - Gives you the lander's vertical position (0 to 1024)

          Angle();	 - Gives the lander's angle w.r.t. vertical in DEGREES (upside-down = 180 degrees)

	  SONAR_DIST[];  - Array with distances obtained by sonar. Index corresponds
                           to angle w.r.t. vertical direction measured clockwise, so that
                           SONAR_DIST[0] is distance at 0 degrees (pointing upward)
                           SONAR_DIST[1] is distance at 10 degrees from vertical
                           SONAR_DIST[2] is distance at 20 degrees from vertical
                           .
                           .
                           .
                           SONAR_DIST[35] is distance at 350 degrees from vertical

                           if distance is '-1' there is no valid reading. Note that updating
                           the sonar readings takes time! Readings remain constant between
                           sonar updates.

          RangeDist();   - Uses a laser range-finder to accurately measure the distance to ground
                           in the direction of the lander's main thruster.
                           The laser range finder never fails (probably was designed and
                           built by PacoNetics Inc.)

          Note: All sensors are NOISY. This makes your life more interesting.

	- Variables accessible to your 'in flight' computer

	  MT_OK		- Boolean, if 1 indicates the main thruster is working properly
	  RT_OK		- Boolean, if 1 indicates the right thruster is working properly
	  LT_OK		- Boolean, if 1 indicates thr left thruster is working properly
          PLAT_X	- X position of the landing platform
          PLAY_Y        - Y position of the landing platform

	- Control of the lander is via the following functions
          (which are noisy!)

	  Main_Thruster(double power);   - Sets main thurster power in [0 1], 0 is off
	  Left_Thruster(double power);	 - Sets left thruster power in [0 1]
	  Right_Thruster(double power);  - Sets right thruster power in [0 1]
	  Rotate(double angle);	 	 - Rotates module 'angle' degrees clockwise
					   (ccw if angle is negative) from current
                                           orientation (i.e. rotation is not w.r.t.
                                           a fixed reference direction).

 					   Note that rotation takes time!


	- Important constants

	  G_ACCEL = 8.87	- Gravitational acceleration on Venus
	  MT_ACCEL = 35.0	- Max acceleration provided by the main thruster
	  RT_ACCEL = 25.0	- Max acceleration provided by right thruster
	  LT_ACCEL = 25.0	- Max acceleration provided by left thruster
          MAX_ROT_RATE = .075    - Maximum rate of rotation (in radians) per unit time

	- Functions you need to analyze and possibly change

	  * The Lander_Control(); function, which determines where the lander should
	    go next and calls control functions
          * The Safety_Override(); function, which determines whether the lander is
            in danger of crashing, and calls control functions to prevent this.

	- You *can* add your own helper functions (e.g. write a robust thruster
	  handler, or your own robust sensor functions - of course, these must
	  use the noisy and possibly faulty ones!).

	- The rest is a black box... life sometimes is like that.

        - Program usage: The program is designed to simulate different failure
                         scenarios. Mode '1' allows for failures in the
                         controls. Mode '2' allows for failures of both
                         controls and sensors. There is also a 'custom' mode
                         that allows you to test your code against specific
                         component failures.

			 Initial lander position, orientation, and velocity are
                         randomized.

	  * The code I am providing will land the module assuming nothing goes wrong
          with the sensors and/or controls, both for the 'easy.ppm' and 'hard.ppm'
          maps.

	  * Failure modes: 0 - Nothing ever fails, life is simple
			   1 - Controls can fail, sensors are always reliable
			   2 - Both controls and sensors can fail (and do!)
			   3 - Selectable failure mode, remaining arguments determine
                               failing component(s):
                               1 - Main thruster
                               2 - Left Thruster
                               3 - Right Thruster
                               4 - Horizontal velocity sensor
                               5 - Vertical velocity sensor
                               6 - Horizontal position sensor
                               7 - Vertical position sensor
                               8 - Angle sensor
                               9 - Sonar

        e.g.

             Lander_Control easy.ppm 3 1 5 8

             Launches the program on the 'easy.ppm' map, and disables the main thruster,
             vertical velocity sensor, and angle sensor.

		* Note - while running. Pressing 'q' on the keyboard terminates the 
			program.

        * Be sure to complete the attached REPORT.TXT and submit the report as well as
          your code by email. Subject should be 'C85 Safe Landings, name_of_your_team'

	Have fun! try not to crash too many landers, they are expensive!

  	Credits: Lander image and rocky texture provided by NASA
		 Per Parker spent some time making sure you will have fun! thanks Per!
*/

/*
  Standard C libraries
*/
#include <math.h>
#include <stdio.h>
#include <iostream>

#include "Lander_Control.h"

struct thruster_struct thruster;
bool ARE_PIDS_INITIALIZED = false;
struct pid_context_struct x_pid;
struct pid_context_struct y_pid;

void reset_pid(struct pid_context_struct* pid) {
  pid->current_proportion = 0;
  pid->valid_current_proportion = false;
  pid->intergal_sum = 0;
}

void initialize_pid(struct pid_context_struct* pid, double k_p, double k_i, double k_d, double arctan_strech_factor) {
  pid->k_d = k_d;
  pid->k_i = k_i;
  pid->k_p = k_p;
  pid->arctan_strech_factor = arctan_strech_factor;
  reset_pid(pid);
}

double x_position_pid(double x_target) {
  // TO-CHECK: we didnt add the time factor for intergal and derivative calculation (b/c its constant)
  // we might need to add this in, in case something gets wonky

  // proportional component
  double proportion_error = PLAT_X - Position_X();
  // intergral component
  double integral_error = x_pid.intergal_sum + (proportion_error * T_STEP);
  x_pid.intergal_sum = integral_error;
  // derivative component
  double derivative_error;
  if (x_pid.valid_current_proportion) {
    derivative_error = (proportion_error - x_pid.current_proportion) / T_STEP;
  } else {
    derivative_error = 0;
  }
  x_pid.current_proportion = proportion_error;
  x_pid.valid_current_proportion = true;
  std::cout << derivative_error << std::endl;

  return (ANGLE_OFFEST_LIMIT / (PI/2))  * 
    atan(1/x_pid.arctan_strech_factor * ((x_pid.k_p * proportion_error) + 
                                         (x_pid.k_i * integral_error) + 
                                         (x_pid.k_d * derivative_error)));
}



void select_thruster(void) {
  if (MT_OK) {
    thruster.max_thrust_acceleration = MT_ACCEL; //select main thruster
    thruster.set_thrust = &Main_Thruster;
    thruster.angle_of_thruster_selected = 0;
  } else if (LT_OK) {
    thruster.max_thrust_acceleration = LT_ACCEL; //select left thruster
    thruster.set_thrust = &Left_Thruster;
    thruster.angle_of_thruster_selected = 270;
  } else {
    thruster.max_thrust_acceleration = RT_ACCEL; //select right thruster
    thruster.set_thrust = &Right_Thruster;
    thruster.angle_of_thruster_selected = 90;
  }  
}

void Mono_thruster(void) {
  // choose thruster thats working and bring that perpendicular to horizontal
  // function that initailizes all vars corresponding to thursters - stuff like default angle and function for choosing thruster
  select_thruster(); 

    // function that brings rocket to some angle w.r.t to working thruster
  // come to zero velocity and on the x-dir and y-dir

  // phase 1

  double angle_offest = x_position_pid(PLAT_X);
  // std::cout << angle_offest << std::endl;
  // TODO: put in its own function - angle computation + correction
  double angle_target = boundAngle(angle_offest + thruster.angle_of_thruster_selected);
  double angle_correction = boundAngle(Angle() - angle_target);
  double thrust_angle = boundAngle(thruster.angle_of_thruster_selected - Angle());

  thruster.set_thrust((G_ACCEL / thruster.max_thrust_acceleration));

  if (angle_correction > 1 && angle_correction < 359) {
    if (angle_correction>=180) Rotate(360-angle_correction);
    else Rotate(-angle_correction); 
    return;
  }

    // determines engine thrust required to maintain ships desired y velocity
  double THRUST_BOOST_UP = 0.2;
  double THRUST_BOOST_DOWN = 0.2;
  // if (Velocity_Y() > 0.1) {
  //   thruster.set_thrust((G_ACCEL / (thruster.max_thrust_acceleration * cos(thrust_angle*PI/180.0))) - THRUST_BOOST_DOWN);
  // } 
  // else if (Velocity_Y() < -0.1) {
  //   thruster.set_thrust((G_ACCEL / (thruster.max_thrust_acceleration * cos(thrust_angle*PI/180.0))) + THRUST_BOOST_UP);
  // } 
  // else {
  //   thruster.set_thrust((G_ACCEL / (thruster.max_thrust_acceleration * cos(thrust_angle*PI/180.0))));
  // }


  // function to control x-velocity
  // function to control y-velocity
  
}
void Lander_Control(void)
{
 /*
   This is the main control function for the lander. It attempts
   to bring the ship to the location of the landing platform
   keeping landing parameters within the acceptable limits.

   How it works:

   - First, if the lander is rotated away from zero-degree angle,
     rotate lander back onto zero degrees.
   - Determine the horizontal distance between the lander and
     the platform, fire horizontal thrusters appropriately
     to change the horizontal velocity so as to decrease this
     distance
   - Determine the vertical distance to landing platform, and
     allow the lander to descend while keeping the vertical
     speed within acceptable bounds. Make sure that the lander
     will not hit the ground before it is over the platform!

   As noted above, this function assumes everything is working
   fine.
*/

/*************************************************
 TO DO: Modify this function so that the ship safely
        reaches the platform even if components and
        sensors fail!

        Note that sensors are noisy, even when
        working properly.

        Finally, YOU SHOULD provide your own
        functions to provide sensor readings,
        these functions should work even when the
        sensors are faulty.

        For example: Write a function Velocity_X_robust()
        which returns the module's horizontal velocity.
        It should determine whether the velocity
        sensor readings are accurate, and if not,
        use some alternate method to determine the
        horizontal velocity of the lander.

        NOTE: Your robust sensor functions can only
        use the available sensor functions and control
        functions!
	DO NOT WRITE SENSOR FUNCTIONS THAT DIRECTLY
        ACCESS THE SIMULATION STATE. That's cheating,
        I'll give you zero.
**************************************************/
  if (!ARE_PIDS_INITIALIZED) {
    initialize_pid(&x_pid, 1, 0, 2.5, 100); // TO CHECK: fine tune weights for x_pid here
    initialize_pid(&y_pid, 1, 1, 1, 10000); // TO CHECK: fine tune weights for y_pid here
    ARE_PIDS_INITIALIZED = true;
  }
  Mono_thruster();
  return;
 double VXlim;
 double VYlim;

 // Set velocity limits depending on distance to platform.
 // If the module is far from the platform allow it to
 // move faster, decrease speed limits as the module
 // approaches landing. You may need to be more conservative
 // with velocity limits when things fail.
 if (fabs(Position_X()-PLAT_X)>200) VXlim=25;
 else if (fabs(Position_X()-PLAT_X)>100) VXlim=15;
 else VXlim=5;

 if (PLAT_Y-Position_Y()>200) VYlim=-20;
 else if (PLAT_Y-Position_Y()>100) VYlim=-10;  // These are negative because they
 else VYlim=-4;				       // limit descent velocity

 // Ensure we will be OVER the platform when we land
 if (fabs(PLAT_X-Position_X())/fabs(Velocity_X())>1.25*fabs(PLAT_Y-Position_Y())/fabs(Velocity_Y())) VYlim=0;

 // IMPORTANT NOTE: The code below assumes all components working
 // properly. IT MAY OR MAY NOT BE USEFUL TO YOU when components
 // fail. More likely, you will need a set of case-based code
 // chunks, each of which works under particular failure conditions.

 // Check for rotation away from zero degrees - Rotate first,
 // use thrusters only when not rotating to avoid adding
 // velocity components along the rotation directions
 // Note that only the latest Rotate() command has any
 // effect, i.e. the rotation angle does not accumulate
 // for successive calls.

 if (Angle()>1&&Angle()<359)
 {
  if (Angle()>=180) Rotate(360-Angle());
  else Rotate(-Angle());
  return;
 }

 // Module is oriented properly, check for horizontal position
 // and set thrusters appropriately.
 if (Position_X()>PLAT_X)
 {
  // Lander is to the LEFT of the landing platform, use Right thrusters to move
  // lander to the left.
  Left_Thruster(0);	// Make sure we're not fighting ourselves here!
  if (Velocity_X()>(-VXlim)) Right_Thruster((VXlim+fmin(0,Velocity_X()))/VXlim);
  else
  {
   // Exceeded velocity limit, brake
   Right_Thruster(0);
   Left_Thruster(fabs(VXlim-Velocity_X()));
  }
 }
 else
 {
  // Lander is to the RIGHT of the landing platform, opposite from above
  Right_Thruster(0);
  if (Velocity_X()<VXlim) Left_Thruster((VXlim-fmax(0,Velocity_X()))/VXlim);
  else
  {
   Left_Thruster(0);
   Right_Thruster(fabs(VXlim-Velocity_X()));
  }
 }

 // Vertical adjustments. Basically, keep the module below the limit for
 // vertical velocity and allow for continuous descent. We trust
 // Safety_Override() to save us from crashing with the ground.
 if (Velocity_Y()<VYlim) Main_Thruster(1.0);
 else Main_Thruster(0);
}

void Safety_Override(void)
{
//  /*
//    This function is intended to keep the lander from
//    crashing. It checks the sonar distance array,
//    if the distance to nearby solid surfaces and
//    uses thrusters to maintain a safe distance from
//    the ground unless the ground happens to be the
//    landing platform.

//    Additionally, it enforces a maximum speed limit
//    which when breached triggers an emergency brake
//    operation.
//  */

// /**************************************************
//  TO DO: Modify this function so that it can do its
//         work even if components or sensors
//         fail
// **************************************************/

// /**************************************************
//   How this works:
//   Check the sonar readings, for each sonar
//   reading that is below a minimum safety threshold
//   AND in the general direction of motion AND
//   not corresponding to the landing platform,
//   carry out speed corrections using the thrusters
// **************************************************/

//  double DistLimit;
//  double Vmag;
//  double dmin;

//  // Establish distance threshold based on lander
//  // speed (we need more time to rectify direction
//  // at high speed)
//  Vmag=Velocity_X()*Velocity_X();
//  Vmag+=Velocity_Y()*Velocity_Y();

//  DistLimit=fmax(75,Vmag);

//  // If we're close to the landing platform, disable
//  // safety override (close to the landing platform
//  // the Control_Policy() should be trusted to
//  // safely land the craft)
//  if (fabs(PLAT_X-Position_X())<150&&fabs(PLAT_Y-Position_Y())<150) return;

//  // Determine the closest surfaces in the direction
//  // of motion. This is done by checking the sonar
//  // array in the quadrant corresponding to the
//  // ship's motion direction to find the entry
//  // with the smallest registered distance

//  // Horizontal direction.
//  dmin=1000000;
//  if (Velocity_X()>0)
//  {
//   for (int i=5;i<14;i++)
//    if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
//  }
//  else
//  {
//   for (int i=22;i<32;i++)
//    if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
//  }
//  // Determine whether we're too close for comfort. There is a reason
//  // to have this distance limit modulated by horizontal speed...
//  // what is it?
//  if (dmin<DistLimit*fmax(.25,fmin(fabs(Velocity_X())/5.0,1)))
//  { // Too close to a surface in the horizontal direction
//   if (Angle()>1&&Angle()<359)
//   {
//    if (Angle()>=180) Rotate(360-Angle());
//    else Rotate(-Angle());
//    return;
//   }

//   if (Velocity_X()>0){
//    Right_Thruster(1.0);
//    Left_Thruster(0.0);
//   }
//   else
//   {
//    Left_Thruster(1.0);
//    Right_Thruster(0.0);
//   }
//  }

//  // Vertical direction
//  dmin=1000000;
//  if (Velocity_Y()>5)      // Mind this! there is a reason for it...
//  {
//   for (int i=0; i<5; i++)
//    if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
//   for (int i=32; i<36; i++)
//    if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
//  }
//  else
//  {
//   for (int i=14; i<22; i++)
//    if (SONAR_DIST[i]>-1&&SONAR_DIST[i]<dmin) dmin=SONAR_DIST[i];
//  }
//  if (dmin<DistLimit)   // Too close to a surface in the horizontal direction
//  {
//   if (Angle()>1||Angle()>359)
//   {
//    if (Angle()>=180) Rotate(360-Angle());
//    else Rotate(-Angle());
//    return;
//   }
//   if (Velocity_Y()>2.0){
//    Main_Thruster(0.0);
//   }
//   else
//   {
//    Main_Thruster(1.0);
//   }
//  }
}
