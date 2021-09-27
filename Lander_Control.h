#ifndef _LANDER_CONTROL_H
#define _LANDER_CONTROL_H

// Simulation parameters - YOU MUST NOT CHANGE ANY OF THESE
#define G_ACCEL 7.8 //8.87
#define MT_ACCEL 35.0
#define RT_ACCEL 25.0
#define LT_ACCEL 25.0
#define MAX_ROT_RATE .075
#define SONAR_RANGE 9.0
#define NP1 .05
#define NP2 .05
#define T_STEP .005
#define S_SCALE 5.0
#define PI 3.14159265359
#define DISPLAY_LATENCY 10
#define HIST 180

// Global variables accessible to your flight computer
extern int MT_OK;
extern int RT_OK;
extern int LT_OK;
extern double PLAT_X;
extern double PLAT_Y;
extern double SONAR_DIST[36];

// Flight controls
void Main_Thruster(double power);
void Left_Thruster(double power);
void Right_Thruster(double power);
void Rotate(double angle);
double Velocity_X(void);
double Velocity_Y(void);
double Position_X(void);
double Position_Y(void);
double Angle(void);
double RangeDist(void);

// Function prototypes for code you need to look at
void Lander_Control(void);
void Safety_Override(void);

struct thruster_struct {
  double angle_of_thruster_selected; // angle from the main thruster
  double max_thrust_acceleration;
  void (*set_thrust)(double);
}; 

struct pid_context_struct {
  // weights for p.i.d TODO: needs fine-tuning
  double k_p; 
  double k_i; 
  double k_d;
  double arctan_strech_factor;
  double intergal_sum;
  double current_proportion;
  bool valid_current_proportion;
};

double ANGLE_OFFEST_LIMIT = 80; // how much the thruster can tilt away from the default position


inline double boundAngle(double theta) {
  if (theta >= 360) return theta - 360;
  if (theta < 0) return theta + 360;
  return theta;
}

#endif
