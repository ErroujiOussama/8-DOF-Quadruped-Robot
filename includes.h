/***************************************************************
 *  Credits:
 *  Oussama Errouji
 *  Developed and adapted this code during the NRC Competition,
 *  2025 edition.
 *
 *  File Purpose:
 *  Core definitions, constants, data structures, and configuration for the quadruped robot. 
 *  Required by all robot modules for universal access to parameters and structures.
 *
 *  Please cite this contribution if reused or reworked.
 ***************************************************************/
//Serial communication variables
String message = ""; 
boolean message_complete = false;

//Servo management variables and constants
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// *************** SERVO IDS
#define HIP_RIGHT_BACK 0
#define KNEE_RIGHT_BACK 1

#define HIP_LEFT_BACK 2
#define KNEE_LEFT_BACK 3

#define HIP_RIGHT_FRONT 12
#define KNEE_RIGHT_FRONT 13

#define HIP_LEFT_FRONT 14
#define KNEE_LEFT_FRONT 15


// ************** DEFAULT GEOMETRY
#define FEMUR_SIZE 65
#define TIBIA_SIZE 45
#define POSE_WIDTH 10
#define POSE_HEIGHT 10
#define POSE_ORIGIN_Y 100
#define POSE_ORIGIN_X 0


// *************** COMMAND CONSTANTS 
#define LEG_BL 0
#define LEG_FL 1
#define LEG_FR 2
#define LEG_BR 3
#define STOP 4



// ***************** DATA STRUCTURES 
struct point {
    int x;
    int y;
};

struct arc {
  point origin;
  point radius;
  int start_angle;
  int end_angle;
};

struct line {
  point P1;
  point P2;
};

struct angles {
    int hip;
    int knee;
};

struct pose {
  int index;
  struct point width_height;
  struct point origin_offset;
};

struct leg {
  int index;
  struct angles position;
  struct point trajectory[10];
  struct arc flight;
  struct angles hipknee;
  boolean forward;
};

char* LegNames[]={"BACK LEFT", "FRONT LEFT", "FRONT RIGHT","BACK RIGHT"};

// *************** ARRAY OF SERVO IDS - CLOCKWISE DIRECTION STARTING WITH BACK LEFT
int servos[4][2] =
{
  {HIP_LEFT_BACK,KNEE_LEFT_BACK},
  {HIP_LEFT_FRONT,KNEE_LEFT_FRONT},
  {HIP_RIGHT_FRONT,KNEE_RIGHT_FRONT},
  {HIP_RIGHT_BACK,KNEE_RIGHT_BACK}
};

// *************** LEGS IN CLOCKWISE DIRECTION
struct leg legs[4] = {};


// *************** DEFAULT STAND POSE
int STAND[4] = {2,2,2,2};


// **************** GAIT DEFINITIONS
//this array has tree dimensions, the first level is the walk sequence
//this can be lateral, parallel, left turn or right turn
//the next dimension holds the five steps of each sequence
//the last dimension hold the four positions for each leg at the given step
//the order is as allways clockwise starting with the back left leg
int WALKS[4][4][4] =
{
  {
    { 2, 3, 1, 4 },  //4 //4 baglens
    { 3, 4, 2, 1 },  //1 //3
    { 4, 1, 3, 2 },  //2 //2
    { 1, 2, 4, 3 }   //3 //1
  },
  {
    { 2, 3, 1, 4 },  //4 //4 baglens
    { 3, 4, 2, 1 },  //1 //3
    { 4, 1, 3, 2 },  //2 //2
    { 1, 2, 4, 3 }   //3 //1
  },
  /*
  {
     { 2, 1, 3, 4 },
     { 3, 2, 4, 0 },
     { 4, 3, 0, 1 },
     { 0, 4, 1, 2 }
     //{ 1, 0, 2, 3 }
  },*/
  {
     { 2, 3, 4, 1 },
     { 1, 2, 0, 2 },
     { 0, 1, 1, 3 },
     { 4, 0, 2, 4 }
     //{ 3, 4, 3, 0 }
  },
  {
     { 2, 3, 4, 1 },
     { 1, 2, 0, 2 },
     { 0, 1, 1, 3 },
     { 4, 0, 2, 4 }
     //{ 3, 4, 3, 0 }
  }
};

#define LATERAL_WALK 0
#define PARALLEL_WALK 1
#define LEFT_TURN 2
#define RIGHT_TURN 3

