#include <Wire.h>
#include <math.h>
#include <Servo.h>
#include <stdio.h>
#include <LibPrintf.h>

#define SV1_DO      2
#define SV2_DO      3
#define SV3_DO      4
#define SV4_DO      5

#define ST1_DIR     7
#define ST1_STEP    8
#define ST1_ENABLE  9
#define ST2_DIR     10
#define ST2_STEP    11
#define ST2_ENABLE  12
#define ST3_DIR     38
#define ST3_STEP    14
#define ST3_ENABLE  15

#define ST1_LIM1    16
#define ST2_LIM1    17
#define ST3_LIM1    18

#define ST1_LIM2    38
#define ST2_LIM2    39
#define ST3_LIM2    40

#define LIGHT_IN1  A5
#define VAC_SENSOR A6
#define ST1_POS    A7
#define ST2_POS    A8
#define ST3_POS    A9
#define SV1_A      A10
#define SV2_A      A11
#define SV3_A      A12
#define SV4_A      A13

#define LIGHT_IN2  28
#define SOL1       32
#define SOL2       33
#define VALVE1     34
#define VALVE2     35
#define SOL3       36

// A pair of varibles to help parse serial commands (thanks Fergs)

int arg = 0;
int idx = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[8];
char argv2[8];
char argv3[8];
char argv4[8];
 

// The arguments h to integers
long arg1;
long arg2;
long arg3;
long arg4;

unsigned char breg = 0;
unsigned char limitTest;
unsigned char zin;
unsigned char zlast = 0xff;

#define ROT_CCW           'w'
#define ROT_CW            'c'
#define ROT_CCW_DEGREES   'y'
#define ROT_CW_DEGREES    'e'

#define ROT_HALT          'h'
#define ZERO              'z'

#define LOCATION          'l'
#define ROT_TO_POSITION   'r'
#define ARM_LOCK          'a'
#define DEMO              'd'
#define POINT             'p'
#define CALIBRATE         'b'
#define SET_ANGLES        's'
#define GO_XY             'g'
#define BASKET_MOVE       'v'
#define MINIARM_MOVE      'm'
#define MINIARM_CALIB     'n'


// Motor states
#define MOTOR_IDLE  0
#define MOTOR_CCW  1
#define MOTOR_CW 2


typedef struct MOTOR {
    char enGpio;
    char dirGpio;
    char stepGpio;
    char positionAnalogPin;
    char limit1;
    char limit2;
    
    float ticksPerDegree;
    float avPerDegree;
    float ticksPerAv;
   
    char direction;
    int beginZeroMonitoring;
    
    char stopFlag;
    char pulseHigh;
    char speedMode;
    int position;

    int lastPositionDegrees;

    int targetAv;
    int currentAv;
    int lastAv;
    
    int moveTicks;
    int ticksLeft;
    int ticksPerformed;           // how many ticks so far
    int ticksFullSpeedTrigger;    // when to go to full speed
    int ticksSlowDownTrigger;     // when to slow down
       
    int ticksSkip;                // ticks left to skip
    int tickSkipIntervalIndex;    // index into skip interval table
    int skipIntervalChangeIncrement;    // length of tick interval
    int ticksIntervalChangeTrigger;    // trigger to move to next interval
    int skipBuckets;
  } MOTOR;
 
typedef struct MOTOR_LIMITS{
  int lowerLimit;
  int upperLimit;
  int defaultSpeed;
} MOTOR_LIMITS;

#define NUMBER_POS_SETS  7

typedef struct POSITION_SET {
  int degree;
  int av;
  float aVPerDegree;
} POSITION_SET;


#define MOTORS_DEFINED 3

MOTOR motors[MOTORS_DEFINED] = { {ST3_ENABLE,ST3_DIR,ST3_STEP,ST3_POS,ST3_LIM1,ST3_LIM2,19.0,4.0,0.0,MOTOR_IDLE,0,0},
                                 {ST2_ENABLE,ST2_DIR,ST2_STEP,ST2_POS,ST2_LIM1,ST2_LIM2,15.9,4.0,0.0,MOTOR_IDLE,0,0}, 
                                 {ST1_ENABLE,ST1_DIR,ST1_STEP,ST1_POS,ST1_LIM1,ST1_LIM2,15.9,3.9,0.0,MOTOR_IDLE,0,0}  }; 

const MOTOR_LIMITS motorLimits[MOTORS_DEFINED] = {  {75,274,0},{80,274,0},{75,291,0} }  ;

 POSITION_SET positionSets[MOTORS_DEFINED][NUMBER_POS_SETS] = 
{
    {   
      {70,   950, float(924-850)/(90-75)},
      {90,   850, float(850.0-720)/45},
      {135,  720, float(720.0-507)/45},
      {180,  507, float(507.0-392)/45},
      {225,  392, float(392.0-129)/45},
      {270,  129, float(129.0-81)/45},
      {275,  81,  float(129.0-81)/45}
    },

     {   
      {70,   950, float(938-868)/float(90-75)},
      {90,   868, float(868-694)/float(45)},
      {135,  694, float(694-540)/float(45)},
      {180,  540, float(540-380)/float(45)},
      {225,  380, float(380-164)/float(45)},
      {270,  164, float(164-98)/float(45)},
      {275,  98,  float(129-81)/float(45)}
    },

    {
     {75, 119,  float(135.0-112)/float(90-75)},
     {90, 182,  float(384.0-182)/float(45)},
     {135,384,  float(507.0-384)/float(45)},
     {180,507,  float(681.0-507)/float(45)},
     {225,704,  float(930.0-681)/float(45)},
     {270,930,  float(1007.0-930)/float(45)},
     {300,1023, float(1007.0-930)/float(45)}
    }
};

int skipIntervalTable[] = {2,2,2,2,1,1,1,0};
volatile unsigned int tickA = 0;
volatile byte flag = 0;
unsigned long lastTime;




enum calibrationStates {CAL_IDLE,CAL_WAITING, ZERO_HOME, ZERO_90,ONE_HOME, ONE_90,TWO_HOME, TWO_90};

enum calibrationStates calState = CAL_IDLE;


#define NUM_SERVOS 4
Servo servo[NUM_SERVOS];  // create servo object to control a servo
int servoGpio [NUM_SERVOS] = {SV1_DO,SV2_DO,SV3_DO,SV4_DO};  // all these pins can do PWM the last two are always moved together
int servoAnalog[NUM_SERVOS] = {SV1_A,SV2_A,SV3_A,SV4_A};  // only two have analog inputs

#define SERVO_FIX  5
#define SERVO_FLAT  88

#define ZEROMASK 0xE0

unsigned char lighttest;
unsigned char lin;
unsigned char llast = 0xff;
unsigned long lighttestTime = 0;

unsigned long avReadTime = 0;
char avReadMode = MOTORS_DEFINED+1;

int demoCycles;
int demoCyclesCount;
int demoTicks;

#define MOTOR_ACC    0
#define MOTOR_STEADY 1
#define MOTOR_DEC    2

double mycurrentposition[3];
double* Robot_Plan;
//double RobotPlan[6] = { 0 };

#define MAX_TRAJ 60
#define NUMB_ANGLES 3

double trajectory[MAX_TRAJ][NUMB_ANGLES];
int trajCount = 0;
int trajIndex;

#define TRAJECTORY_IDLE 0
#define TRAJECTORY_ACTIVE 1

char trajectoryState = TRAJECTORY_IDLE;

char offsetL = 0;
char offsetR = 0;

IntervalTimer myTimer;

double myloc(double b1, double b2, double b3);
void* gotopoint(double currentposition[], double newposition[], double ll1, double ll2, int leftrightbit, double tm4, double newtm4);
double* elbow2(double Pef[], double r1, double r2, double tm4, int myleftrightbit);
double* getmycurrentwristposition(double angle1, double angle2, double angle3, double H);
double* getmyarmpoints(double g1, double g2, double a1, double a2, double h, double d2);
double aabs(float absvalue);

using namespace std;
//*************************************test*********************************

int mechToKinDegrees(int motor,int mdegrees){

  //if (motor == 0)
    return mdegrees;

  return mdegrees -180;
}

int kinToMechDegrees(int motor,int kdegrees){

//  if (motor == 0)
    return kdegrees;

  return kdegrees +180;
}


double limitMovement(int m, double inp)
{
   double t = max(double(motorLimits[m].lowerLimit),inp);
   
   t = min(double(motorLimits[m].upperLimit),t);
   return t;

}

void addTrajectoryPoint(double as,double ae,double aw)
{
  if (trajCount < MAX_TRAJ)
  {
    trajectory[trajCount][0]= as ; //limitMovement(0,as+90);
    trajectory[trajCount][1]= ae;  //limitMovement(1,ae);
    trajectory[trajCount][2]= aw;  //limitMovement(2,aw);  
    trajCount++;
    return;

    if (trajCount == 0)
       trajCount++;
    else
      if (  (trajectory[trajCount-1][0] != trajectory[trajCount][0]) ||
            (trajectory[trajCount-1][1] != trajectory[trajCount][1]) ||
            (trajectory[trajCount-1][2] != trajectory[trajCount][2]) )
             trajCount++;   
  }
}


#define plorneg(mynum) (signbit(mynum) ?  -1 : 1)


//************************************* myloc *********************************

double myloc(double b1, double b2, double b3) {

	double my_loc_result = acos((b1 * b1 + b2 * b2 - b3 * b3) / (2 * b1 * b2));

	return my_loc_result;
}

//************************************* getarmpoints *********************************


double armpoints[9] = { 0 };

double* getmyarmpoints(double g1, double g2, double a1, double a2, double h, double d2) {
	//double* p1;
	//double* p2;
	double p1[3] = { 0 };
	double p2[3] = { 0 };
	//armpoints[9] = { 0 };
	double mytheta2;
	double mytheta1;
	double pi = 3.1415928;

	//convert all angles from bobs angles

	if (a1 >= 0 and a1 < 90) {
		mytheta2 = 90 - a1 - a2;
		mytheta1 = 270 - a1;//was 90
		//printf("zone 1 \n");

	}
	else if (a1 >= 90 and a1 < 180) {
		mytheta2 =90-a1-a2;
		mytheta1 = (270-a1);
		//printf("%f,zone 2 \n");
	}
	else if (a1 >= 180 and a1 < 270) {
		mytheta2 = 90 - a1 - a2;
		mytheta1 = 270-a1;
		//printf("zone 3 \n");
	}
	else if (a1 >= 270 and a1 < 360) {
		mytheta2 = 90 - a1 - a2;
		mytheta1 = 270-a1;
		//printf("zone 4 \n");
	}
	else {
		mytheta2 = 90-135-180;
		mytheta1 = 270-135;// something went wrong
		printf("bad value \n");
	};




	p1[0] = g1 * cos(mytheta1 * pi / 180);
	p1[1] = g1 * sin(mytheta1 * pi / 180);
	p1[2] = h;

	p2[0] = p1[0] + g2 * cos((mytheta2) * pi / 180);
	p2[1] = p1[1] + g2 * sin((mytheta2) * pi / 180);
	p2[2] = h;


	armpoints[0] = p1[0];
	armpoints[1] = p1[1];
	armpoints[2] = p1[2];
	armpoints[3] = p1[0] - (g2 * cos((a1 + a2) * pi / 180));  //use these if there is a heght change on wrist servo
	armpoints[4] = p1[1] + (g2 * sin((a1 + a2) * pi / 180));   //use these if there is a heght change on wrist servo
	armpoints[5] = p1[2] + d2;  //use these if there is a heght change on wrist servo
	armpoints[6] = p2[0];
	armpoints[7] = p2[1];
	armpoints[8] = p2[2];


	return armpoints;

}
double RobotPlan[4] = { 0 };
void* gotopoint(double acurrentposition[], double newposition[], double ll1, double ll2, int leftrightbit, double tm4, double newtm4)
{
	double anginc = 5;
	//double wrist_rotation = 0;
	double* Robot_Plan;
	//RobotPlan[4] = { 0 };
	double Curr_msx = acurrentposition[0];
	double Curr_msy = acurrentposition[1];
	//double Curr_msz = acurrentposition[2];
	double Curr_tm = tm4;

	double New_msx = newposition[0];
	double New_msy = newposition[1];
	//double New_msz = newposition[2];
	double New_tm = newtm4;


	double del_x = (New_msx - Curr_msx);
	double del_y = (New_msy - Curr_msy);
	//double del_z = (New_msz - Curr_msz);
	double del_tm = (New_tm - Curr_tm);
	double m = int(del_tm / anginc); // problem here if negative

	int n;
	
	for (n = 1; n <= aabs(int(del_x)); ++n) {
		acurrentposition[0] += 1 * (signbit(del_x) ? -1 : 1);
		Robot_Plan = elbow2(acurrentposition, ll1,ll2, Curr_tm, leftrightbit);
	  addTrajectoryPoint(Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]); 
    		printf("%.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]);
		printf("\n");
		
	};
	for (n = 1; n <= aabs(int(del_y)); ++n) {
		acurrentposition[1] += 1 * (signbit(del_y) ? -1 : 1);
		Robot_Plan = elbow2(acurrentposition, ll1, ll2, Curr_tm, leftrightbit);
	  addTrajectoryPoint(Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]); 
    printf("%.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]);
		printf("\n");
		
	};

	
	for (n = 1; n <= aabs(m); ++n) {
		Curr_tm += anginc * (signbit(del_tm) ? -1 : 1);
		Robot_Plan = elbow2(acurrentposition, ll1, ll2, Curr_tm, leftrightbit);
		printf("%.2f, %.2f, %.2f", Robot_Plan[0], Robot_Plan[1], Robot_Plan[2]);
		printf("\n");
	};
	
	
	return 0;


}
double gotmyposition[3] = { 0 };
double* getmycurrentwristposition(double angle1, double angle2, double angle3, double H) {
	double* myposition;
	//gotmyposition[3] = { 0 };
	//double l00 = 4;
	//double pi = 3.1415928;

	myposition = getmyarmpoints(9, 9, angle1, angle2, H, 0); // 9 and 9 are the length of the bicept and arm and needs to be gloalized.

	gotmyposition[0] = myposition[6]; //+l00 * cos(angle3 * pi / 180);
	gotmyposition[1] = myposition[7]; //+l00 * sin(angle3 * pi / 180);
	//gotmyposition[0] = l00 * cos(angle3 * pi / 180);
	//gotmyposition[1] = l00 * sin(angle3 * pi / 180);
	gotmyposition[2] = myposition[8];

	return gotmyposition;
	
}

double aabs(float absvalue) {
	double myabsvalue;
	if (absvalue < 0)
		myabsvalue = -1 * absvalue;
	else
		myabsvalue = absvalue;

	return myabsvalue;
}

double my_DATA[4] = { 0 };
	
double* elbow2(double Pef[], double r1, double r2, double tm4, int myleftrightbit) {
	//my_DATA[4] = { 0 };
	
  //printf("inside elbow -> %f,%f,%f,%f,%f,%f,%d\n", Pef[0], Pef[1],Pef[2],r1,r2,tm4,myleftrightbit);
  double BobsSholderAngle = 0.0;
	double BobsElbowAngle = 0.0;
	int quadbit = 0;
	double x1 = 0.0;
	double y1 = 0.0;
	double x2 = Pef[0];
	double y2 = Pef[1];
	double d = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
	double k = 180 / 3.1415928;
	//*************************************************************  Bob please see this note
	if (d > (r1 + r2)) {
		d = r1 + r2;
		//need to report that desired point is outside the reach of the robot
	};
	double l = ((r1 * r1) - (r2 * r2) + (d * d)) / (2 * d);
	double h = sqrt((r1 * r1) - (l * l));
	
	// Left Arm elbow point
	double right_x = (l / d) * (x2 - x1) + (h / d) * (y2 - y1) + x1;
	double left_x =  (l / d) * (x2 - x1) - (h / d) * (y2 - y1) + x1;

	// Right Arm elbow point
	double right_y = (l / d) * (y2 - y1) - (h / d) * (x2 - x1) + y1;
	double left_y =  (l / d) * (y2 - y1) + (h / d) * (x2 - x1) + y1;
 // printf("leftrightcalc -> %f,%f,%f,%f,%f,%f,%f,%f\n", left_x, left_y,right_x, right_y, l,h,d,aabs(left_x));

	if (myleftrightbit == 0) {
		// left handed angle calculations
		if (left_x >= 0 and left_y >= 0) {
			if (aabs(left_x) <= 0.0001) {
				left_x = 0.00001;
			};
			BobsSholderAngle = 270 - k * atan(aabs(left_y) / aabs(left_x));
			BobsElbowAngle = 360 - k * myloc(r1, r2, d);
			quadbit = 1;
			//double blah = k * myloc(r1, r2, d);
			//printf("%f, %f, %f\n\n", blah, abs(left_x), left_x, left_y);

		}
		else if (left_x < 0 and left_y > 0) {
			if (aabs(left_x) <= 0.0001) {
				left_x = -0.00001;
			}
       //printf("inside 2 loop -> %f,%f,%f\n", k * atan(aabs(left_y) / aabs(left_x)),left_y, left_x);
			BobsSholderAngle = 90 + k * atan(aabs(left_y) / aabs(left_x));
			BobsElbowAngle = 360 - k * myloc(r1, r2, d);
			quadbit = 2;
			//printf("%f, %f, %f\n\n", k * atan(abs(left_y) / abs(left_x)), left_x, left_y);
		}
		else if (left_x < 0 and left_y < 0) {
			if (aabs(left_x) <= 0.0001) {
				left_x = -0.00001;
			};
			BobsSholderAngle = 90 - k * atan(aabs(left_y) / aabs(left_x));
			BobsElbowAngle = 360 - k * myloc(r1, r2, d);
			quadbit = 3;
		}
		else  // (left_x > 0 and left_y < 0) 
		{
			if (aabs(left_x) <= 0.0001) {
				left_x = 0.00001;
			};
			BobsSholderAngle = 270 + k * atan(aabs(left_y) / aabs(left_x));
			BobsElbowAngle = 360 - k * myloc(r1, r2, d);
			quadbit = 4;
		}

	}
	else if (myleftrightbit == 1) {
		//right handed calculations
		if (right_x >= 0 and right_y >= 0) {
			if (aabs(left_x) <= 0.0001) {
				right_x = 0.00001;
			};
			BobsSholderAngle = 270 - k * atan(aabs(right_y) / aabs(right_x));
			BobsElbowAngle = k * myloc(r1, r2, d);
			quadbit = 11;
		}
		else if (right_x < 0 and right_y > 0) {
			if (aabs(right_x) <= 0.0001) {
				right_x = -0.00001;
			};
			BobsSholderAngle = 90 + k * atan(aabs(right_y) / aabs(right_x));
			BobsElbowAngle = k * myloc(r1, r2, d);
			quadbit = 12;
		}
		else if (right_x < 0 and right_y < 0) {
			if (aabs(right_x) <= 0.0001) {
				right_x = -0.00001;
			};
			BobsSholderAngle = 90 - k * atan(aabs(right_y) / aabs(right_x));
			BobsElbowAngle = k * myloc(r1, r2, d);
			quadbit = 13;
		}
		else  // (right_x > 0 and right_y < 0) 
		{
			if (aabs(right_x) <= 0.0001) {
				right_x = 0.00001;
			};
			BobsSholderAngle = 270 + k * atan(aabs(right_y) / aabs(right_x));
			BobsElbowAngle =   k * myloc(r1, r2, d);  // law of cosines
			quadbit = 14;
		}

	}
	else {
		//error
	}
  if (BobsSholderAngle < 70) {
  BobsSholderAngle = 70;
  };
if (BobsSholderAngle > 270) {
  BobsSholderAngle = 270;
  };
if (BobsElbowAngle < 70) {
  BobsElbowAngle = 70;
  };
if (BobsElbowAngle > 270) {
  BobsElbowAngle = 270;
  };
	my_DATA[0] = BobsSholderAngle;
	my_DATA[1] = BobsElbowAngle;
	my_DATA[2] = tm4;
	 
  //printf("inside elbow -> %f,%f,%f,%d\n", my_DATA[0], my_DATA[1],my_DATA[2],quadbit);
	
	return my_DATA;
};


int main2()
{
	//testing in main

	double l1 = 9; //length of biceps needs to be global
	double l2 = 9; //length of forearm needs to be global
	//double* fmyposition;
	double* hereismyposition;
	double* hereismypositiontest;
	//std::cout << "\n";
	int myleftrightbit = 0; // 0 is for left handed robot arm and 1 is for right handed robot arm
	double mycurrentposition[3] = { -6.25, 15.88, 40 };


	double mynewposition[3] = { 8, 8, 40 };


	hereismyposition = elbow2(mynewposition, l1, l2, 0,  myleftrightbit); // 0 = left arm  , 1 = right arm

	
	printf("%f,%f,%f\n", hereismyposition[0], hereismyposition[1], hereismyposition[2]);
	

	printf("0->************************************************************************\n\n");

	mynewposition[0] = 8;
	mynewposition[1] = 8;
	mynewposition[2] = 40;

	myleftrightbit = 1;
	hereismyposition = elbow2(mynewposition, l1, l2, 0, myleftrightbit); // 0 = left arm  , 1 = right arm

	 
	printf("1->%f,%f,%f\n", hereismyposition[0], hereismyposition[1], hereismyposition[2]);
	 
	//gotopoint(mycurrentposition, mynewposition, l1, l2, myleftrightbit);
	printf("************************************************************************\n\n");
	double bobsshoulderangle = 141.13;
	double bobselbowangle = 216.63;
	double bobswristangle = 0;
	double bobsheight = 40;
	
	hereismypositiontest = getmycurrentwristposition(bobsshoulderangle, bobselbowangle, bobswristangle, bobsheight);  //these angles come from the spreadsheet in MatLab

	printf("2->%f,%f,%f\n\n", hereismypositiontest[0], hereismypositiontest[1], hereismypositiontest[2]);
	printf("************************************************************************\n\n");
	mycurrentposition[0] = -6.26;
	mycurrentposition[1] = 15.88;
	mycurrentposition[2] = 40;
		//(8, 10, 20);
	mynewposition[0] = 8;
	mynewposition[1] = 8;
	mynewposition[2] = 40;
	
	gotopoint(mycurrentposition, mynewposition, l1, l2, 0, 0, 90);
	printf("3->************************************************************************\n\n");
	mycurrentposition[0] = -6.26;
	mycurrentposition[1] = 15.88;
	mycurrentposition[2] = 40;
	//(8, 10, 20);
	mynewposition[0] = 8;
	mynewposition[1] = 8;
	mynewposition[2] = 40;

	gotopoint(mycurrentposition, mynewposition, l1, l2, 1, 0, 90);
	//printf("%f,%f", cos(45), cos(45 * 3.14159 / 180));
	//gotopoint(mycurrentposition, mynewposition, l1, l2, 0, 0, 90);
	printf("4->************************************************************************\n\n");
	mycurrentposition[0] = 8;
	mycurrentposition[1] = 8;
	mycurrentposition[2] = 40;
	//(8, 10, 20);
	mynewposition[0] = -6.;
	mynewposition[1] = 16;
	mynewposition[2] = 40;

	gotopoint(mycurrentposition, mynewposition, l1, l2, 0, 0, 90);
	return 0;
  //printf("%f,%f", cos(45), cos(45 * 3.14159 / 180));
}

double *hereismyposition;

void CurrentLocation(char *cp)
{
    int currentPosition[3];
    
    for (int i = 0; i < 3; i++)
    {
      currentPosition[i] = getCurrentPosition(i);
    }
    
    // convert to x,y,rot

	  hereismyposition = getmycurrentwristposition(double(currentPosition[0]), double(mechToKinDegrees(1,currentPosition[1])), double(mechToKinDegrees(2,currentPosition[2])), double(48));  //these angles come from the spreadsheet in MatLab

    mycurrentposition[0] = hereismyposition[0] ;
    mycurrentposition[1] = hereismyposition[1];
	  mycurrentposition[2] = hereismyposition[2];

    Serial.println(  "OK + (" + String(hereismyposition[0]) + ',' + String(hereismyposition[1]) + ',' + String(hereismyposition[2])  + ',' + String(hereismyposition[3]) + ")"  );
         

}
POSITION_SET * getPositionSetbyDegree(int motor,int degrees, char increasing)
{
  int i;

  if (increasing)  // look for highest
  {
    for (i = 0; i < NUMBER_POS_SETS; i++)
    {
//      Serial.println("Dposition increasing");
//      Serial.println("Dposition itst :" + String(degrees) + " " + String( positionSets[motor][i].degree));
      if (degrees < positionSets[motor][i].degree){
//        Serial.println("Dposition set :" + String(i));
        return &positionSets[motor][i-1];
      }
    }
  }
  else
  {
//    Serial.println("Dposition decreasing");
    for (i = NUMBER_POS_SETS-1; i >= 0; i--) 
    {
//      Serial.println("Dposition dtst :" + String(degrees) + " " + String( positionSets[motor][i].degree));
       if (degrees > positionSets[motor][i].degree)
      {
//        Serial.println("Dposition set :" + String(i+1));
        return &positionSets[motor][i+1];
      }
    }
  }
  Serial.println("Dposition failure :" + String(i));
  return 0;
}

POSITION_SET * getPositionSetbyAv(int motor,int av, char increasing)
{
  int i;

  if (increasing)  // look for highest
  {
    for (i = 0; i < NUMBER_POS_SETS; i++)
    {
 //     Serial.println("Aposition itst :" + String(i) + " " + String( positionSets[motor][i].degree) + " " + String( positionSets[motor][i].av));

      if (av < positionSets[motor][i].av){
  //      Serial.println("Aposition set :" + String(positionSets[motor][i].degree));
        return &positionSets[motor][i-1];
      }
    }
  }
  else
  {
    for (i = NUMBER_POS_SETS-1; i >= 0; i--) 
    {
  //    Serial.println("Aposition dtst :" + String(i) + " " + String( positionSets[motor][i].degree) + " " + String( positionSets[motor][i].av));
     if (av < positionSets[motor][i].av)
      {
  //      Serial.println("Aposition set :" + String(positionSets[motor][i].degree));
       
        return &positionSets[motor][i];
      }
    } 
  }
  Serial.println("Aposition failure :" + String(i));
  return 0;
}

void motorTimer() {
  int m;

  tickA++;

  if (tickA >=2000) //2000 x 250us = 500ms
  {
    flag = 1;
    tickA = 0;
    digitalWrite(13, !digitalRead(13));
  }
  
  for (m=0; m < MOTORS_DEFINED;m++)
  {
    if (motors[m].direction != MOTOR_IDLE)
    {
      //Serial.println(motors[m].ticksPerformed);
      //Serial.println(motors[m].zeroMask,HEX);


      if  ((motors[m].ticksPerformed > motors[m].beginZeroMonitoring)  && (motors[m].stopFlag == 0 ) )
      {
        // we ran out of ticks stop
        motors[m].stopFlag = 1;
        Serial.print("out of ticks ");
        Serial.println(m);
        continue;
      }
    }  

    // see if skipping this tick interrupt
    
    if (motors[m].ticksSkip != 0)
    {
      if (motors[m].ticksSkip < 0)
        Serial.println("negative ticksSkip ");
       
      // yes
      motors[m].ticksSkip--;
      continue;
    }

    // don't skip this interrupt but first see if we are done
    
    if ((motors[m].direction != MOTOR_IDLE) && (motors[m].stopFlag == 0)  )
    {
       motors[m].currentAv = analogRead(motors[m].positionAnalogPin);

       //Serial.println("Y " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
      
      if(motors[m].direction == MOTOR_CW) 
      {

        // motors 0 and 1 rotate opposite motor 2
        
        if (0) // m == 2)
        {
          if (motors[m].currentAv >= motors[m].targetAv)
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("CW2-done " + String(m) + ":"+ String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }
        } 
        else
        {
          if (motors[m].currentAv < motors[m].targetAv)
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("CW01-Done " + String(m) + ":" + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }
        }       
      }
      else
      {
         // going CCW

        if (0) //m == 2)
        {
          if (motors[m].currentAv < motors[m].targetAv) 
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("CCW2-Done  " + String(m) + ":" + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }
        }
        else
        {
          if (0) // motors[m].currentAv >= motors[m].targetAv) 
          {
             // we are done
             motors[m].stopFlag = 1;
             Serial.println("CCw-01-Done " + String(m) + ":" + String(motors[m].ticksLeft) + " " + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
             continue;
          }       
        } 
      }

      // do the tick
      
      if (motors[m].pulseHigh)
      {
        digitalWrite(motors[m].stepGpio, LOW);        
        motors[m].pulseHigh = 0;
        motors[m].ticksLeft --;
        motors[m].position += (motors[m].direction == MOTOR_CW ? 1 : -1);
        motors[m].ticksPerformed++; 

        // was that the last tick?
        
        if (motors[m].ticksLeft == 0) {
          
            Serial.println("X5 " + String(m) + ":" + String(motors[m].currentAv) + ":" + String( motors[m].targetAv));
           
           // see if we are doing repeats for demo

           if (demoCycles != 0)
           {
             demoCycles--;
             motors[m].ticksLeft = demoTicks;
             motors[m].direction = (motors[m].direction == MOTOR_CCW ? MOTOR_CW : MOTOR_CCW);
           }
           motors[m].stopFlag = 1;
           continue;
        }
        
        // check if we are actually making progress
        
        //if ( abs(motors[m].currentAv -  motors[m].lastAv) <  1)
        //   Serial.println('j');

        motors[m].lastAv =  motors[m].currentAv;

        // there are ticks left. Is it time to change the skip interval?
        
        switch (motors[m].speedMode)
        {
        case MOTOR_ACC:
          // still getting up to speed. See if we are there
 //         Serial.print("a");
          
           if (motors[m].ticksPerformed >= motors[m].ticksFullSpeedTrigger )
           {
              // yes, go to steady
              motors[m].speedMode = MOTOR_STEADY;
              motors[m].ticksSkip = 0;
              //Serial.println("AS");
              continue;
           }

           // not time to go steady, is it time to change the skip value

          //Serial.print(motors[m].ticksPerformed);
          //Serial.print(":");          
          //Serial.println(motors[m].ticksIntervalChangeTrigger);
          
          if (motors[m].ticksPerformed >= motors[m].ticksIntervalChangeTrigger )
          {
            // bump interval count
              
            if (motors[m].tickSkipIntervalIndex != motors[m].skipBuckets-1 )
              motors[m].tickSkipIntervalIndex++;             
            // calc next update trigger
            motors[m].ticksIntervalChangeTrigger = motors[m].ticksPerformed + motors[m].skipIntervalChangeIncrement ;                 
            //Serial.print("Ai");
          }

          // load skip value
          motors[m].ticksSkip = skipIntervalTable[motors[m].tickSkipIntervalIndex];
//          Serial.println(motors[m].ticksSkip);
          break;
        
        case MOTOR_DEC:
          //Serial.print("d");
  
          // see if time to go next interval
          if (motors[m].ticksPerformed >= motors[m].ticksIntervalChangeTrigger )
          {
            // dec interval count unless already at lowest value
              
            if (motors[m].tickSkipIntervalIndex != 0 )
              motors[m].tickSkipIntervalIndex--;
            
            motors[m].ticksIntervalChangeTrigger = motors[m].ticksPerformed + motors[m].skipIntervalChangeIncrement ;                 

            //Serial.println("-Dd-");              
          }
          // load skip value
          motors[m].ticksSkip = skipIntervalTable[motors[m].tickSkipIntervalIndex];
          //Serial.println(motors[m].ticksSkip);
         break;;
        
        case MOTOR_STEADY:
          //Serial.print("s");
          // see if time to start slowing down
          
          if (motors[m].ticksPerformed > motors[m].ticksSlowDownTrigger )
           {
              // yes
              motors[m].speedMode = MOTOR_DEC;
              motors[m].tickSkipIntervalIndex = motors[m].skipBuckets-1;
              motors[m].ticksIntervalChangeTrigger = motors[m].ticksPerformed + motors[m].skipIntervalChangeIncrement ;                 
              motors[m].ticksSkip = skipIntervalTable[motors[m].tickSkipIntervalIndex];
              //Serial.print("Sd-");              
              //Serial.println(motors[m].ticksSkip);
         }
          else
            motors[m].ticksSkip = 0;
          break;  
        }          
       }
      else
      {
        // assert pulse low
     //   Serial.print("x");
        digitalWrite(motors[m].stepGpio, HIGH);        
        motors[m].pulseHigh = 1;
      }
    }
  }
}




float aVPerDegree = 3.4;

int getCurrentPosition(int motor )
{
      POSITION_SET * pset;
      int currentPosition;
      motors[motor].currentAv = analogRead(motors[motor].positionAnalogPin);
      Serial.println("Av " + String(motors[motor].currentAv)); 
         
      if (motor == 2)
      {
         // get current degree position from currentAv
           
         pset = getPositionSetbyAv(motor,  motors[motor].currentAv, true);
         currentPosition = pset->degree +  (motors[motor].currentAv - pset->av) / pset->aVPerDegree;
      }
      else 
      {
        pset = getPositionSetbyAv(motor,  motors[motor].currentAv, false);   
        currentPosition = pset->degree + ( pset->av - motors[motor].currentAv) / pset->aVPerDegree;     
      }     
      return currentPosition;
}

POSITION_SET * getTargetMovement(int motor,int newPosition,int *currentPosition)
{
      POSITION_SET * pset;
      int increasingPosition;

      motors[motor].currentAv = analogRead(motors[motor].positionAnalogPin);

      Serial.println("Av " + String(motors[motor].currentAv)); 
         
      if (motor == 2)
      {
           // get current degree position from currentAv
           
           pset = getPositionSetbyAv(motor,  motors[motor].currentAv, true);
           *currentPosition = pset->degree +  (motors[motor].currentAv - pset->av) / pset->aVPerDegree;
           
          increasingPosition =  (newPosition > *currentPosition ? true : false);
          pset = getPositionSetbyDegree(motor,  newPosition,increasingPosition);
       }
      else 
      {
          pset = getPositionSetbyAv(motor,  motors[motor].currentAv, false);         
          *currentPosition = pset->degree + ( pset->av - motors[motor].currentAv) / pset->aVPerDegree;

          increasingPosition =  (newPosition > *currentPosition ? true : false);           
          pset = getPositionSetbyDegree(motor,  newPosition,!increasingPosition);
        }     
       Serial.println("gtm av " + String(motors[motor].currentAv) + " " + String(*currentPosition)); 

        return pset;
}


int moveToPosition(int motor,int newPosition,int start)
{
    int moveAv,currentPosition;
     POSITION_SET * pset;

       Serial.println("mtp: " + String(motor) + " " + String(newPosition) );

        if (newPosition > motorLimits[motor].upperLimit )
        {
          Serial.println ("ERR - too far: " + String(motor) + " " + String(newPosition));
          newPosition = motorLimits[motor].upperLimit-4 ;
        }
        
        if (newPosition < motorLimits[motor].lowerLimit )
        {
          Serial.println ("ERR - too close: " + String(motor) + " " + String(newPosition));
          newPosition = motorLimits[motor].lowerLimit +4 ;
        }

        pset = getTargetMovement(motor,newPosition,&currentPosition);

        Serial.println("curp" +  String(motors[motor].currentAv) + " " + String(currentPosition) );
         
         // we have info to calc targetAv
        
        Serial.println("pset " + String(pset->av) + " " + String(pset->degree) + " " + String(pset->aVPerDegree));
        
        motors[motor].targetAv = pset->av + (newPosition - pset->degree) * pset->aVPerDegree;
                
        moveAv = motors[motor].targetAv - motors[motor].currentAv;               
        motors[motor].moveTicks = moveAv / motors[motor].avPerDegree ;
        
        Serial.println("tgt " + String(motors[motor].targetAv) + " " + String(moveAv) + " " + String(motors[motor].moveTicks));
 
        if (!start)
          return 1;
          
        // start all three motors now
        
        for (int m = 0 ; m < MOTORS_DEFINED; m++)
        {
          if (0) //m == 2)
          {
            startMotor(m,(motors[m].moveTicks >= 0 ? ROT_CW : ROT_CCW),aabs(motors[m].moveTicks)+1000,0);
          }
          else
          {
             startMotor(m,(motors[m].moveTicks < 0 ? ROT_CW : ROT_CCW),aabs(motors[m].moveTicks) + 1000,0);
          }
        }

  return 1;
}

void startMotor(int motor, char cmd,int ticks, int repeat)
{
 
       if (cmd == ROT_CCW)
        {
          motors[motor].direction = MOTOR_CCW;
          digitalWrite(motors[motor].dirGpio,HIGH);
          Serial.println("Going CCW");
        }
        else
        {
          motors[motor].direction = MOTOR_CW;
          digitalWrite(motors[motor].dirGpio,LOW);
          Serial.println("Going CW");
         }

        motors[motor].stopFlag = 0;
        motors[motor].ticksLeft = (ticks == 0? 2 : ticks);
        motors[motor].pulseHigh = 0; 
        motors[motor].beginZeroMonitoring = (motors[motor].ticksLeft < 200 ? motors[motor].ticksLeft/2 : 200);
        motors[motor].ticksPerformed = 0; 
        motors[motor].skipBuckets = sizeof (skipIntervalTable)  /sizeof(skipIntervalTable[0]);
        motors[motor].ticksFullSpeedTrigger = min(200, motors[motor].ticksLeft/8);
        motors[motor].ticksSlowDownTrigger = motors[arg1].ticksLeft - motors[motor].ticksFullSpeedTrigger;
        motors[motor].skipIntervalChangeIncrement = motors[motor].ticksFullSpeedTrigger/motors[motor].skipBuckets;          
        
        motors[motor].speedMode = MOTOR_ACC;
        motors[motor].tickSkipIntervalIndex = 0;
        motors[motor].ticksSkip = skipIntervalTable[motors[motor].tickSkipIntervalIndex];          
        motors[motor].ticksIntervalChangeTrigger = motors[motor].ticksPerformed + motors[motor].skipIntervalChangeIncrement;
        demoTicks = motors[motor].ticksLeft;
        demoCycles = repeat;
        demoCyclesCount = 0;
        
        digitalWrite(motors[motor].enGpio ,LOW);
}

void prt3(char *pc,int v1, int v2, int v3)
{
  Serial.print(pc);
  Serial.print(v1);
  Serial.print(' ');
  Serial.print(v2);
  Serial.print(' ');
  Serial.println(v3);

}
void prt3d(const char *pc,double v1, double v2, double v3)
{
  Serial.print(pc);
  Serial.print(v1);
  Serial.print(' ');
  Serial.print(v2);
  Serial.print(' ');
  Serial.println(v3);
}


int runCommand() {
  int i = 0;
  int m,v;
  //char *p = argv1;
  //char *str;
  int rept, endpt,ticks;
  int currentPosition[3];
  double* hereismyposition;
  arg1 = atol(argv1);
  arg2 = atol(argv2);
  arg3 = atol(argv3);
  arg4 = atol(argv4);
  
   
  switch(cmd) {

  case '0':    // limit test

    if (arg1 == 1)
      limitTest = 1;
    else
      limitTest = 0;
    Serial.println ("OK");
    break;

  case '1':   // solenoid tests
    switch (arg1){
    case 1:
      digitalWrite(SOL1,(arg2 == 1 ? 1 : 0));
      break;

    case 2:
      digitalWrite(SOL2,(arg2 == 1 ? 1 : 0));
      break;
    
    case 3:
      digitalWrite(SOL3,(arg2 == 1 ? 1 : 0));
      break;
    
    default:
      Serial.println ("ERR");
      return 0;
    }
    Serial.println ("OK");
    break;

  case 2:
    if (arg1 == 1)
      lighttest = 1;
    else
      lighttest = 0;
  
    lighttestTime = millis() + 1000;
    Serial.println ("OK");
    break;

  case '3':  // vac sensor and light sensor
    Serial.print  ("OK: ");
    Serial.print  (analogRead(VAC_SENSOR));
    Serial.print  (" : ");
    Serial.print  (analogRead(LIGHT_IN1));
    Serial.print  (" : ");
    Serial.println(digitalRead(LIGHT_IN2));
    break;

  case '4':  // vac 
    if (arg1 == 1)
      v = VALVE1;
    else
      v = VALVE2;
      
    if (arg2 == 1)
      digitalWrite(v,1);
    else 
      digitalWrite(v,0);
    Serial.println ("OK");
    break;

  case '5':

    for (i = 0; i < MOTORS_DEFINED; i ++)
       Serial.println( "OK: " + String(i) + " " + String(analogRead(motors[i].positionAnalogPin)));
    return 0;

  case '6':
    switch (argv1[0]) {
      
    case 'r':

        endpt = 180 -(arg2 & 0xff);
        servo[1].write(endpt);
        break;
    
    case 'l':

        endpt = (arg2 & 0xff);
        servo[0].write(endpt);
        break;
    
    case 's':
      endpt = (arg2 & 0xff);
      servo[2].write(endpt);
      servo[3].write(180-endpt+SERVO_FIX); 
      break;
      
    default:
      Serial.println ("Err");
      return 0;
    }
    Serial.println ("OK");
    break;

  case '7':
    
      Serial.println( "OK - " + String(analogRead(servoAnalog[0])) + " " + String(analogRead(servoAnalog[1])));
      
    break;
  
  case '8':    // servo test   6 servo# endpt repeat

   moveToPosition(arg1,arg2,1);
   Serial.println ("OK");
   break;

    if (arg1 < NUM_SERVOS)
     {
        arg1--;
        endpt = arg2 & 0xff;
        rept = (arg3 == 0 ? 1 : arg3);

        for (i = 0 ; i < rept; i++)
        {
          servo[arg1].write(0);
          delay(500);
          servo[arg1].write(endpt);
          delay(500);
        }
        Serial.println ("OK");
     }
     else
        Serial.println ("Err");
      break;

  case '9':
    
    if (arg1 <  MOTORS_DEFINED)   // for reading stepper motor pots
    {
      if (arg2 == 1)
      {
        avReadMode = arg1;
        avReadTime = millis() + 1000;
      } 
      else
        avReadMode = MOTORS_DEFINED+1;     
    }
    Serial.println ("OK");
    break;

  case ROT_CCW_DEGREES:
  case ROT_CW_DEGREES:
  
    trajectoryState = TRAJECTORY_IDLE;
    if (arg1 < MOTORS_DEFINED)
    {
        ticks = int(motors[arg1].ticksPerDegree * arg2);
        
        cmd = (cmd == ROT_CCW_DEGREES ? 'w' : 'c');

        Serial.println(ticks);
        startMotor(arg1, cmd,ticks, 0);
        Serial.println("OK ");
    }
    else
    {
      Serial.println("ERR"); 
    }
    break;

  case ROT_CCW:
  case ROT_CW:
      
    trajectoryState = TRAJECTORY_IDLE;
 
      if (arg1 < MOTORS_DEFINED)
      {
        startMotor(arg1, cmd,arg2, 0);
        Serial.println("OK ");
      }
      else
        Serial.println("ERR"); 
      break;

  case ROT_HALT:
      for (i = 0; i < MOTORS_DEFINED; i++)
      {
        motors[i].stopFlag = 1;
        demoCycles = 0;
        motors[i].ticksLeft = 0;
        motors[i].pulseHigh = 0;           
        digitalWrite(motors[i].enGpio ,HIGH);
        Serial.println("OK"); 
      }
      break; 

  case ARM_LOCK:
      for (i=0; i < MOTORS_DEFINED;i++)
      {
        digitalWrite(motors[i].enGpio,(arg1 == 1? LOW : HIGH));
      }
      Serial.println("OK");
      break;

  case ROT_TO_POSITION:
   trajectoryState = TRAJECTORY_IDLE;

    if (arg1 < MOTORS_DEFINED)
      {
         moveToPosition(arg1,arg2, (arg1 ==2 ? 1 : 0));
         Serial.println("OK");
      }
      else 
         Serial.println("ERR");
      break;
    

  case GO_XY:
    
  	   double mynewposition[3];

       mynewposition[0] = arg1;
       mynewposition[1] = arg2;
       mynewposition[2] = 48;
       
       Serial.println("current: (" + String(mycurrentposition[0]) + ',' + String(mycurrentposition[1]) + ','+ String(mycurrentposition[2]) + ')');
       Serial.println("target:  (" + String(mynewposition[0]) + ',' + String(mynewposition[1]) + ','+ String(mynewposition[2]) + ')');

      trajCount = 0;
 
  	  gotopoint(mycurrentposition, mynewposition,9.0,9.0,1,0,double(arg3));

   //   for (int i = 0; i < trajCount; i++)
   //     Serial.println("--" + String(i) + " (" + String(trajectory[i][0]) + "," + String(trajectory[i][1])+ "," + String(trajectory[i][2]) + ")" );

      Serial.println("OK");
    
    break;


  case SET_ANGLES:
    // set all three angles
    
    Serial.println("***target: (" + String(arg1) + ',' + String(arg2) + ','+ String(arg3) + ')');

    if (moveToPosition(0,arg1,0) == 0)
    {
      Serial.println("ERR 0");
      break;
    }
    
    if (moveToPosition(1,arg2,0) == 0)
    {
      Serial.println("ERR 1");
      break;
    }
 
    if (moveToPosition(2,arg3,1) == 0)
    {
      Serial.println("ERR 2");
      break;
    }
    Serial.println("OK");
    break;

  case LOCATION:
     if (argv1[0] == 'a')
    {
      Serial.print("OK: " + String(motors[0].position) );
      Serial.print(":"+ String(motors[0].position/motors[0].ticksPerDegree) );
      Serial.print("  "    + String(motors[1].position)) ;
      Serial.print(":" + String(motors[1].position/motors[1].ticksPerDegree) );
      Serial.print("  "  + String(motors[2].position));
      Serial.println(":"+ String(motors[2].position/motors[2].ticksPerDegree) );
    }
    else
      if (argv1[0] == 'p')
      {
         // first get current positionpset
         
         for (int i = 0; i < 3; i++)
         {
             currentPosition[i] = getCurrentPosition(i);
             Serial.println(currentPosition[i]);
         }
    
         // convert to x,y,rot

	       hereismyposition = getmycurrentwristposition(double(currentPosition[0]), double(mechToKinDegrees(1,currentPosition[1])), double(mechToKinDegrees(2,currentPosition[2])), double(48));  //these angles come from the spreadsheet in MatLab

         mycurrentposition[0] = hereismyposition[0] ;
         mycurrentposition[1] = hereismyposition[1];
	       mycurrentposition[2] = hereismyposition[2];

         Serial.println(  "OK + (" + String(hereismyposition[0]) + ',' + String(hereismyposition[1]) + ',' + String(hereismyposition[2])  + ',' + String(hereismyposition[3]) + ")"  );
         
      }
      break;

  case POINT:
    trajectoryState = TRAJECTORY_IDLE;
    if (argv1[0] == 'g')
    {
      trajIndex = 0;
      
      
      trajectoryState = TRAJECTORY_ACTIVE;
      for (m = 0; m < MOTORS_DEFINED; m++)
        moveToPosition(m, trajectory[0][m], (m == 2 ? 1 : 0));            
    }  
    else if (argv1[0] == 'l')
      {
        trajIndex = 0;
        Serial.println(trajCount);

        for (m = 0; m < trajCount; m++)
        {
          Serial.print("(" + String(trajectory[m][0]) ); 
          Serial.print("," + String(trajectory[m][1]) ); 
          Serial.println("," + String(trajectory[m][2]) + ")" ); 
        }                
      } 
 
    Serial.println("OK");
    break;


  case CALIBRATE:
   trajectoryState = TRAJECTORY_IDLE;
    Serial.println(argv1[0]);
    
    if (argv1[0] == 's')
    {
      for (i=0; i < MOTORS_DEFINED;i++)
      {
        digitalWrite(motors[i].enGpio,(arg1 == 1? LOW : HIGH));
      }
      
      calState = CAL_WAITING;
      Serial.println("Arrange arm and enter b p when done");      
    }
    else
       if (argv1[0] == 'p')
       {
          if (calState == CAL_WAITING)
          {
            calState = ZERO_HOME;
            startMotor(0, ROT_CCW,10000, 0);
            Serial.println("OK Proceeding");
          }
          else
            Serial.println("ERR");
       }
       else
         Serial.println("ERR");
    break;

   case BASKET_MOVE:
     if (argv1[0] == 'r')
     {
        // rotate basket 90 degrees, a bit at atime

        for (i = SERVO_FLAT; i != 140; i++)
        {
          servo[2].write(i);
          servo[3].write(180-i+SERVO_FIX);
           delay(50);
       }
     }
     else
     {  // move it back to flat
        for (i = 140; i != SERVO_FLAT; i--)
        {
          servo[2].write(i);
          servo[3].write(180-i+SERVO_FIX);
          delay(50);
        }
     }
     Serial.println("OK");
     break;

   case MINIARM_CALIB:
    for (i = 0; i < NUM_SERVOS;i++)
    {
     servo[i].detach();
    }

    arg1 = max(arg1,10);
    for (i = 0; i < arg1; i++)
    { 
      Serial.println( String(i) + ":" + String(analogRead(servoAnalog[0])) + " " + String(analogRead(servoAnalog[1])));
      delay(500);
    }
    Serial.println( "OK");
    break;

/****************  Macro definitions  ****************/
#define ARM_A                   148    // upper arm
#define ARM_B                   160    // lower arm
#define ARM_2AB                 47360  // 2*A*B
#define ARM_A2                  21904  // A^2
#define ARM_B2                  25600  // B^2
#define ARM_A2B2                47504  // A^2 + B^2
#define ARM_STRETCH_MIN         0
#define ARM_STRETCH_MAX         300
#define ARM_HEIGHT_MIN          -180
#define ARM_HEIGHT_MAX          160
#define FIXED_OFFSET_L          18
#define FIXED_OFFSET_R          36

   case MINIARM_MOVE:
   {

     double _stretch = double(arg1);
     double _height =  double(arg2);

     _stretch = constrain(_stretch, ARM_STRETCH_MIN,   ARM_STRETCH_MAX);		// +55, set zero -stretch 
	   _height  = constrain(_height,  ARM_HEIGHT_MIN,    ARM_HEIGHT_MAX);
 	
     prt3d ("input: ", _stretch, _height, 0);
 
     // angle calculation
	   double stretch2height2 = _stretch * _stretch + _height * _height;              // 
	   double angleA = (acos( (ARM_A2B2 - stretch2height2) / ARM_2AB )) * RAD_TO_DEG; // angle between the upper and the lower
	   double angleB = (atan(_height/_stretch)) * RAD_TO_DEG;                         // 
	   double angleC = (acos((ARM_A2 + stretch2height2 -ARM_B2)/(2 * ARM_A * sqrt(stretch2height2)))) * RAD_TO_DEG; // 

     prt3d ("angles: ", angleA, angleB, angleC);
  
	   int angleR = 180 - angleA - angleB - angleC +133;         // 
	   int angleL = angleB + angleC +102 ;  
	   // 
	   // angle limit
	   //angleL = constrain(angleL, 10 + offsetL, 145 + offsetL);
	   //angleR = constrain(angleR, 25 + offsetR, 150 + offsetR);
	   //angleR = constrain(angleR, angleL - 90 + offsetR, angleR);	// behind  -120+30 = -90
     prt3(" servo position: ",angleR,angleL, 0);

	   // if(angleL<15+offsetL)
	   //   angleR = constrain(angleR, 70 + offsetR, angleR);			// front down
    
     servo[0].write(angleL);
     servo[1].write(180-angleR);
     Serial.println( "OK");
    }
    break;

   default:
      break;
  }
  return 0;
}

/* Clear the current command parameters */
void resetCommand() {
   
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
 
  idx = 0;
  arg = 0;

}


void setup() {

    int i;
    int currentPosition[3];
  
    Serial.begin(9600);
    Serial.println("1");
    Wire.begin();
   
    pinMode(SOL1 ,OUTPUT);
    pinMode(SOL2 ,OUTPUT);
    pinMode(SOL3 ,OUTPUT);
    pinMode( VALVE1,OUTPUT);
    pinMode( VALVE2,OUTPUT);

    pinMode( LIGHT_IN1,INPUT_PULLUP);
   


    for (i = 0; i < MOTORS_DEFINED; i++)
    {
      pinMode(motors[i].stepGpio ,OUTPUT);
      pinMode(motors[i].enGpio ,OUTPUT);
      pinMode(motors[i].dirGpio ,INPUT);
      pinMode(motors[i].limit1,INPUT_PULLUP);
      pinMode(motors[i].limit2,INPUT_PULLUP);
      
      digitalWrite(motors[i].stepGpio,HIGH);   
      digitalWrite(motors[i].enGpio,LOW) ;
      motors[i].direction = MOTOR_IDLE; 
      motors[i].ticksPerAv = motors[i].ticksPerDegree/motors[i].avPerDegree;

    }
   
    for (i = 0; i < NUM_SERVOS;i++)
    {
     servo[i].attach(servoGpio[i]);
     servo[i].write(SERVO_FLAT);
    }
     servo[3].write(SERVO_FLAT+SERVO_FIX);


    myTimer.begin(motorTimer, 250);  // 250us

    avReadMode = MOTORS_DEFINED+1;
    calState = CAL_IDLE;

    
    // get the current location
    
    for (int i = 0; i < 3; i++)
      currentPosition[i] = getCurrentPosition(i);
          
    // convert to x,y,rot

	  double *hereismyposition = getmycurrentwristposition(double(currentPosition[0]-90), double(mechToKinDegrees(1,currentPosition[1])), double(mechToKinDegrees(2,currentPosition[2])), double(48));  //these angles come from the spreadsheet in MatLab
   
    mycurrentposition[0] = hereismyposition[0] ;
    mycurrentposition[1] = hereismyposition[1];
	  mycurrentposition[2] = hereismyposition[2];

    Serial.println(  "OK- Setup Done (" + String(hereismyposition[0]) + ',' + String(hereismyposition[1]) + ',' + String(hereismyposition[2])  + ',' + String(hereismyposition[3]) + ")"  );
 
}

void loop() {
       
  // handle any pending characters
  
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();
 
    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[idx] = 0;
      else if (arg == 2) argv2[idx] = 0;
      else if (arg == 3) argv3[idx] = 0;
      else argv4[idx] = 0;

      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[idx] = 0;
        arg = 2;
        idx = 0;
      } else if (arg == 2)  {
        argv2[idx] = 0;
        arg = 3;
        idx = 0;
      } else if (arg == 3)  {
        argv3[idx] = 0;
        arg = 4;
        idx = 0;
      } else if (arg == 4)  {
        argv4[idx] = 0;
        arg = 5;
        idx = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[idx] = chr;
        idx++;
      }
      else if (arg == 2) {
        argv2[idx] = chr;
        idx++;
      }
      else if (arg == 3) {
        argv3[idx] = chr;
        idx++;
      }
      else if (arg == 4) {
        argv4[idx] = chr;
        idx++;
      }
    }
  }

  // do idle setting here since cannot to mcp calls from inside an ISR
  

    for (int m=0; m < MOTORS_DEFINED;m++)
    {
      if (motors[m].direction == MOTOR_IDLE)
        continue;
        
      if (motors[m].stopFlag == 1)
      {
         // digitalWrite(motors[m].enGpio,HIGH);
         motors[m].direction = MOTOR_IDLE;
       
         if (motors[m].ticksLeft != 0)
         {
           // we aborted early. display remaining ticks. At some point this will be part of calibration process. 

           Serial.print("Ticks Remaining: ");
           Serial.println(motors[m].ticksLeft );
         }
      } 
    }
  
  
  switch (calState){

  case CAL_IDLE:
    break;
    
  case CAL_WAITING:
    break;

  case ZERO_HOME:
    // ignore til movement is done
    if  (motors[0].stopFlag == 0)
      break;

    motors[0].position = motorLimits[0].lowerLimit * motors[0].ticksPerDegree;
    // move it 90 degrees
    calState = ZERO_90;
    startMotor(0, ROT_CW,90*motors[0].ticksPerDegree, 0);     
    break;

  case ZERO_90:
    // ignore til movement is done
    if  (motors[0].stopFlag == 0)
      break;

    calState = ONE_HOME;
    startMotor(1, ROT_CCW,10000, 0);
    break;

  case ONE_HOME:
    // ignore til movement is done
    if  (motors[1].stopFlag == 0)
      break;

    // move it 90 degrees
    motors[1].position = motorLimits[1].lowerLimit*motors[1].ticksPerDegree;
    calState = ONE_90;
    startMotor(1,ROT_CW,90*motors[1].ticksPerDegree, 0);    
    break;

  case ONE_90:
    // ignore til movement is done
    if  (motors[1].stopFlag == 0)
      break;
    calState = TWO_HOME;
    startMotor(2, ROT_CCW,10000, 0);
    break;

  case TWO_HOME:
    // ignore til movement is done
    if  (motors[2].stopFlag == 0)
      break;
    motors[2].position = motorLimits[2].lowerLimit*motors[2].ticksPerDegree;
    calState  = TWO_90;
    startMotor(2,ROT_CW,90*motors[2].ticksPerDegree, 0);     
    break;

  case TWO_90:
    // ignore til movement is done
    if  (motors[2].stopFlag == 0)
      break;
 
    Serial.println("Calibration Complete");
    calState = CAL_IDLE;
    break;
  }

  if (trajectoryState == TRAJECTORY_ACTIVE)
  {
    // we are doing point move, we go on to the next point when all three motors are stopped

    if (motors[0].stopFlag && motors[1].stopFlag && motors[2].stopFlag )
    { 
       // move on to the next point if there is one
  
      trajIndex++;
      
      if (trajIndex < trajCount)
       {
          // got another point
         Serial.print("next point ");
         Serial.println(trajIndex);
         Serial.println( "(" + String(trajectory[trajIndex][0]) + "," +String(trajectory[trajIndex][1]) + "," +String(trajectory[trajIndex][2]) + ")"  );

          for (int m = 0; m < MOTORS_DEFINED-1; m++)
            moveToPosition(m, trajectory[trajIndex][m], (m ==2 ? 0 : 0));  // 1            
       }
       else
       {
        int currentPosition[3];
         trajectoryState = TRAJECTORY_IDLE;
         Serial.println("Movement Complete");
         for (int  i = 0; i < 3; i++)
         {
             currentPosition[i] = getCurrentPosition(i);
             Serial.println(currentPosition[i]);
         }

       }
    }
  }

  if (limitTest)
  {
     zin = (digitalRead(ST1_LIM1) << 5) + (digitalRead(ST1_LIM2) << 4) + (digitalRead(ST2_LIM1) << 3) + (digitalRead(ST2_LIM2) << 2) + (digitalRead(ST3_LIM1) << 1) + (digitalRead(ST3_LIM2)) ;
     if (1) { //zin != zlast){
       Serial.print("L: ");
       Serial.println(zin,HEX);
       zlast = zin;
     }
  }

  if (lighttest) {
    lin = digitalRead(LIGHT_IN2);
    if (lin != llast) {
       Serial.println(lin);
       llast = lin;
    }
  
    if (lighttestTime < millis()) {
      Serial.print("V: ");
      Serial.println(analogRead(LIGHT_IN1));
      lighttestTime = millis() + 1000;     // every second
    }
  }

  if (avReadTime  < millis())
  {
    avReadTime = millis() + 1000;
    switch (avReadMode)
    {
    case 0:
      Serial.println (analogRead(ST1_POS));
      break;
      
    case 1:
      Serial.println (analogRead(ST2_POS));
      break;
      
    case 2:
      Serial.println (analogRead(ST3_POS));
      break;

    default:
      break;
    }
  }
}  
