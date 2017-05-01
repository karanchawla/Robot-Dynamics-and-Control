#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

/*********************************************************************************
 *
 * Functions
 *
 **********************************************************************************/

void setBlink();
void getInvMotorAngles(float x, float y, float z);
void getForwardKinematics(float theta1motor, float theta2motor, float theta3motor);
void cubic_trajectory(float t);

// Your global variables.  

/*********************************************************************************
 *
 * Lab 0 Variables
 *
 **********************************************************************************/

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;	//variable used to testing printing to MATLAB

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

//variables used to change the blinking pattern of the LED on the emergency stop box
// blinking pattern is SOS in morse
float isDot = 1; //true indicates dot sequence, false indicates dash sequence
long timeCounter = 0;	//counts number of iterations passed
long tempVar = 0;	// variable used to hold intermediate values during calculations
float sequenceCounter = 0; //indicates number of times repeated sequence of dot or dash

/*********************************************************************************
 *
 * Lab 1 Variables
 *
 **********************************************************************************/

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them 
// here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.43;
float offset_Enc3_rad = 0.25;

/*
 * arrays used to hold the output of the calculated inverse kinematics.
 * indexes refer to the motor number.
 * note that element at index 0 is just a buffer value to keep indexes consistent
 */
float invThetasDH[4] = {0, 0, 0, 0};
float invThetasMotor[4] = {0, 0, 0, 0};

/*
 * array used to hold the joint angles calculated from forward kinematics.
 * indexes refer to the joint numbering.
 * note that element at index 0 is just a buffer value in order to keep indexes consistent
 */
float forwardJointTheta[4] = {0, 0, 0, 0};

/*
 * array used to hold the cartisian location (millimeters) of the end effector,
 * obtained from forward kinematics.
 * indexes to the following:
 * 		fowardEE[0] refers to the x location
 * 		fowardEE[1] refers to the y location
 * 		fowardEE[2] refers to the z location
 */
float forwardEE[3] = {0, 0, 0};


//variables used to print the angle of the motor from the sensors to tera term
float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

//variables used to print the angle of the joint to tera term
float printJointTheta1 = 0;
float printJointTheta2 = 0;
float printJointTheta3 = 0;

//variables used to print the angle of the joint derived from inverse kinematics
float printInvTheta1Motor = 0;
float printInvTheta2Motor = 0;
float printInvTheta3Motor = 0;

//variables used to print the end effector location derived from forward kinematics
float printEEX = 0;
float printEEY = 0;
float printEEZ = 0;

//static variables used in calculating inverse kinematics
static float d1 = 254; //distance from table to joint 2
static float a2 = 254; //length between joint 2 and joint 3
static float a3 = 254; //length between joint 3 and end effector

/*
 * variables used for calculating inverse kinematics.
 * for specifics on variable use, please refer to the report
 */
float alpha = 0;
float beta = 0;
float R = 0;
float w = 0;

/*********************************************************************************
 *
 * Lab 2 Variables
 *
 **********************************************************************************/

/*
 * variables for calculation of omegas
 */
int flag = 0;

float theta1_old = 0;
float omega1_old1 = 0;
float omega1_old2 = 0;
float omega1 = 0;

float theta2_old = 0;
float omega2_old1 = 0;
float omega2_old2 = 0;
float omega2 = 0;

float theta3_old = 0;
float omega3_old1 = 0;
float omega3_old2 = 0;
float omega3 = 0;

/*
 * variables for calculation of the integral terms
 */

float I1=0;
float I1_previous=0;
float error1 = 0;
float error1_previous =0;

float I2=0;
float I2_previous=0;
float error2 = 0;
float error2_previous =0;

float I3=0;
float I3_previous=0;
float error3 = 0;
float error3_previous =0;


//Desired theta motor angles
float theta1motor_D = 0;
float theta2motor_D = 0;
float theta3motor_D = 0;

/*
 * variables for calculating torque
 * "raw" values are the output of the control before saturation
 * "sat" values are the "raw" values truncated between -5 and 5
 */
float tau1_raw = 0;
float tau2_raw = 0;
float tau3_raw = 0;

float tau1_sat;
float tau2_sat;
float tau3_sat;


/*
 * variables for following an equation
 */
//Variables used for following a trajectory.  They indicate the desired x,y,z position
float zd;
float xd;
float yd;

//Variables used for following a cubic trajectory
float theta1d_D;
float theta2d_D;
float theta3d_D;

float theta1dd_D;
float theta2dd_D;
float theta3dd_D;

/*********************************************************************************
 *
 * Lab 3 Variables
 *
 **********************************************************************************/

//Variables for friction compensation
float Vpos1 = 0.21;
float Cpos1 = 0.3637;
float Vneg1 = 0.24;
float Cneg1 = -0.2948;


float Vpos2 = 0.235;
float Cpos2 = 0.4759;
float Vneg2 = 0.28;
float Cneg2 = -0.5031;

float Vpos3 = 0.19;
float Cpos3 = 0.25;
float Vneg3 = 0.21;
float Cneg3 = -0.5;


//Parameters without mass
//float p[5]= {0.0300, 0.0128, 0.0076, 0.0753, 0.0298};

//Parameters for mass added
float p[5]= {0.0466, 0.0388, 0.0284, 0.1405, 0.1298};

//Variables and data for the calculation of the quick trajectory
typedef struct steptraj_s {
	long double b[5];
	long double a[5];
	long double xk[5];
	long double yk[5];
	float qd_old;
	float qddot_old;
	int size;
} steptraj_t;

steptraj_t trajectory = {9.6098034448281648e-09L,3.8439213779312659e-08L,5.7658820668968989e-08L,3.8439213779312659e-08L,9.6098034448281648e-09L,
		1.0000000000000000e+00L,-3.9207920792079212e+00L,5.7647289481423405e+00L,-3.7670505997761832e+00L,9.2311388459861921e-01L,
		0,0,0,0,0,
		0,0,0,0,0,
		0,
		0,
		5};

void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot);

float mystep;

/**********************************************************************************
 *
 * Lab 4 Variables
 *
 **********************************************************************************/
// Jacobian
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

//These variables allow us to select the rotation of the axis for the impedance controller
float thetaz = 0;
float thetax = 0;
float thetay = 0;

// Rotation Matrix and transpose
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

//The desired x, y and z in the world frame
float x_D = 10;
float y_D = 0;
float z_D = 20;

//The desired velocities for x, y and z
float xd_D = 0;
float yd_D = 0;
float zd_D = 0;

float x;
float y;
float z;

//Variables for calculating the velocity of x, y and z
float xd = 0;
float x_old = 0;
float xd_old2 = 0;
float xd_old1 = 0;

float yd = 0;
float y_old = 0;
float yd_old2 = 0;
float yd_old1 = 0;

float zd = 0;
float z_old = 0;
float zd_old2 = 0;
float zd_old1 = 0;

//Friction coefficient (if needed)
float fric1 = 1;
float fric2 = 1;
float fric3 = 1;

//Start and end points for following a straight line
float x_a = 16;
float y_a = -3.5;
float z_a = 10;

float x_b = 9;
float y_b = 3.5;
float z_b = 10;

long t = 0;

long t_start = 0;

// Total time for traversing Straight Line
long t_total = 2000; //2 seconds

/***********************************************************************************
 *
 * Gains and set points
 *
 **********************************************************************************/
//PD and PID Controller Lab 2
float Kp1 = 130;
float Kd1 = 3.3;
float Ki1 = 40;
float threshold1 = 0.05;

float Kp2 = 50;
float Kd2 = 2.3;
float Ki2 = 70;
float threshold2 = 0.05;

float Kp3 = 80;
float Kd3 = 2.0;
float Ki3 = 50;
float threshold3 = 0.05;

//Inverse Dynamics Controller Lab 2

float Kp1_INV = 130;
float Kd1_INV = 3.3;

float Kp2_INV = 12000;
float Kd2_INV = 200;

float Kp3_INV = 10000;
float Kd3_INV = 200;

//PD Plus Feedforward Controller Lab 3

float Kp1_FF = 130;
float Kd1_FF = 3.3;

float Kp2_FF = 150;
float Kd2_FF = 2.3;

float Kp3_FF = 80;
float Kd3_FF = 2.0;

// Impedence Control Gains Lab 4

float KpX = 0.5;
float KdX = 0.025;

float KpY = 0.5;
float KdY = 0.025;

float KpZ = 0.5;
float KdZ = 0.025;

//Impedance Control Rotation Lab 4

float KpXn = 0.5;
float KdXn = 0.025;

float KpYn = 0.5;
float KdYn = 0.025;

float KpZn = 0.5;
float KdZn = 0.025;

/**********************************************************************************
 *
 * Final Project Variables
 *
 **********************************************************************************/

#define XYZSTIFF 1
#define ZSTIFF 2
#define XSTIFF 3
#define YSTIFF 4
#define EGGSTIFF 5

typedef struct position {
	float x;
	float y;
	float z;
	float mode;
	float rot;
	float velocity;
} position_t;

#define SLOW 9
#define FAST 7


#define HOLEX 1.16
#define HOLEY 13.75

#define ZZDEPTH 8.50
#define LINE1START_X 14.65
#define LINE1START_Y 4.57
#define LINE1END_X 15.95
#define LINE1END_Y 3.1
#define LINE1ROT 36.9*PI/180

#define LINE2START_X 15.08
#define LINE2START_Y 2.03
#define LINE2END_X 13.32
#define LINE2END_Y 2.39
#define LINE2ROT -15*PI/180

#define LINE3START_X 12.79
#define LINE3START_Y 1.39
#define LINE3END_X 15.26
#define LINE3END_Y -1.36
#define LINE3ROT -36.9*PI/180

#define EGG_X 14.3
#define EGG_Y -4.98
#define EGG_UP 14.28
#define EGG_DOWN 13.8

#define NUM_PTS  26
position_t pos[NUM_PTS]={
		{5.52, 0, 16.61, XYZSTIFF ,0 , FAST+1}, //Home
		{7.35, 8.57, 13, XYZSTIFF ,0 , FAST+1},
		{HOLEX, HOLEY, 13, ZSTIFF , 0 , 5},
		{HOLEX, HOLEY, 5.2, ZSTIFF , 0 , 0},
		{HOLEX, HOLEY, 5.2, ZSTIFF , 0 , 5},
		{HOLEX, HOLEY, 13, XYZSTIFF , 0 , FAST+3},
		{7.35, 8.57, 13, XYZSTIFF, 0 , FAST+3},
		{12.38, 4.57, 13, XYZSTIFF, 0 , SLOW},
		{12.38, 4.57, ZZDEPTH, XYZSTIFF, 0 , FAST+3},
		{LINE1START_X, LINE1START_Y, ZZDEPTH, YSTIFF, LINE1ROT , SLOW}, //Entering zigzag
		{LINE1END_X, LINE1END_Y, ZZDEPTH, YSTIFF , 0, SLOW},
		{LINE1END_X, LINE2START_Y, ZZDEPTH, XSTIFF, 0, SLOW},
		{LINE2START_X, LINE2START_Y, ZZDEPTH, XSTIFF, LINE2ROT, SLOW},
		{LINE2END_X, LINE2END_Y, ZZDEPTH, XSTIFF, 0, SLOW},
		{12.91, 2.18, ZZDEPTH, XSTIFF, 27.12*PI/180, SLOW},
		{12.58, 1.89, ZZDEPTH, XSTIFF, 41.3*PI/180, SLOW},
		{12.79, 1.39, ZZDEPTH, YSTIFF, 22.7*PI/180, SLOW},
		{LINE3START_X, LINE3START_Y, ZZDEPTH, YSTIFF, LINE3ROT, SLOW},
		{LINE3END_X, LINE3END_Y, ZZDEPTH, YSTIFF, 0, FAST},
		{EGG_X, LINE3END_Y, EGG_UP+1, XYZSTIFF, 0, FAST},
		{EGG_X, EGG_Y, EGG_UP+1, XYZSTIFF, 0, FAST},
		{EGG_X, EGG_Y, EGG_UP, XYZSTIFF, 0, FAST},
		{EGG_X, EGG_Y, EGG_DOWN, EGGSTIFF, 0, 0.01},
		{EGG_X, EGG_Y, EGG_DOWN, EGGSTIFF, 0, 0},
		{EGG_X, EGG_Y, EGG_DOWN, XYZSTIFF, 0, FAST},
		{5.52, 0, 16.61, XYZSTIFF ,0 , 0}
};

int index=0;

/**********************************************************************************
 *
 * The main function.  It is called every 1ms
 *
 *********************************************************************************/

void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {
	getForwardKinematics(theta1motor, theta2motor, theta3motor);
	/**********************************************************************************
	*
	* PD and PID Controller Implementation
	*
	*********************************************************************************/
	/**********************************************************************************
	* Velocity
	*********************************************************************************/
	//IIR filter for omega1
	omega1 = (theta1motor - theta1_old)/0.001;
	omega1 = (omega1 + omega1_old1 + omega1_old2)/3;
	theta1_old = theta1motor;
	omega1_old2 = omega1_old1;
	omega1_old1 = omega1;

	//IIR filter for omega2
	omega2 = (theta2motor - theta2_old)/0.001;
	omega2 = (omega2 + omega2_old1 + omega2_old2)/3;
	theta2_old = theta2motor;
	omega2_old2 = omega2_old1;
	omega2_old1 = omega2;

	//IIR filter for omega3
	omega3 = (theta3motor - theta3_old)/0.001;
	omega3 = (omega3 + omega3_old1 + omega3_old2)/3;
	theta3_old = theta3motor;
	omega3_old2 = omega3_old1;
	omega3_old1 = omega3;

	/**********************************************************************************
	* Integral (Trapezoidal Method)
	*********************************************************************************/
	// Integration control for theta1
	if(fabs(theta1motor_D - theta1motor) < threshold1){
		error1 = theta1motor_D - theta1motor;
		I1 = I1_previous + ((error1 + error1_previous)*0.001)/2;
		error1_previous = error1;
		I1_previous = I1;
	}else{
		I1 = 0;
		I1_previous = 0;
	}

	//Integration Control for theta2
	if(fabs(theta2motor_D-theta2motor) < threshold2){
		error2 = theta2motor_D - theta2motor;
		I2 = I2_previous + ((error2 + error2_previous)*0.001)/2;
		error2_previous = error2;
		I2_previous = I2;
	}else{
		I2 = 0;
		I2_previous = 0;
	}

	//Integration Control for theta3
	if(fabs(theta3motor_D-theta3motor) < threshold3){
		error3 = theta3motor_D - theta3motor;
		I3 = I3_previous + ((error3 + error3_previous)*0.001)/2;
		error3_previous = error3;
		I3_previous = I3;
	}else{
		I3 = 0;
		I3_previous = 0;

	/**********************************************************************************
	*
	*Final Project Trajectory
	*
	*********************************************************************************/

	if (pos[index].mode==XYZSTIFF){
		KpXn = 1.5;
		KdXn = 0.025;

		KpYn = 1.5;
		KdYn = 0.025;

		KpZn = 1.5;
		KdZn = 0.025;
	}else if(pos[index].mode==ZSTIFF){
		KpXn = 0.1;
		KdXn = 0.01;

		KpYn = 0.1;
		KdYn = 0.01;

		KpZn = 2;
		KdZn = 0.025;

	}else if(pos[index].mode==XSTIFF){
		KpXn = 2;
		KdXn = 0.025;

		KpYn = 0.1;
		KdYn = 0.01;

		KpZn = 1;
		KdZn = 0.025;
	}else if(pos[index].mode==YSTIFF){
		KpXn = 0.1;
		KdXn = 0.01;

		KpYn = 2;
		KdYn = 0.025;

		KpZn = 1;
		KdZn = 0.025;
	}else if(pos[index].mode==EGGSTIFF){
		KpXn = 2;
		KdXn = 0.025;

		KpYn = 2;
		KdYn = 0.025;

		KpZn = 1.4;
		KdZn = 0.025;
	}

	if(pos[index].velocity==0){
		t_total = 1000;


		x_D = pos[index].x;
		y_D = pos[index].y;
		z_D = pos[index].z;


		xd_D = 0;
		yd_D = 0;
		zd_D = 0;

		if (t > (t_total + t_start)){

			t_start = t;
			index++;
		}

	}else{
		x_a = pos[index].x;
		y_a = pos[index].y;
		z_a = pos[index].z;

		x_b = pos[index+1].x;
		y_b = pos[index+1].y;
		z_b = pos[index+1].z;



		thetaz = pos[index].rot;



		float delta_x = x_b - x_a; //Length and direction in the x direction
		float delta_y = y_b - y_a; //Length and direction in the y direction
		float delta_z = z_b - z_a; //Length and direction in the z direction



		t_total = 1000*(sqrt(delta_x*delta_x+delta_y*delta_y+delta_z*delta_z))/(pos[index].velocity);

		if (t > (t_total + t_start)){

			t_start = t;
			index++;
		}

		x_D = delta_x * (t - t_start)/t_total + x_a;
		y_D = delta_y * (t - t_start)/t_total + y_a;
		z_D = delta_z * (t - t_start)/t_total + z_a;


		xd_D = delta_x/(t_total*0.001);
		yd_D = delta_y/(t_total*0.001);
		zd_D = delta_z/(t_total*0.001);
	}
	t++; //Updating the time counter (1ms each time)


	/*
Task Space Control
Up until this point, we have implemented joint space control, meaning our control was based on
theta_1, theta_2 and theta_3.  By implementing task space control, we derive our control based
on the x, y and z values.  This is done by the use of the analytical Jacobian, which allows us 
to convert between theta_1, theta_2 and theta_3 and x, y and z.  
	*/
	// Jacobian Transpose
	cosq1 = cos(theta1motor);
	sinq1 = sin(theta1motor);
	cosq2 = cos(theta2motor);
	sinq2 = sin(theta2motor);
	cosq3 = cos(theta3motor);
	sinq3 = sin(theta3motor);

	JT_11 = -10*sinq1*(cosq3 + sinq2);
	JT_12 = 10*cosq1*(cosq3 + sinq2);
	JT_13 = 0;
	JT_21 = 10*cosq1*(cosq2 - sinq3);
	JT_22 = 10*sinq1*(cosq2 - sinq3);
	JT_23 = -10*(cosq3 + sinq2);
	JT_31 = -10*cosq1*sinq3;
	JT_32 = -10*sinq1*sinq3;
	JT_33 = -10*cosq3;

	/*
Forward Kinematics
The forward kinematics equations are used to determine the present location of the end effector.
Since the feedback from the arm is in the form of joint angles as theta1motor, theta2motor and 
theta3motor, we use the forward kinematics to obtain the x, y, and z position of the end effector.
	*/
	x = 10 * cosq1 * (cosq3 + sinq2);
	y = 10 * sinq1 * (cosq3 + sinq2);
	z = 10 * (1 + cosq2 - sinq3);

	/*
Calculation of Velocity
The PD controller requires velocity to calculate the control effort.  If we only compare the most 
recent values of x (for example) to determine the velocity of x, it will be a very noisy signal.  
To overcome this, we used an IIR filter as was done in Lab 2.  The IIR, or Infinite Impulse 
Response, filter uses an average of the current x velocity, and the two previous averages of velocity.  
Thus it retains an effect from all previous values of velocity.  This results in a less noisy 
calculation of velocity for our controller.
	*/
	xd = (x - x_old)/0.001;
	xd = (xd + xd_old1 + xd_old2)/3;
	x_old = x;
	xd_old2 = xd_old1;
	xd_old1 = xd;

	yd = (y - y_old)/0.001;
	yd = (yd + yd_old1 + yd_old2)/3;
	y_old = y;
	yd_old2 = yd_old1;
	yd_old1 = yd;

	zd = (z - z_old)/0.001;
	zd = (zd + zd_old1 + zd_old2)/3;
	z_old = z;
	zd_old2 = zd_old1;
	zd_old1 = zd;

	/*
Taskspace Controller
This is the implementation of the Taskspace PD controller.  The PD controller calculates a 
control effort in the x, y and z directions and the Jacobian is used to convert these to 
the control effort for each joint.  As the equations are very complex, we divided it up 
into two steps to calculate the raw control effort.  This effort is then passed to friction 
compensation and saturation limitations.
	*/
	/*
	float cont_x = KpX*(x_D - x) + KdX*(xd_D - xd); //F_x
	float cont_y = KpY*(y_D - y) + KdY*(yd_D - yd); //F_y
	float cont_z = KpZ*(z_D - z) + KdZ*(zd_D - zd); //F_z


	tau1_raw = JT_11*cont_x + JT_12*cont_y + JT_13*cont_z;
	tau2_raw = JT_21*cont_x + JT_22*cont_y + JT_23*cont_z;
	tau3_raw = JT_31*cont_x + JT_32*cont_y + JT_33*cont_z;
	*/
	/*
Impedance Control with Rotation
The previous controller was designed based on the x, y, and z of the world frame, but by 
applying a rotational matrix, we can change the orientation of the x, y and z axes.  
With only the world frame, the arm is very limited in choice of directions to strengthen 
or weaken.  By rotating the reference frame, the arm can stiff or weak in any direction.
	*/

	/*
Rotation Matrix
The rotation matrix takes the values of thetax, thetay and thetaz, which are global variable, 
and creates a rotation matrix.  The rotation matrix rotates from the world frame (W) to the 
new coordinate frame (N).  Rotation from the N frame to the W frame is done by the 
transpose of the rotation matrix.
	*/

	// Rotation zxy and its Transpose
	cosz = cos(thetaz);
	sinz = sin(thetaz);
	cosx = cos(thetax);
	sinx = sin(thetax);
	cosy = cos(thetay);
	siny = sin(thetay);
	RT11 = R11 = cosz*cosy-sinz*sinx*siny;
	RT21 = R12 = -sinz*cosx;
	RT31 = R13 = cosz*siny+sinz*sinx*cosy;
	RT12 = R21 = sinz*cosy+cosz*sinx*siny;
	RT22 = R22 = cosz*cosx;
	RT32 = R23 = sinz*siny-cosz*sinx*cosy;
	RT13 = R31 = -cosx*siny;
	RT23 = R32 = sinx;
	RT33 = R33 = cosx*cosy;

	/*
Taskspace Controller with Rotation
This is the implementation of the Taskspace PD controller with rotation.  As the equations are very complex, 
we divided it up into three steps.  A1, A2 and A3 (for example) represent a row of an intermediate matrix 
calculation. They are then used to calculate the raw control effort.  This effort is then passed to friction 
compensation and saturation limitations.  By using the transpose of the rotation matrix on the x, y, and z 
of frame N, we put them back into frame W which makes using the arm much simpler.
	*/

	float A1 = KpXn*(RT11*(x_D - x) + RT12*(y_D - y) + RT13*(z_D - z));
	float A2 = KpYn*(RT21*(x_D - x) + RT22*(y_D - y) + RT23*(z_D - z));
	float A3 = KpZn*(RT31*(x_D - x) + RT32*(y_D - y) + RT33*(z_D - z));

	float B1 = KdXn*(RT11*(xd_D - xd) + RT12*(yd_D - yd) + RT13*(zd_D - zd));
	float B2 = KdYn*(RT21*(xd_D - xd) + RT22*(yd_D - yd) + RT23*(zd_D - zd));
	float B3 = KdZn*(RT31*(xd_D - xd) + RT32*(yd_D - yd) + RT33*(zd_D - zd));

	float C1 = (R11*(A1 + B1) + R12*(A2 + B2) + R13*(A3 + B3));
	float C2 = (R21*(A1 + B1) + R22*(A2 + B2) + R23*(A3 + B3));
	float C3 = (R31*(A1 + B1) + R32*(A2 + B2) + R33*(A3 + B3));

	tau1_raw = JT_11*C1 + JT_12*C2 + JT_13*C3;
	tau2_raw = JT_21*C1 + JT_22*C2 + JT_23*C3;
	tau3_raw = JT_31*C1 + JT_32*C2 + JT_33*C3;

	/**********************************************************************************
	* Friction Compensation
	*********************************************************************************/

	if(omega1>0.1){
		tau1_raw = tau1_raw + fric1*(Vpos1*omega1 + Cpos1);
	}else if(omega1<-0.1){
		tau1_raw = tau1_raw + fric1*(Vneg1*omega1 + Cneg1);
	}else{
		tau1_raw = tau1_raw + 3.6*omega1;
	}

	if(omega2>0.1){
		tau2_raw = tau2_raw + fric2*(Vpos2*omega2+Cpos2);
	}else if(omega2<-0.1){
		tau2_raw = tau2_raw + fric2*(Vneg2*omega2+Cneg2);
	}else{
		tau2_raw = tau2_raw + 3.6*omega2;
	}

	if(omega3>0.1){
		tau3_raw = tau3_raw + fric3*(Vpos3*omega3+Cpos3);
	}else if(omega3<-0.1){
		tau3_raw = tau3_raw + fric3*(Vneg3*omega3+Cneg3);
	}else{
		tau3_raw = tau3_raw + 3.6*omega3;
	}

	/**********************************************************************************
	* Saturation
	*********************************************************************************/
	// Saturation for tau1
	if(tau1_raw <-5){
		tau1_sat = -5;
	}else if(tau1_raw > 5){
		tau1_sat = 5;
	}else{
		tau1_sat = tau1_raw;
	}

	// Saturation for tau2
	if(tau2_raw < -5){
		tau2_sat = -5;
	}else if(tau2_raw > 5){
		tau2_sat = 5;
	}else{
		tau2_sat = tau2_raw;
	}

	// Saturation for tau3
	if(tau3_raw < -5){
		tau3_sat = -5;
	}else if(tau3_raw > 5){
		tau3_sat = 5;
	}else{
		tau3_sat = tau3_raw;
	}

	/**********************************************************************************
	* input torque to robot arm
	*********************************************************************************/
/*
	if(pos[index].velocity == 0){
		tau1_sat =0;
		tau2_sat =0;
		tau3_sat =0;
	}
*/
	if(index>NUM_PTS-1){
		index=NUM_PTS-1;
	}

	//tau1_sat =0;
	//tau2_sat =0;
	//tau3_sat =0;

	*tau1 = tau1_sat;
	*tau2 = tau2_sat;
	*tau3 = tau3_sat;

	// save past states
	if ((mycount%50)==0) {
		theta1array[arrayindex] = theta1motor;
		if (arrayindex >= 100) {
			arrayindex = 0;
		} else {
			arrayindex++;
		}
	}

	if ((mycount%500)==0) {
		if (whattoprint > 0.5) {
			serial_printf(&SerialA, "I love robotics\n\r");
		} else {
			printtheta1motor = theta1motor;
			printtheta2motor = theta2motor;
			printtheta3motor = theta3motor;
			SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		}
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card

		//function dictates the blinking of the LED on the emergency stop box
		setBlink();

		//setting theta2array test based on lab instructions
		theta2array[0] = 10;
		theta2array[1] = 11;

		//set variables to print joint angle based off of motor angle obtained from robot's sensor
		printJointTheta1 = forwardJointTheta[1];
		printJointTheta2 = forwardJointTheta[2];
		printJointTheta3 = forwardJointTheta[3];

		//set variables to print the cartesian location of the end effector based on forward kinematics
		printEEX = forwardEE[0];
		printEEY = forwardEE[1];
		printEEZ = forwardEE[2];

		//calculates inverse kinematics based on the the location of the end effector
		getInvMotorAngles(printEEX, printEEY, printEEZ);

		//set variables to print the angles of the joints outputed by inverse kinematics
		printInvTheta1Motor = invThetasDH[1];
		printInvTheta2Motor = invThetasDH[2] + PI/2;
		printInvTheta3Motor = invThetasDH[2] + invThetasDH[3];

	}


	Simulink_PlotVar1 = theta3motor_D;
	Simulink_PlotVar2 = theta1motor;
	Simulink_PlotVar3 = theta2motor;
	Simulink_PlotVar4 = theta3motor;


	mycount++;
}
/*
 *This function receives time t and calculates velocity and acceleration for each joint
 *to track a cubic polynomial. The equations were derived from matlab code.  It moves each 
 *joint from 0 radians to 0.5 radians in 1 second and then returns to 0 radians in 1 second.
 *It sets everything to 0 for time greater than 2 seconds.
 */
void cubic_trajectory(float t){
	if(t<=1){
		theta1motor_D = 1.5*t*t - 1.0*t*t*t;
		theta1d_D = 3.0*t -3.0*t*t;
		theta1dd_D = 3-6*t;
		theta2motor_D = theta1motor_D;
		theta2d_D = theta1d_D;
		theta2dd_D = theta1dd_D;
		theta3motor_D = theta1motor_D;
		theta3d_D = theta1d_D;
		theta3dd_D = theta1dd_D;
	}else if((t>1)&&(t<=2)){
		theta1motor_D  = -2 + 6.0*t - 4.5*t*t + 1.0*t*t*t;
		theta1d_D = 6-9*t+3.0*t*t;
		theta1dd_D = -9+6.0*t;
		theta2motor_D  = theta1motor_D;
		theta2d_D = theta1d_D;
		theta2dd_D = theta1dd_D;
		theta3motor_D  = theta1motor_D;
		theta3d_D = theta1d_D;
		theta3dd_D = theta1dd_D;
	}else{
		theta1motor_D  = 0;
		theta1d_D = 0;
		theta1dd_D = 0;
		theta2motor_D  = 0;
		theta2d_D = 0;
		theta2dd_D = 0;
		theta3motor_D  = 0;
		theta3d_D = 0;
		theta3dd_D = 0;
	}
}


void printing(void){
	//serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);
	//serial_printf(&SerialA, "Sensor tracked motor angles: \n\r");
	//serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r", theta1motor, theta2motor, theta3motor);
	//serial_printf(&SerialA, "printing joint angles from robot sensors: \n\r");
	//serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r", printJointTheta1*180/PI, printJointTheta2*180/PI, printJointTheta3*180/PI);
	serial_printf(&SerialA, "printing x,y,z: \n\r");
	serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r", printEEX, printEEY, printEEZ);
	//serial_printf(&SerialA, "printing inverse kinematics joint angles: \n\r");
	//serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r",  invThetasDH[1]*180/PI, invThetasDH[2]*180/PI, invThetasDH[3]*180/PI);
	//serial_printf(&SerialA, "printing inverse kinematics motor angles: \n\r");
	//serial_printf(&SerialA, "%.2f, %.2f, %.2f \n\r\n",  printInvTheta1Motor*180/PI, printInvTheta2Motor*180/PI, printInvTheta3Motor*180/PI);
}

/*
 *
 * This function decides the blinking of the LED on the emergency control box.
 * The blinking is in the form or SOS in Morse code (dot dot dot dash dash dash). This is done by having timeCounter
 * track the discrete time that this program is running. With discrete time being tracked we
 * split the the time into blocks of 5 units, where each unit is a dot or dash.
 * The sequenceCounter variable tracks the number of dots or dashes that has been done.
 * Dot is done by having the LED on for the first time unit and off for the rest of the time units in the block.
 * Dash is inverse of the Dot sequence
 */
void setBlink(){
	tempVar = timeCounter%5;

	if(isDot == 1){	//for when LED should be blinking in dot sequence
		if(tempVar == 0){	//light on for first of 5 unit time to indicate dot
			GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1; // ON
		}
		else {
			GpioDataRegs.GPBSET.bit.GPIO60 = 1; // OFF
		}
		if(tempVar == 4){	//at 5th unit time increase the sequence counter
			sequenceCounter++;
			//checks sequence counter, decides if the next block of sequences follows dot or dash
			if(sequenceCounter >= 3){
				//next sequence is dash sequence, so reset sequenceCounter to 0
				sequenceCounter = 0;
				isDot = 0;
			}
		}

	}
	else{
		if(tempVar == 4){	//for when LED should be blinking in dash sequence
			GpioDataRegs.GPBSET.bit.GPIO60 = 1; // OFF

			sequenceCounter++;
			// checks sequence counter, decides if next block of sequences is dot or dash
			if(sequenceCounter >= 3){
				//next sequence is dot sequence, reset sequenceCounter to 0
				sequenceCounter = 0;
				isDot = 1;
			}
		}
		else{
			GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1; // ON
		}

	}
	timeCounter++;	//increment unit time block
}

/*
 * This function takes in the xyz location (millimeters) and calculates the inverse
 * kinematics of the robot arm. The output is the joint angles of the robot, stored
 * in the array invThetasDH. For the explanation of the process of this function, please
 * refer to the report
 */
void getInvMotorAngles(float x, float y, float z){
	R = sqrt(x*x + y*y);
	w = sqrt((z-d1)*(z-d1) + R*R);
	alpha = atan2(z - d1, R);
	invThetasDH[1] = atan2(y, x);
	invThetasDH[3] = PI - acos((a2*a2 + a3*a3 - w*w)/(2*a2*a3));
	beta = invThetasDH[3] / 2;
	invThetasDH[2] = -beta - alpha;

	invThetasMotor[1] = invThetasDH[1];
	invThetasMotor[2] = invThetasDH[2] + PI/2;
	invThetasMotor[3] = invThetasDH[3] + invThetasDH[2];
}

/*
 * This function takes in the motor angles obtained from the robot's sensors and calculates the
 * joint angles and the Cartesian location based on the forward kinematics. A description of the
 * formulas used can be found in the lab report
 */
void getForwardKinematics(float theta1motor, float theta2motor, float theta3motor){
	//obtains joint angles based off of motor angle obtained from robot's sensor
	forwardJointTheta[1] = theta1motor;
	forwardJointTheta[2] = theta2motor - PI/2;
	forwardJointTheta[3] = -theta2motor + theta3motor + PI/2;

	//obtains the cartesian location of the end effector loation
	forwardEE[0] = 10*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
	forwardEE[1] = 10*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
	forwardEE[2] = 10*(1 + cos(theta2motor) - sin(theta3motor));
}

//Function for the quick trajectory
// this function must be called every 1ms.
void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot) {
	int i = 0;

	traj->xk[0] = step;
	traj->yk[0] = traj->b[0]*traj->xk[0];
	for (i = 1;i<traj->size;i++) {
		traj->yk[0] = traj->yk[0] + traj->b[i]*traj->xk[i] - traj->a[i]*traj->yk[i];
	}

	for (i = (traj->size-1);i>0;i--) {
		traj->xk[i] = traj->xk[i-1];
		traj->yk[i] = traj->yk[i-1];
	}

	*qd = traj->yk[0];
	*qd_dot = (*qd - traj->qd_old)*1000;  //0.001 sample period
	*qd_ddot = (*qd_dot - traj->qddot_old)*1000;

	traj->qd_old = *qd;
	traj->qddot_old = *qd_dot;
}