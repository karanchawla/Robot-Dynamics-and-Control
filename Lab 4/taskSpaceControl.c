#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c
// You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.3834;													// Offsets of real angles to the encoder angles
float offset_Enc3_rad = 0.2756;

float cx =0;																							// Initialize the center of x
float cy=0;																								// Initialize the center of y
float cz=0;																								// Initialize the center of z

// Your global varialbes.
long mycount = 0;
long newCount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

//positions in the global coordinate frame
float x;
float y;
float z;
float thetas[3];

//PID Gains structure definition
typedef struct TaskSpace
{
	float Kp;
	float Kd;
	float Ki;
}TaskSpace;

typedef struct vel_filter_axis
{
	float axis_old;
	float vel_old1;
	float vel_old2;
	float vel;
}vel_filter_axis;

typedef struct vel_filter
{
	float theta_old;
	float omega_old1;
	float omega_old2;
	float omega;
}vel_filter;

//Friction Compensation Parameters

float slopesJoint1[3] = {0.245,0.26,4.8};											//Slopes for friction-angle curve of joint 1
float colJoint1[2] = {0.1,-0.1};															//Low angular velocity range of joint 1
float intersectJoint1[2] = {0.3637, -0.31};                   //Intersections for friction-angle curve of joint 1

float slopesJoint2[3] = {0.25,0.287,3.6};											//Slopes for friction-angle curve of joint 2
float colJoint2[2] = {0.05,-0.05};														//Low angular velocity range of joint 2
float intersectJoint2[2] = {0.4759, -0.5031};									//Intersections for friction-angle curve of joint 2
float tuneJoint2 = 0.4;

float slopesJoint3[3] = {0.35,0.2132,4.5};										//Slopes for friction-angle curve of joint 3
float colJoint3[2] = {0.09,-0.09};														//Low angular velocity range of joint 3
float intersectJoint3[2] = {0.195, -0.5190};									//Intersections for friction-angle curve of joint 3

//Global theta desired values
float theta1motor_des = 0.0;																	//Initialize the desired motor angles
float theta2motor_des = 0.0;
float theta3motor_des = 0.0;

//Declaration of task space PID objects for 3 joints
TaskSpace ctrlx = {0.8,0.075,0};
TaskSpace ctrly = {0.5,0.08,0};
TaskSpace ctrlz = {0.5,0.05,0};

//Initializing velocity struct
vel_filter_axis filterx = {0,0,0,0};													//Initialize the parameters for filters
vel_filter_axis filtery = {0,0,0,0};
vel_filter_axis filterz = {0,0,0,0};

vel_filter_axis filter1 = {0,0,0,0};
vel_filter_axis filter2 = {0,0,0,0};
vel_filter_axis filter3 = {0,0,0,0};

//Initialization for jacobian matrix
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

//Utility sin cos
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

//Force intialization
float Fx = 0;
float Fy = 0;
float Fz = 0;

float xdot = 0;
float ydot = 0;
float zdot = 0;

//Utility variables defined for storing the torque calculations by the utility_controller function
float taur1 = 0;
float taur2 = 0;
float taur3 = 0;

//Rotation matrix declarations
float R11 = 0;																									//Initialize Rotation Matrix
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;																									//Initialize the transpose of Rotation Matrix
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;
float cosz = 0;																									//Initialize the values of cos(z), sin(z), cos(x), sin(x), cos(y) and sin(y)
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0.2;
float thetax = 0.2;
float thetay = 0.2;

// Starting location for the trajectory (x,y,z)
float traj[3] = {10,0,15};
// Ending location for the trajectory (x,y,z)
float traj1[3] = {15,0,15};
// Desired end effector position calculated from trajectory every 1ms (x,y,z)
float desired[3] = {0,0,0};
// Desired end effector velocity calculated from trajectory every 1ms (xdot,ydot,zdot)
float desiredDot[3] = {0,0,0};
// Motor angles during the robot arm motion
float thetamotor[3] = {0,0,0};
// Motor omegas calculated using the filter function
float thetamotor_dot[3] = {0,0,0};
// Flag used to indicate the end of trajectory and to go back along the same path
int gobackFlag = 0;
// Variable to change trajectory speed
int travelTime = 2000;

//Auxilary variables defined for the trajectory generation
float del_x = 0;
float del_y = 0;
float del_z = 0;


/*
 The code above mainly initialize the global parameters we need to use in the next code
*/

/*
Inputs: Pointer to Filter struct for storing the filter parameters and variable for which velocity needs to be calculated
Return: Filtered velocity for the axis
*/

static inline float Filter(vel_filter_axis* vel_filter_axis, float value )
{

		vel_filter_axis->vel = (value - vel_filter_axis->axis_old)/0.001;																								//calculate the angular velocity at instant of time
		vel_filter_axis->vel = (vel_filter_axis->vel + vel_filter_axis->vel_old1 + vel_filter_axis->vel_old2)/3.0;      //calculate the average of angular velocity in past three time steps

		vel_filter_axis->axis_old = value;																																							//Renew the old values

		vel_filter_axis->vel_old2 = vel_filter_axis->vel_old1;
		vel_filter_axis->vel_old1 = vel_filter_axis->vel;

	return vel_filter_axis->vel;																																										 //Output the angular velocity after Filtered
}

/*
This structure creates a filter to calculate the smoothed angular velocity by averaging the previous three instant angular velocity. Therefore, the extreme high angular velocity can be avoided.
*/

//Inverse kinematics for the robot arm
/*
Inputs: Pointer to thetas array for storing thus calculated motor angles
Return: The function returns void as the required values are stored in the array
*/
void invKinematics(float x, float y, float z, float* thetas)
{
	float z0 = z - 10;																																															 //Turn the position values to the DH frame angles
	float d = (x*x + y*y + z0*z0 - 200)/200;
	float q1inv = atan2(y, x);
	float q3inv =  atan2(sqrt(1 - d*d), d);
	float q2inv = -atan2(z0, sqrt(x * x + y * y) ) - atan2(10 * sin(q3inv) , 10 + 10 * cos(q3inv));

	float q1 = q1inv;																																																//Turn the DH frame angles to the machine angles
	float q2 = q2inv + PI/2;
	float q3 = q3inv + q2 - PI/2;

	thetas[0] = q1;																																																	// Apply the machine angles to the desired angles
	thetas[1] = q2;
	thetas[2] = q3;
}
/*
This function calculates the machine angles of each joints according to the provided position of end effector, in elbow-up position
*/


/*
Inputs: Takes void as the input as all variables used by the function are global
Return: The function doesn't return anything as the values are stored inside the global x,y and z variables
*/
static inline void forwardKinematics(void)
{
	x = 10 * cosq1 * (cosq3+sinq2);																																									// Turn the machine angles to the position of end effector
	y = 10 * sinq1 * (cosq3+sinq2);
	z = 10 * (1 + cosq2 - sinq3);
}
/*
This function calculates the position of end effector according to the joint angles.
*/

/*
Inputs: Takes void as the input as all variables used by the function are global
Return: Returns void as this function off loads the computation from main controller function
*/

static inline void controllerUtility(void)
{
	cosz = cos(thetaz);																																															//Pre-Calculate the values of cos(z), sin(z), cos(x), sin(x), cos(y) and sin(y) to improve the efficiency for next calculations
	sinz = sin(thetaz);
	cosx = cos(thetax);
	sinx = sin(thetax);
	cosy = cos(thetay);
	siny = sin(thetay);

	JT_11 = -10*sinq1*(cosq3 + sinq2);																																							//Calculate the jacobian
	JT_12 = 10*cosq1*(cosq3 + sinq2);
	JT_13 = 0;
	JT_21 = 10*cosq1*(cosq2 - sinq3);
	JT_22 = 10*sinq1*(cosq2 - sinq3);
	JT_23 = -10*(cosq3 + sinq2);
	JT_31 = -10*cosq1*sinq3;
	JT_32 = -10*sinq1*sinq3;
	JT_33 = -10*cosq3;

	RT11 = R11 = cosz*cosy-sinz*sinx*siny;																																					//Calculate the Rotation Matrix and the transpose of Rotation Matrix
	RT21 = R12 = -sinz*cosx;
	RT31 = R13 = cosz*siny+sinz*sinx*cosy;
	RT12 = R21 = sinz*cosy+cosz*sinx*siny;
	RT22 = R22 = cosz*cosx;
	RT32 = R23 = sinz*siny-cosz*sinx*cosy;
	RT13 = R31 = -cosx*siny;
	RT23 = R32 = sinx;
	RT33 = R33 = cosx*cosy;

	float error_x = RT11*(desired[0]-x) + RT12*(desired[1]-y) + RT13*(desired[2]-z);																//Calculate the error between desired angles to the real angles
	float error_y = RT21*(desired[0]-x) + RT22*(desired[1]-y) + RT23*(desired[2]-z);
	float error_z = RT31*(desired[0]-x) + RT32*(desired[1]-y) + RT33*(desired[2]-z);

	float x_dot = Filter(&filterx,x);																																								//Apply the filters to the velocity part to calculate the velocity of end effector in all directions
	float y_dot = Filter(&filtery,y);
	float z_dot = Filter(&filterz,z);

	float errorx_dot = RT11*(desiredDot[0]-x_dot)+RT12*(desiredDot[1]-y_dot)+RT13*(desiredDot[2]-z_dot);           //Calculate the error between desired velocity to the real velocity in all directions
	float errory_dot = RT21*(desiredDot[0]-x_dot)+RT22*(desiredDot[1]-y_dot)+RT23*(desiredDot[2]-z_dot);
	float errorz_dot = RT31*(desiredDot[0]-x_dot)+RT32*(desiredDot[1]-y_dot)+RT33*(desiredDot[2]-z_dot);

	 taur1= R11*(ctrlx.Kp*(error_x)+ctrlx.Kd*errorx_dot)+R12*(ctrly.Kp*(error_y)+ctrly.Kd*errory_dot)+R13*(ctrlz.Kp*(error_z)+ctrlz.Kd*errorz_dot); //Apply PID controller to generate the final desired torques at joints 1
	 taur2= R21*(ctrlx.Kp*(error_x)+ctrlx.Kd*errorx_dot)+R22*(ctrly.Kp*(error_y)+ctrly.Kd*errory_dot)+R23*(ctrlz.Kp*(error_z)+ctrlz.Kd*errorz_dot); //Apply PID controller to generate the final desired torques at joints 2
	 taur3= R31*(ctrlx.Kp*(error_x)+ctrlx.Kd*errorx_dot)+R32*(ctrly.Kp*(error_y)+ctrly.Kd*errory_dot)+R33*(ctrlz.Kp*(error_z)+ctrlz.Kd*errorz_dot); //Apply PID controller to generate the final desired torques at joints 3

	return;
}

/*
This structure calculates the jacobians and Rotation Matrix first. Calculate the error of position and the error of velocity in all directions. Finally, using the PID control to control the torque at each joints.
*/




/*
Inputs: Char for the joint and the angular velocity of the motor which is computed by the vel_filter function every 1ms in the lab function
Return: Returns the computed torque that needs to be applied to each motor
*/

static inline float Controller(char joint, float thetamotor_dot)
{
	float tau = 0;

	if(joint == 'x')																																																			//Calculate the final torque at joint 1 with considering the frictions at joints in different velocity ranges
	{
		if(thetamotor_dot > colJoint1[0])
		{
			tau = JT_11*taur1+JT_12*taur2+JT_13*taur3 + slopesJoint1[0]*thetamotor_dot + intersectJoint1[0] ;
		}
		else if(thetamotor_dot < colJoint1[1])
		{
			tau = JT_11*taur1+JT_12*taur2+JT_13*taur3 + slopesJoint1[1]*thetamotor_dot + intersectJoint1[1];
		}
		else
		{
			tau = JT_11*taur1+JT_12*taur2+JT_13*taur3 + slopesJoint1[2]*thetamotor_dot;
		}
	}
	else if(joint == 'y')																																																	//Calculate the final torque at joint 2 with considering the frictions at joints in different velocity ranges
	{
		if(thetamotor_dot > colJoint2[0])
		{
			tau = JT_21*taur1+JT_22*taur2+JT_23*taur3 + tuneJoint2*(slopesJoint2[0]*thetamotor_dot + intersectJoint2[0]) ;
		}
		else if(thetamotor_dot < colJoint2[1])
		{
			tau = JT_21*taur1+JT_22*taur2+JT_23*taur3 + tuneJoint2*(slopesJoint2[1]*thetamotor_dot + intersectJoint2[1]);
		}
		else
		{
			tau = JT_21*taur1+JT_22*taur2+JT_23*taur3 + tuneJoint2*(slopesJoint2[2]*thetamotor_dot);
		}
	}
	else if(joint == 'z')																																																	//Calculate the final torque at joint 3 with considering the frictions at joints in different velocity ranges
	{
		if(thetamotor_dot > colJoint3[0])
		{
			tau = JT_31*taur1+JT_32*taur2+JT_33*taur3 + slopesJoint3[0]*thetamotor_dot + intersectJoint3[0] ;
		}
		else if(thetamotor_dot < colJoint3[1])
		{
			tau = JT_31*taur1+JT_32*taur2+JT_33*taur3 + slopesJoint3[1]*thetamotor_dot + intersectJoint3[1];
		}
		else
		{
			tau = JT_31*taur1+JT_32*taur2+JT_33*taur3 + slopesJoint3[2]*thetamotor_dot;
		}
	}

	//Saturate output torque
	if(tau>5)
		return 5.0;
	else if(tau < -5)
		return -5.0;
	else
		return tau;
}

/*
This structure calculates the final torque of the joints. Taking the friction into consideration to decrease the error. The parameter tuneJoint2 is used to reduce the influence of additional torque at joint 2 which used to counter friction
*/

// This function is called every 1 ms
/*
the main function refreshes motor angular velocity and all auxillary and trigonometry definitions for impedence control that relates to motor angles or motor angular velocities.
Then a straight line following section is implemented enabling the end effector to track back and forth between one point to another in 3D space, which is done basically by defining desired motor angles and velocties every 1ms.
Lastly we call controller and controllerUtility to implement impedence control. While the roboti arm is performing straight line following, its rotation matrix and PD gains can be tuned to reach various impedence settings, which
is an ideal method for future line following challenges.
*/
void lab(float theta1motor, float theta2motor, float theta3motor, float *tau1, float *tau2, float *tau3, int error) {

	// Calculate the motor speeds using the filter
	float theta1motor_dot = Filter(&filter1, theta1motor);
	float theta2motor_dot = Filter(&filter2, theta2motor);
	float theta3motor_dot = Filter(&filter3, theta3motor);

	// Some auxillary definitions

	cosq1 = cos(theta1motor);
	sinq1 = sin(theta1motor);
	cosq2 = cos(theta2motor);
	sinq2 = sin(theta2motor);
	cosq3 = cos(theta3motor);
	sinq3 = sin(theta3motor);

	// Combine three motor angular velocites parameter into one array as a parameter in function controller
	thetamotor_dot[0] = theta1motor_dot;
	thetamotor_dot[1] = theta2motor_dot;
	thetamotor_dot[2] = theta3motor_dot;

	forwardKinematics(); // refresh task space coordinates values x,y,z using forwardKinematics with current motor angles inputs.

	if (mycount%travelTime == 0)// set up a gobackFlag for a "go forth" and a "go back" condition in straight line following, the flag is toggled every "travelTime" millisecond.
	{
		if (gobackFlag == 0)
			gobackFlag = 1;
		else
			gobackFlag = 0;
	}
	long trajCount = mycount%travelTime; //set up a long number to count steps during either "go forth" or "go back" condition within "travelTime".
	if (gobackFlag == 0) // "go forth" condition toggled
	{
		del_x = traj1[0] - traj[0]; //distance from point 0 to point 1 in x direction
		del_y = traj1[1] - traj[1]; //distance from point 0 to point 1 in y direction
		del_z = traj1[2] - traj[2]; //distance from point 0 to point 1 in z direction


		desired[0] = del_x *((float)(trajCount) / travelTime) + traj[0]; //Calculate desired x position value for each specific step within "travelTime" in "go forth" condition
		desired[1] = del_y *((float)(trajCount) / travelTime) + traj[1]; //Calculate desired z position value for each specific step within "travelTime" in "go forth" condition
		desired[2] = del_z *((float)(trajCount) / travelTime) + traj[2]; //Calculate desired y position value for each specific step within "travelTime" in "go forth" condition
	}
	else // "go back" condition
	{
		del_x = traj1[0] - traj[0]; //distance from point 0 to point 1 in x direction
		del_y = traj1[1] - traj[1]; //distance from point 0 to point 1 in y direction
		del_z = traj1[2] - traj[2]; //distance from point 0 to point 1 in z direction

		//reverse the distance values in order to "go back"
		del_x = -del_x;
		del_y = -del_y;
		del_z = -del_z;

		desired[0] = del_x *((float)(trajCount) / travelTime) + traj1[0];//Calculate desired x position value for each specific step within "travelTime" in "go back" condition
		desired[1] = del_y *((float)(trajCount) / travelTime) + traj1[1];//Calculate desired y position value for each specific step within "travelTime" in "go back" condition
		desired[2] = del_z *((float)(trajCount) / travelTime) + traj1[2];//Calculate desired z position value for each specific step within "travelTime" in "go back" condition

		//Caculate desired task space velocities according to distances and "travelTime".
		desiredDot[0] = del_x*1000.0 / travelTime;
		desiredDot[1] = del_y*1000.0 / travelTime;
		desiredDot[2] = del_z*1000.0 / travelTime;
	}

	//Motor torque limitation(Max: 5 Min: -5)
	//Task space Impedance control for the joints

	controllerUtility();// call controllerUtility() to generate control effort of impedence control

	//call controller to add friction compensation to x,y,z accordingly.
	*tau1 = Controller('x', theta1motor_dot);
	*tau2 = Controller('y', theta2motor_dot);
	*tau3 = Controller('z', theta3motor_dot);


	// save past states
	if ((mycount % 50) == 0) {

		theta1array[arrayindex] = theta1motor;
		theta2array[arrayindex] = theta2motor;
		if (arrayindex >= 100) {
			arrayindex = 0;
		}
		else {
			arrayindex++;
		}

	}

	if ((mycount % 1000) == 0) {
		printtheta1motor = theta1motor;
		printtheta2motor = theta2motor;
		printtheta3motor = theta3motor;
		SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
		GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
	}




	mycount++;
	newCount++;


}

void printing(void){
	serial_printf(&SerialA, "%.2f %.2f %.2f :: %.2f %.2f %.2f :: %.2f %.2f %.2f \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,x,y,z,thetas[0]*180/PI,thetas[1]*180/PI,thetas[2]*180/PI);
}
