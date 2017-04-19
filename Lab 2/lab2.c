#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.3834;
float offset_Enc3_rad = 0.2756;


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
typedef struct PID
{
	float Kp;
	float Kd;
	float Ki;
}PID;

typedef struct vel_filter
{
	float theta_old;
	float omega_old1;
	float omega_old2;
	float omega;
}vel_filter;

//Inverse kinematics for the robot arm
void invKinematics(float x, float y, float z, float* thetas)
{
	float z0 = z - 10;
	float d = (x*x + y*y + z0*z0 - 200)/200;
	float q1inv = atan2(y, x);
	float q3inv =  atan2(sqrt(1 - d*d), d);
	float q2inv = -atan2(z0, sqrt(x * x + y * y) ) - atan2(10 * sin(q3inv) , 10 + 10 * cos(q3inv));

	float q1 = q1inv;
	float q2 = q2inv + PI/2;
	float q3 = q3inv + q2 - PI/2;

	thetas[0] = q1;
	thetas[1] = q2;
	thetas[2] = q3;
}

//Global theta desired values
float theta1motor_des = 0.0;
float theta2motor_des = 0.0;
float theta3motor_des = 0.0;

//Declaration of PID objects for 3 joints
PID ctrl1 = {60,2,0.3};
PID ctrl2 = {60,1,0.3};
PID ctrl3 = {60,1,0.3};

//Initializing velocity struct
vel_filter filter1 = {0,0,0,0};
vel_filter filter2= {0,0,0,0};
vel_filter filter3= {0,0,0,0};

float prev_error1 = 0.0;
float prev_error2 = 0.0;
float prev_error3 = 0.0;

float curr_error1 = 0.0;
float curr_error2 = 0.0;
float curr_error3 = 0.0;

float total_error1 = 0.0;
float total_error2 = 0.0;
float total_error3 = 0.0;


float DesiredValue[3] = {0,0,0};


//Feedforward constants
float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;

static inline float velocityFilter(float thetamotor, vel_filter* vel_filter)
{
	vel_filter->omega = (thetamotor - vel_filter->theta_old)/0.001;
	vel_filter->omega = (vel_filter->omega + vel_filter->omega_old1 + vel_filter->omega_old2)/3.0;

	vel_filter->theta_old = thetamotor;

	vel_filter->omega_old2 = vel_filter->omega_old1;
	vel_filter->omega_old1 = vel_filter->omega;

	return vel_filter->omega;
}

//PID control definition
static inline float PIDController(float thetamotor, float theta_desired, float thetamotor_dot, PID *ctrl, float* curr_error, float* prev_error, float *total_error)
{
	//Integration error
	*curr_error = (theta_desired-thetamotor);
	*total_error = *total_error + (*prev_error + *curr_error)*.001/2;
	float int_error = *total_error;
	//Use the PID controller close to desired angles
	float tau = 0;
	if(fabs(*curr_error) < 0.2) {
		tau = ctrl->Kp*(theta_desired-thetamotor) + ctrl->Kd*(-thetamotor_dot) + ctrl->Ki*(int_error);
	} else {
		//Use PD controller over everything else
		*total_error = 0;
		tau = ctrl->Kp*(theta_desired-thetamotor) + ctrl->Kd*(-thetamotor_dot) + ctrl->Ki*(0);
	}

	*prev_error = *curr_error;

	//Saturate output torque
	if(tau>5)
		return 5.0;
	else if(tau < -5)
		return -5.0;
	else
		return tau;
}

//Function genrates theta, thetadot and thetaddot for the trajectory
void trajectoryGenerator(float t)
{
	float Coeff1[4] = {0.0, 0.0, 1.5, -1.0};
	float Coeff2[4] = {-2.0, 6.0, -4.5, 1.0};

	if(t < 1000)
	{
		t=t/1000;
		DesiredValue[0] = Coeff1[0]+Coeff1[1]*t+Coeff1[2]*t*t+Coeff1[3]*t*t*t;
		DesiredValue[1] = Coeff1[1]+2*Coeff1[2]*t+3*Coeff1[3]*t*t;
		DesiredValue[2] = 2*Coeff1[2]+6*Coeff1[3]*t;
	}
	if( t >=1000 && t <= 2000)
	{
		t= t/1000;
		DesiredValue[0] = Coeff2[0]+Coeff2[1]*t+Coeff2[2]*t*t+Coeff2[3]*t*t*t;
		DesiredValue[1] = Coeff2[1]+2*Coeff2[2]*t+3*Coeff2[3]*t*t;
		DesiredValue[2] = 2*Coeff2[2]+6*Coeff2[3]*t;
	}
	if( t > 2000 )
	{
			DesiredValue[0] = 0;
			DesiredValue[1] = 0;
			DesiredValue[2] = 0;
	}
}

//Inverse kinematics for the arm in 2D plane
void InverseK(float x, float y, float z, float L, float *td1, float* td2, float* td3)
{
	*td1 = atan2(y,x);

	float R = sqrt(x*x + y*y);
	float d = sqrt(R*R + z*z);
	float beta = atan2(z,R);
	float gamma = atan2(sqrt(L*L - d*d/4),d/2);

	*td2 = PI/2 - gamma - beta;
	*td3 = gamma - beta;
}

float td1 = 0;
float td2 = 0;
float td3 = 0;

float td10 = 0;
float td20 = 0;
float td30 = 0;

float td1n = 0;
float td2n = 0;
float td3n = 0;

float td1dot = 0;
float td2dot = 0;
float td3dot = 0;

float td1ddot = 0;
float td2ddot = 0;
float td3ddot = 0;

float xr0 = 12;
float y0 = 0;
float zr0 = 8;
float Rr0 = 3;

//Function to generate the desired trajectory
static inline void circleTrajectory(float t)
{

	float time = t*0.001;
	float period = 4;
	float phi = 2*PI*time/period;

	//float T = 10000;
	float delta = 0.005;

	if( time <= 100*period)
	{
		x = xr0 + Rr0 * sin(phi);
		y = y0;
		z = zr0 + Rr0 * cos(phi);

		//InverseK(x0, y0, z0, 10.0, &td10, &td20, &td30);

		InverseK(x, y, z, 10.0, &td1n, &td2n, &td3n);
		float x0 = xr0 + Rr0* sin(phi - PI*delta*2/period);
		y0 = 0;
		float z0 = zr0 + Rr0*cos(phi - PI*delta*2/period);

		float x2 = xr0 + Rr0* sin(phi + PI*delta*2/period);
		float y2 = 0;
		float z2 = zr0 + Rr0*cos(phi + PI*delta*2/period);

		InverseK(x0, y0, z0, 10.0, &td10, &td20, &td30);
		InverseK(x2, y2, z2, 10.0, &td1, &td2, &td3);

	    td1dot = (td1 - td10)/delta/2;
	    td2dot = (td2 - td20)/delta/2;
	    td3dot = (td3 - td30)/delta/2;

	    td1ddot = (td1 - 2*td1n + td10)/(delta*delta);
	    td2ddot = (td2 - 2*td2n + td20)/(delta*delta);
	    td3ddot = (td3 - 2*td3n + td30)/(delta*delta);

	}
}

//Infinity trajectory definition
static inline void infinity(float t)
{
	float time = t*0.001;
	float period = 8;
	float phi = 2*PI*time/period;

	float delta = 0.005;
	float scale = 2/(3 - cos(2*phi));

	if( time <= 100*period)
	{
		x = xr0 + Rr0 * scale *cos(phi);
		y = 0;
		z = zr0 + Rr0 * scale*sin(2*phi)/2;
		InverseK(x, y, z, 10.0, &td1n, &td2n, &td3n);

		float x0 = xr0 + Rr0 * scale*cos(phi - PI*delta*2/period);
		y0 = 0;
		float z0 = zr0 + Rr0 *scale*sin(2*(phi - PI*delta*2/period))/2;

		InverseK(x0, y0, z0, 10.0, &td10, &td20, &td30);

		float x2 = xr0 + Rr0*scale*cos(phi + PI*delta*2/period);
		float y2 = 0;
		float z2 = zr0 + Rr0*scale*sin(2*(phi + PI*delta*2/period))/2;

		InverseK(x2, y2, z2, 10.0, &td1, &td2, &td3);

		td1dot = (td1 - td10)/delta/2;
		td2dot = (td2 - td20)/delta/2;
		td3dot = (td3 - td30)/delta/2;

		td1ddot = (td1 - 2*td1n + td10)/(delta*delta);
		td2ddot = (td2 - 2*td2n + td20)/(delta*delta);
		td3ddot = (td3 - 2*td3n + td30)/(delta*delta);
	}



}

//Feedforward control definition
static inline float feedForwardController(char joint , float thetamotor, float thetamotor_dot, PID *ctrl, float* curr_error, float* prev_error, float *total_error)
{
	//Use the PID controller close to desired angles
	float tau = 0;
	float int_error = 0;

	if(fabs(*curr_error) < 0.2)
	{
		//For joint 1
		if(joint=='1')
		{
			//Integration error
			*curr_error = (td10 - thetamotor);
			*total_error = *total_error + (*prev_error + *curr_error)*.001/2;
			int_error = *total_error;
			tau = ctrl->Kp*(td10-thetamotor) + ctrl->Kd*(td1dot - thetamotor_dot) + ctrl->Ki*(int_error); //J2*td1ddot
			*prev_error = *curr_error;
		}
		//For joint 2
		else if(joint=='2')
		{
			//Integration error
			*curr_error = (td20 - thetamotor);
			*total_error = *total_error + (*prev_error + *curr_error)*.001/2;
			int_error = *total_error;
			tau = ctrl->Kp*(td20-thetamotor) + ctrl->Kd*(td2dot - thetamotor_dot) + ctrl->Ki*(int_error);
			*prev_error = *curr_error;
		}
		//For joint 3
		else if(joint=='3')
		{
			//Integration error
			*curr_error = (td30 - thetamotor);
			*total_error = *total_error + (*prev_error + *curr_error)*.001/2;
			int_error = *total_error;
			tau = ctrl->Kp*(td30-thetamotor) + ctrl->Kd*(td3dot - thetamotor_dot) + ctrl->Ki*(int_error);
			*prev_error = *curr_error;
		}
	}
	else
	{
		//Use PD controller for everything else
		*total_error = 0;
		if(joint=='1')
		{
			tau = J1*td1ddot + ctrl->Kp*(td10-thetamotor) + ctrl->Kd*(td1dot - thetamotor_dot);
		}
		else if(joint=='2')
		{
			tau = J2*td2ddot + ctrl->Kp*(td20-thetamotor) + ctrl->Kd*(td2dot - thetamotor_dot);
		}
		else if(joint=='3')
		{
			tau = J3*td3ddot + ctrl->Kp*(td30-thetamotor) + ctrl->Kd*(td3dot - thetamotor_dot);
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

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

	float theta1motor_dot = velocityFilter(theta1motor, &filter1);
	float theta2motor_dot = velocityFilter(theta2motor, &filter2);
	float theta3motor_dot = velocityFilter(theta3motor, &filter3);

	if((mycount%5000)==0)
	{
		if(theta1motor_des==0)
		{
			theta1motor_des = PI/6;
			theta2motor_des = PI/6;
			theta3motor_des = PI/6;
		}
		else
		{
			theta1motor_des = 0;
			theta2motor_des = 0;
			theta3motor_des = 0;
		}
	}

	//trajectoryGenerator(mycount);

	//circleTrajectory(mycount);

	infinity(mycount);

	//Motor torque limitation(Max: 5 Min: -5)

	//PID control for the joints
	*tau1 = PIDController(theta1motor, theta1motor_des, theta1motor_dot, &ctrl2, &curr_error1, &prev_error1, &total_error1);
	*tau2 = PIDController(theta2motor, theta2motor_des, theta2motor_dot, &ctrl2, &curr_error2, &prev_error2, &total_error2);
	*tau3 = PIDController(theta3motor, theta3motor_des, theta3motor_dot, &ctrl2, &curr_error3, &prev_error3, &total_error3);

	//Feedforward control for the joints
//	*tau1 = feedForwardController('1',theta1motor, theta1motor_dot, &ctrl1, &curr_error1, &prev_error1, &total_error1);
//	*tau2 = feedForwardController('2',theta2motor, theta2motor_dot, &ctrl2, &curr_error2, &prev_error2, &total_error2);
//	*tau3 = feedForwardController('3',theta3motor, theta3motor_dot, &ctrl3, &curr_error3, &prev_error3, &total_error3);

	// save past states
	if ((mycount%50)==0) {

		theta1array[arrayindex] = theta1motor;
		theta2array[arrayindex] = theta2motor;
		if (arrayindex >= 100) {
			arrayindex = 0;
		} else {
			arrayindex++;
		}

	}

	if ((mycount%1000)==0) {
		printtheta1motor = theta1motor;
		printtheta2motor = theta2motor;
		printtheta3motor = theta3motor;
		SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
		GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
	}

	Simulink_PlotVar1 = theta2motor_des;
	Simulink_PlotVar2 = theta2motor;
	Simulink_PlotVar3 = *tau2;
	Simulink_PlotVar4 = theta2motor-theta2motor_des;

	mycount++;
	newCount++;
}

void printing(void){
	serial_printf(&SerialA, "%.2f %.2f %.2f :: %.2f %.2f %.2f :: %.2f %.2f %.2f \n\r",printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI, x, y, z, thetas[0]*180/PI, thetas[1]*180/PI, thetas[2]*180/PI);
}