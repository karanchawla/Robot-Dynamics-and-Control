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

typedef struct PDinv
{
	float Kp;
	float Kd;
}PDinv;

typedef struct vel_filter
{
	float theta_old;
	float omega_old1;
	float omega_old2;
	float omega;
}vel_filter;

//Friction Compensation Parameters

float slopesJoint1[3] = {0.245,0.26,4.8};
float colJoint1[2] = {0.1,-0.1};
float intersectJoint1[2] = {0.3637, -0.31};

float slopesJoint2[3] = {0.25,0.287,3.6};
float colJoint2[2] = {0.05,-0.05};
float intersectJoint2[2] = {0.4759, -0.5031};
float tuneJoint2 = 0.5;

float slopesJoint3[3] = {0.35,0.2132,4.5};
float colJoint3[2] = {0.09,-0.09};
float intersectJoint3[2] = {0.195, -0.5190};

//Global theta desired values
float theta1motor_des = 0.0;
float theta2motor_des = 0.0;
float theta3motor_des = 0.0;

//Declaration of PID objects for 3 joints
PID ctrl1 = {60,3,0.3};
PID ctrl2 = {60,3,0.3};
PID ctrl3 = {60,3,0.3};

PID control1 = {60,3,0.3};

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

static inline float velocityFilter(float thetamotor, vel_filter* vel_filter)
{
	vel_filter->omega = (thetamotor - vel_filter->theta_old)/0.001;
	vel_filter->omega = (vel_filter->omega + vel_filter->omega_old1 + vel_filter->omega_old2)/3.0;

	vel_filter->theta_old = thetamotor;

	vel_filter->omega_old2 = vel_filter->omega_old1;
	vel_filter->omega_old1 = vel_filter->omega;

	return vel_filter->omega;
}

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

//Generate the cubic trajectory which was calculated in MATLAB
void trajectoryGenerator(float t)
{
	float Coeff1[4] = {0.25, 0.0, 13.5, -27.0};
	float Coeff2[4] = {-1.943250000000086e+03, 1.404000000000062e+03, -3.375000000000148e+02, 2.700000000000117e+01};
	/*
	 * 0.12% steady state error
	 */
	if(t <= 333)
	{
		t=t/1000;
		td1 = Coeff1[0]+Coeff1[1]*t+Coeff1[2]*t*t+Coeff1[3]*t*t*t;
		td1dot = Coeff1[1]+2*Coeff1[2]*t+3*Coeff1[3]*t*t;
		td1ddot = 2*Coeff1[2]+6*Coeff1[3]*t;

		td2 = Coeff1[0]+Coeff1[1]*t+Coeff1[2]*t*t+Coeff1[3]*t*t*t;
		td2dot = Coeff1[1]+2*Coeff1[2]*t+3*Coeff1[3]*t*t;
		td2ddot = 2*Coeff1[2]+6*Coeff1[3]*t;

		td3 = Coeff1[0]+Coeff1[1]*t+Coeff1[2]*t*t+Coeff1[3]*t*t*t;
		td3dot = Coeff1[1]+2*Coeff1[2]*t+3*Coeff1[3]*t*t;
		td3ddot = 2*Coeff1[2]+6*Coeff1[3]*t;
	}
	if( t < 4000 && t > 333)
	{
//		td1 = td1;
		td1dot = 0;
		td1ddot = 0;

//		td2 = 0t;
		td2dot = 0;
		td2ddot = 0;

//		td3 = 0.75;
		td3dot = 0;
		td3ddot = 0;
		}
	if( t >=4000 && t <= 4333)
	{
		t= t/1000;

		td1 = Coeff2[0]+Coeff2[1]*t+Coeff2[2]*t*t+Coeff2[3]*t*t*t;
		td1dot = Coeff2[1]+2*Coeff2[2]*t+3*Coeff2[3]*t*t;
		td1ddot = 2*Coeff2[2]+6*Coeff2[3]*t;

		td2 = Coeff2[0]+Coeff2[1]*t+Coeff2[2]*t*t+Coeff2[3]*t*t*t;
		td2dot = Coeff2[1]+2*Coeff2[2]*t+3*Coeff2[3]*t*t;
		td2ddot = 2*Coeff2[2]+6*Coeff2[3]*t;

		td3 = Coeff2[0]+Coeff2[1]*t+Coeff2[2]*t*t+Coeff2[3]*t*t*t;
		td3dot = Coeff2[1]+2*Coeff2[2]*t+3*Coeff2[3]*t*t;
		td3ddot = 2*Coeff2[2]+6*Coeff2[3]*t;
	}
	if( t > 4333)
	{
//		td1 = 0.25;
		td1dot = 0;
		td1ddot = 0;

//		td2 = 0.25;
		td2dot = 0;
		td2ddot = 0;

//		td3 = 0.25;
		td3dot = 0;
		td3ddot = 0;
	}
}


static inline float Controller(char joint , float thetamotor, float thetamotor_dot, PID *ctrl, float* curr_error, float* prev_error, float *total_error)
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
			*curr_error = (td1 - thetamotor);
			*total_error = *total_error + (*prev_error + *curr_error)*.001/2;
			int_error = *total_error;
			tau = ctrl->Kp*(td1-thetamotor) + ctrl->Kd*(td1dot - thetamotor_dot);// + ctrl->Ki*(int_error); //J2*td1ddot
			*prev_error = *curr_error;
			//tau = 0;
			if(thetamotor_dot > colJoint1[0])
			{
				tau = tau + slopesJoint1[0]*thetamotor_dot + intersectJoint1[0] ;
			}
			else if(thetamotor_dot < colJoint1[1])
			{
				tau = tau + slopesJoint1[1]*thetamotor_dot + intersectJoint1[1];
			}
			else
			{
				tau = tau + slopesJoint1[2]*thetamotor_dot;
			}
		//	tau = 0;
		}
		//For joint 2
		else if(joint=='2')
		{
			//Integration error
			*curr_error = (td2 - thetamotor);
			*total_error = *total_error + (*prev_error + *curr_error)*.001/2;
			int_error = *total_error;
			tau = ctrl->Kp*(td2-thetamotor) + ctrl->Kd*(td2dot - thetamotor_dot); //+ ctrl->Ki*(int_error);
			*prev_error = *curr_error;
			//tau = 0;
				if(thetamotor_dot > colJoint2[0])
				{
					tau = tau + tuneJoint2*( slopesJoint2[0]*thetamotor_dot + intersectJoint2[0]) ;

				}
				else if(thetamotor_dot < colJoint2[1])
				{
					tau = tau +  tuneJoint2*(slopesJoint2[1]*thetamotor_dot + intersectJoint2[1]);
				}
				else
				{
					tau = tau +  tuneJoint2*(slopesJoint2[2]*thetamotor_dot);
				}
				//tau = 0;

		}
		//For joint 3
		else if(joint=='3')
		{
			//Integration error
			*curr_error = (td3 - thetamotor);
			*total_error = *total_error + (*prev_error + *curr_error)*.001/2;
			int_error = *total_error;
			tau = ctrl->Kp*(td3-thetamotor) + ctrl->Kd*(td3dot - thetamotor_dot); //+ ctrl->Ki*(int_error);
			*prev_error = *curr_error;
		//	tau = 0;
					if(thetamotor_dot > colJoint3[0])
					{
						tau = tau +  slopesJoint3[0]*thetamotor_dot + intersectJoint3[0] ;
					}
					else if(thetamotor_dot < colJoint3[1])
					{
						tau = tau +  slopesJoint3[1]*thetamotor_dot + intersectJoint3[1];
					}
					else
					{
						tau = tau +  slopesJoint3[2]*thetamotor_dot;
					}

		}
	}
	else
	{
		//Use PD controller for everything else
		*total_error = 0;
		if(joint=='1')
		{
			tau = J1*td1ddot + ctrl->Kp*(td1-thetamotor) + ctrl->Kd*(td1dot - thetamotor_dot);
			//tau = 0;
			if(thetamotor_dot > colJoint1[0])
			{
				tau = tau + slopesJoint1[0]*thetamotor_dot + intersectJoint1[0] ;
			}
			else if(thetamotor_dot < colJoint1[1])
			{
				tau = tau + slopesJoint1[1]*thetamotor_dot + intersectJoint1[1];
			}
			else
			{
				tau = tau + slopesJoint1[2]*thetamotor_dot;
			}
			//tau = 0;
		}
		else if(joint=='2')
		{
			tau = J2*td2ddot + ctrl->Kp*(td2-thetamotor) + ctrl->Kd*(td2dot - thetamotor_dot);
			//tau = 0;
			if(thetamotor_dot > colJoint2[0])
			{
				tau = tau +  tuneJoint2*(slopesJoint2[0]*thetamotor_dot + intersectJoint2[0]) ;

			}
			else if(thetamotor_dot < colJoint2[1])
			{
				tau = tau +  tuneJoint2*(slopesJoint2[1]*thetamotor_dot + intersectJoint2[1]);
			}
			else
			{
				tau = tau + tuneJoint2*( slopesJoint2[2]*thetamotor_dot);
			}
			//tau = 0;

		}
		else if(joint=='3')
		{
			tau = J3*td3ddot + ctrl->Kp*(td3-thetamotor) + ctrl->Kd*(td3dot - thetamotor_dot);
			//tau = 0;
			if(thetamotor_dot > colJoint3[0])
			{
				tau = tau +  slopesJoint3[0]*thetamotor_dot + intersectJoint3[0] ;
			}
			else if(thetamotor_dot < colJoint3[1])
			{
				tau = tau +  slopesJoint3[1]*thetamotor_dot + intersectJoint3[1];
			}
			else
			{
				tau = tau +  slopesJoint3[2]*thetamotor_dot;
			}

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

//Very hacky change this later
float thetamotor_dot[3] = {0,0,0};
float thetamotor[3] = {0,0,0};

float Kp2 = 4000;
float Kd2 = 200;

float Kp3 = 4000;
float Kd3 = 200;

float sintheta2 = 0;
float costheta2 = 0;
float costheta3 = 0;
float sintheta32 = 0;
float costheta32 = 0;

float g = 9.81;

//Inverse Dynamics Control
static inline float invDynamicsController(char joint , float *thetamotor, float *thetamotor_dot, PID *control,float sintheta2,float costheta2,float costheta3,float sintheta32,float costheta32)
{
	//Use the inverse dynamics controller everywhere
	float tau = 0;

	//params for inv dynamics control law
	//float p[5] = {0.0300, 0.0128, 0.0076, 0.0753, 0.0298};
	float p[5] = {	0.0466, 0.0388, 0.0284, 0.1405, 0.1298};//with mass
	// Coeffs for inv dynamics control
	float a2 = td2ddot + Kp2*(td2 - thetamotor[1]) + Kd2*(td2dot-thetamotor_dot[1]);
	float a3 = td3ddot + Kp3*(td3 - thetamotor[2]) + Kd3*(td3dot - thetamotor_dot[2]);

	//Inv dynamics controller

	//For joint 1 it remains the same
	if(joint=='1')
	{
		tau = J1*td1ddot + control->Kp*(td1 - thetamotor[0]) + control->Kd*(td1dot - thetamotor_dot[0]);
		if(thetamotor_dot[0] > colJoint1[0])
		{
			tau = tau + slopesJoint1[0]*thetamotor_dot[0] + intersectJoint1[0] ;
		}
		else if(thetamotor_dot[0] < colJoint1[1])
		{
			tau = tau + slopesJoint1[1]*thetamotor_dot[0] + intersectJoint1[1];
		}
		else
		{
			tau = tau + slopesJoint1[2]*thetamotor_dot[0];
		}
	}
	else if(joint=='2')
	{
		tau = (p[0]*a2 - 2*p[2]*sintheta32*a3) + ( - p[2]*costheta32*thetamotor_dot[2]*thetamotor_dot[2])  + ( -p[3]*g*sintheta2);


//		tau += J2*td2ddot;
		if(thetamotor_dot[1] > colJoint2[0])
		{
			tau = tau +  tuneJoint2*(slopesJoint2[0]*thetamotor_dot[1] + intersectJoint2[0]) ;
			}
		else if(thetamotor_dot[1] < colJoint2[1])
		{
			tau = tau +  tuneJoint2*(slopesJoint2[1]*thetamotor_dot[1] + intersectJoint2[1]);
		}
		else
		{
			tau = tau + tuneJoint2*( slopesJoint2[2]*thetamotor_dot[1]);
		}
	}
	else if(joint=='3')
	{
		tau = ( - p[2]*sintheta32)*a2 + p[1]*a3
				+ ( p[2]*costheta32*thetamotor_dot[1] )*thetamotor_dot[1]
				+ ( -p[4]*g*costheta3 );

//		tau += J3*td3ddot;

		if(thetamotor_dot[2] > colJoint3[0])
		{
			tau = tau +  slopesJoint3[0]*thetamotor_dot[2] + intersectJoint3[0] ;
		}
		else if(thetamotor_dot[2] < colJoint3[1])
		{
			tau = tau +  slopesJoint3[1]*thetamotor_dot[2] + intersectJoint3[1];
		}
		else
		{
			tau = tau +  slopesJoint3[2]*thetamotor_dot[2];
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

	sintheta2 = sin(theta2motor);
	costheta2 = cos(theta2motor);
	costheta3 = cos(theta3motor);
	sintheta32 = sin(theta3motor-theta2motor);
	costheta32 = cos(theta3motor-theta2motor);

	thetamotor[0] = theta1motor;
	thetamotor[1] = theta2motor;
	thetamotor[2] = theta3motor;

	thetamotor_dot[0] = theta1motor_dot;
	thetamotor_dot[1] = theta2motor_dot;
	thetamotor_dot[2] = theta3motor_dot;

	trajectoryGenerator(mycount%8000);

	//Motor torque limitation(Max: 5 Min: -5)

	//Feedforward control for the joints
	*tau1 = Controller('1',theta1motor, theta1motor_dot, &ctrl1, &curr_error1, &prev_error1, &total_error1);
	*tau2 = Controller('2',theta2motor, theta2motor_dot, &ctrl2, &curr_error2, &prev_error2, &total_error2);
	*tau3 = Controller('3',theta3motor, theta3motor_dot, &ctrl3, &curr_error3, &prev_error3, &total_error3);

//
//	*tau1 = invDynamicsController('1',thetamotor,thetamotor_dot, &control1,sintheta2,costheta2,costheta3,sintheta32,costheta32);
//	*tau2 = invDynamicsController('2',thetamotor,thetamotor_dot, &control1,sintheta2,costheta2,costheta3,sintheta32,costheta32);
//	*tau3 = invDynamicsController('3',thetamotor,thetamotor_dot, &control1,sintheta2,costheta2,costheta3,sintheta32,costheta32);

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

	Simulink_PlotVar1 = td1;
	Simulink_PlotVar2 = theta1motor;
	Simulink_PlotVar3 = theta2motor;
	Simulink_PlotVar4 = theta3motor;

	mycount++;
	newCount++;

}

void printing(void){
	serial_printf(&SerialA, "%.2f %.2f %.2f :: %.2f %.2f %.2f :: %.2f %.2f %.2f \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,x,y,z,thetas[0]*180/PI,thetas[1]*180/PI,thetas[2]*180/PI);
}
