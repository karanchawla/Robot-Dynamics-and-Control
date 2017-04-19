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

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

    //Motor torque limitation(Max: 5 Min: -5)

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
        if (whattoprint > 0.5) {
            serial_printf(&SerialA, "I love robotics\n\r");
        } else {
            printtheta1motor = theta1motor;
            printtheta2motor = theta2motor;
            printtheta3motor = theta3motor;
            SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        //GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }
    if(newCount<=250)
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
    }

    if(newCount>250 && newCount<=280)
    {
        GpioDataRegs.GPBSET.bit.GPIO60 = 1;
    }
    if(newCount>260 && newCount<=552)
    {
        GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
    }

    if(newCount >552 && newCount <1000)
    {
        GpioDataRegs.GPBSET.bit.GPIO60 = 1;
    }
    if(newCount > 1000)
    {
        newCount = 0;
    }

//

    Simulink_PlotVar1 = 0;
    Simulink_PlotVar2 = 0;
    Simulink_PlotVar3 = 0;
    Simulink_PlotVar4 = 0;

    mycount++;
    newCount++;

    float q1 = theta1motor;
    float q2 = theta2motor;
    float q3 = theta3motor;

    x = 20 * cos(q1) * sin( PI/4 + q2/2 - q3/2) * sin( PI/4 + q2/2 + q3/2);
    y = 20 * sin(q1) * sin(PI/4 + q2/2 - q3/2) * sin(PI/4 + q2/2 + q3/2);
    z = 10 * (1 + cos(q2) - sin(q3));
    
    invKinematics(x,y,z,thetas);
}

void printing(void){
    serial_printf(&SerialA, "%.2f %.2f %.2f :: %.2f %.2f %.2f :: %.2f %.2f %.2f \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI,x,y,z,
    thetas[0]*180/PI,thetas[1]*180/PI,thetas[2]*180/PI);
}