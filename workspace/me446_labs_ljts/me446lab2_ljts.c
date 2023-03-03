#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.4454;
float offset_Enc3_rad = 0.2436;
//float offset_Enc2_rad = 0.0;
//float offset_Enc3_rad = 0.0;


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;
#pragma DATA_SECTION(toPrint, ".my_vars")
float toPrint = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];
#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];


long arrayindex = 0;
int UARTprint = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float motortheta1 = 0;
float motortheta2 = 0;
float motortheta3 = 0;

float theta1 = 0;
float theta2 = 0;
float theta3 = 0;

float x = 0;
float y = 0;
float z = 0;

// Initialize values for all 3 joint motors
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

float theta1d = 0;
float theta2d = 0;
float theta3d = 0;

float kp1 = 40;
float kp2 = 45;
float kp3 = 80;
float kd1 = 1.9;
float kd2 = 1.9;
float kd3 = 1.9;

float ptau1 = 0;
float ptau2 = 0;
float ptau3 = 0;

float err_old1 = 0;
float err_old2 = 0;
float err_old3 = 0;

float err1 = 0;
float err2 = 0;
float err3 = 0;

float Ikold1 = 0;
float Ikold2 = 0;
float Ikold3 = 0;

float Ik1 = 0;
float Ik2 = 0;
float Ik3 = 0;

float ki1 = 0;
float ki2 = 40000;
float ki3 = 10000;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

float threshold1 = 0.01;
float threshold2 = 0.03;
float threshold3 = 0.02;

float thetaddot = 0;

float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;

float a2 = 1.5;
float a3 = -1;

float t = 0.0;
int home_traj = 0;

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

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

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    // Print motor angles from encoders
    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    x = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    y = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    z = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;

    // Calculated motor thetas with inverse kinematics
    motortheta1 = atan2(y,x);
//    motortheta2 = 1.570796 - atan2((2.0*sqrt((0.064516 - 0.25*x*x - 0.25*y*y - 0.25*(z - 0.254)*(z - 0.254)))),sqrt(((z - 0.254)*(z - 0.254) + x*x + y*y)))*1.0;
    //motortheta2 = theta2motor;
    motortheta2 = 1.570796 - 1.0*atan2(1.0*z - 0.254, sqrt(x*x + y*y)) - 1.0*atan2(2.0*sqrt(0.064516 - 0.25*x*x - 0.25*y*y - 0.25*(z - 0.254)*(z-0.254)), sqrt((z - 0.254)*(z - 0.254) + x*x + y*y));
    motortheta3 = 1.0*atan2(2.0*sqrt(0.064516 - 0.25*x*x - 0.25*y*y - 0.25*(z - 0.254)*(z-0.254)), sqrt((z - 0.254)*(z-0.254) + x*x + y*y)) - 1.0*atan2(1.0*z - 0.254, sqrt(x*x + y*y));

    // DH Thetas
    theta1 = motortheta1;
    theta2 = motortheta2 - PI/2;
    theta3 = motortheta3 - motortheta2 + PI/2;



    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
    //order matters here. Because we are saving the old value first before overriding it
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    //order matters here. Because we are saving the old value first before overriding it
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    //order matters here. Because we are saving the old value first before overriding it
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;


//    t = mycount;

    if(mycount%2000 == 0) {
        if(home_traj == 1) {
            home_traj = 0;
        } else {
            home_traj = 1;
        }
    }

    t = mycount%2000 / 1000;
    if(home_traj) {
        theta1d = 0.0;
    } else {
        theta1d = a2*pow(t,2)+a3*pow(t,3);
    }


//    if((mycount%1000)==0) {
//        if(theta1d > 0.1) {
//            theta1d = a2*t^2+a3*t.^3;;
//        } else {
//            theta1d = PI/6;
//        }
//    }
//
//    if((mycount%1000)==0) {
//        if(theta2d > 0.1) {
//            theta2d = 0;
//        } else {
//            theta2d = PI/6;
//        }
//    }
//
//    if((mycount%1000)==0) {
//        if(theta3d > 0.1) {
//            theta3d = 0;
//        } else {
//            theta3d = PI/6;
//        }
//    }

    err1 = theta1d-theta1motor;
    err2 = theta1d-theta2motor;
    err3 = theta1d-theta3motor;

    Ik1 = Ikold1+(err1-err_old1)/2.0*0.001;
    Ik2 = Ikold2+(err2-err_old2)/2.0*0.001;
    Ik3 = Ikold3+(err3-err_old3)/2.0*0.001;

    thetaddot = 2*a2+6*a3*mycount/1000;
    // Case where
    if (fabs(err1) < threshold1) {
        ptau1 = kp1*(err1) - kd1*Omega1 + ki1*Ik1+thetaddot*J1;
    }  else {
        Ik1 = 0;
        ptau1 = kp1*(err1) - kd1*Omega1+thetaddot*J1;
    }

    if (fabs(err2) < threshold2) {
        ptau2 = kp2*(err2) - kd2*Omega2 + ki2*Ik2+thetaddot*J2;
    }  else {
        Ik2 = 0;
        ptau2 = kp2*(err2) - kd2*Omega2+thetaddot*J2;
    }

    if (fabs(err3) < threshold3) {
        ptau3 = kp3*(err3) - kd3*Omega3 + ki3*Ik3+thetaddot*J3;
    }  else {
        Ik3 = 0;
        ptau3 = kp3*(err3) - kd3*Omega3+thetaddot*J3;
    }

    // Saturation of torque values
    if (ptau1 > 5) {
        ptau1 = 5;
    } else if (ptau1 < -5) {
        ptau1 = -5;
    }

    if (ptau2 > 5) {
        ptau2 = 5;
    } else if (ptau2 < -5) {
        ptau2 = -5;
    }

    if (ptau3 > 5) {
        ptau3 = 5;
    } else if (ptau3 < -5) {
        ptau3 = -5;
    }

    *tau1 = ptau1;
    *tau2 = ptau2;
    *tau3 = ptau3;

    Ikold1 = Ik1;
    Ikold2 = Ik2;
    Ikold3 = Ik3;

    err_old1 = err1;
    err_old2 = err2;
    err_old3 = err3;

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = theta1d;

    mycount++;

}

void printing(void){
    serial_printf(&SerialA, "%.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f  \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI, x,y,z, motortheta1*180/PI, motortheta2*180/PI, motortheta3*180/PI, theta1*180/PI, theta2*180/PI, theta3*180/PI);
//    serial_printf(&SerialA, "tau2: %.2f tau3: %.2f \n\r", ptau2, ptau3);
}

