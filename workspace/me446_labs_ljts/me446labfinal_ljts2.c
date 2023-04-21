#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81
#define num_points  7
#define d_speed     0.1

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

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

float thetaddot = 0;

float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;

float a2 = 0;
float a3 = 0;

float a0 = 0;
float a1 = 0;

float t = 0.0;
int home_traj = 0;

float Omega1 = 0;
float Omega2 = 0;
float Omega3 = 0;

float Viscous_positive1 = 0.22;
float Viscous_negative1 = 0.22;
float Coulomb_positive1 = 0.3637;
float Coulomb_negative1 = -0.2948;

float Viscous_positive2 = 0.2500;
float Viscous_negative2 = 0.287;
float Coulomb_positive2 = 0.45;
float Coulomb_negative2 = -0.47;

float Viscous_positive3 = 0.1922;
float Viscous_negative3 = 0.2132;
float Coulomb_positive3 = 0.440;
float Coulomb_negative3 = -0.440;

float u_fric1 = 0.0;
float u_fric2 = 0.0;
float u_fric3 = 0.0;

float v_co_1 = 3.6;
float v_co_2 = 3.6;
float v_co_3 = 3.3;

float p1 = 0.03;
float p2 = 0.0128;
float p3 = 0.0076;
float p4 = 0.0753;
float p5 = 0.0298;

float at2 = 0.0;
float at3 = 0.0;
float qd = 0;
float qddot = 0;
float qdddot = 0;

float sintheta1=0;
float sintheta32 = 0;
float costheta32 = 0;

//float kp1f = 300;
//float kp2f = 300;
//float kp3f = 160;
//float kd1f = 1.6;
//float kd2f = 1.9;
//float kd3f = 1.9;

float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;

float x = 0;
float y = 0;
float z = 0;

float KPx = 500.0;
float KPy = 500.0;
float KPz = 500.0;

float KDx = 10.0;
float KDy = 10.0;
float KDz = 10.0;

float x_dot = 0.0;
float y_dot = 0.0;
float z_dot = 0.0;

float xd_dot = 0.0;
float yd_dot = 0.0;
float zd_dot = 0.0;

// Initialize variables to keep track of old vals
float x_old = 0;
float x_dot_old = 0;
float x_dot_old2 = 0;

float y_old = 0;
float y_dot_old = 0;
float y_dot_old2 = 0;

float z_old = 0;
float z_dot_old = 0;
float z_dot_old2 = 0;

float xd = 0;
float yd = 0;
float zd = 0;

float Fx = 0.0;
float Fy = 0.0;
float Fz = 0.0;

float JF1 = 0.0;
float JF2 = 0.0;
float JF3 = 0.0;

float ff = 0.8;
float Kt = 6.0;
float FZcmd = 12.0; // Gravity comp constant
float JFZ1 = 0.0;
float JFZ2 = 0.0;
float JFZ3 = 0.0;

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

float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

float thetaz = 0;
float thetax = 0;
float thetay = 0;

float FxN= 0;
float FyN= 0;
float FzN= 0;

float xn = 0;
float yn = 0;
float zn = 0;

float xw = 0;
float yw = 0;
float zw = 0;
float xdw = 0;
float ydw = 0;
float zdw = 0;

float KPxn = 500.0;
float KPyn = 500.0;
float KPzn = 500.0;

float KDxn = 50.0;
float KDyn = 50.0;
float KDzn = 50.0;

float dirx = 0.0;
float diry = 0.0;
float dirz = 0.0;

float xa = 0.137;
float ya = 0.0;
float za = 0.422;

float xb = 0.4;
float yb = -0.1;
float zb = 0.3;


float speed = 0.15;
float t_start = 0.0;
float t_total = 0.0;
int state = 0; //0 is heading to b, 1 is heading to a
int i = 0; // Current point index
int end = 0;


typedef struct steptraj_s {
    long double b[5];
    long double a[5];
    long double xk[5];
    long double yk[5];
    float qd_old;
    float qddot_old;
    int size;
} steptraj_t;

typedef struct points {
    float xb;
    float yb;
    float zb;
    float thetaz;
    int mode;
} point;

// Mode 0: Normal speed, no weakening
// Mode 1: Waiting 1 second
// Mode 2: Weakening in x and y

point my_points [num_points] =
    {{0.137, 0.00, 0.422, 0.0, 0},
    {0.260, 0.00, 0.500, 0.0, 0}, // Zero DH Position
    {0.0289, 0.3498, 0.1890, 0.0, 0}, // Above hole
    {0.0289, 0.3497, 0.1350, 0.0, 2},// In hole
    {0.0289, 0.3497, 0.1350, 0.0, 1},// In hole
    {0.0289, 0.3498, 0.1890, 0.0, 0}, // Return to Above Hole
    {0.260, 0.00, 0.500, 0.0, 0} // Return to Zero DH Position
};

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


    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
//    //order matters here. Because we are saving the old value first before overriding it
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;
//
    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    //order matters here. Because we are saving the old value first before overriding it
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;
//
    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    //order matters here. Because we are saving the old value first before overriding it
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;


//    t = (mycount%2000)*0.001;
////    new trajectory
//    if((mycount%4000)==0) {
//        if(step1 > 0.26) {
//            step1 = 0.25;
//        } else {
//            step1 = 0.85;
//        }
//    }
//    if((mycount%4000)==0) {
//        if(step2 < 0.29) {
//            step2 = 0.3;
//        } else {
//            step2 = -0.3;
//        }
//    }


//    implement_discrete_tf(&trajectory, step1, &qd, &qddot, &qdddot);
//    theta1d = qd;
//    thetadot = qddot;
//    thetaddot = qdddot;
//    implement_discrete_tf(&trajectory2, step2, &qd, &qddot, &qdddot);
//    theta3d = qd;
//    theta3dot = qddot;
//    theta3ddot = qdddot;
//
//
//    err1 = theta1d-theta1motor;
//    err2 = theta1d-theta2motor;
//    err3 = theta3d-theta3motor;
//
//    err_dot1 = thetadot - Omega1;
//    err_dot2 = thetadot - Omega2;
//    err_dot3 = theta3dot - Omega3;
//
//    Ik1 = Ikold1+(err1-err_old1)/2.0*0.001;
//    Ik2 = Ikold2+(err2-err_old2)/2.0*0.001;
//    Ik3 = Ikold3+(err3-err_old3)/2.0*0.001;


    //friction
    if (Omega1 > 0.1) {
     u_fric1 = Viscous_positive1*Omega1 + Coulomb_positive1;
    } else if (Omega1 < -0.1) {
     u_fric1 = Viscous_negative1*Omega1 + Coulomb_negative1;
    } else {
     u_fric1 = v_co_1*Omega1;
    }

    if (Omega2 > 0.05) {
     u_fric2 = Viscous_positive2*Omega2 + Coulomb_positive2;
    } else if (Omega2 < -0.05) {
     u_fric2 = Viscous_negative2*Omega2 + Coulomb_negative2;
    } else {
     u_fric2 = v_co_2*Omega2;
    }

    if (Omega3 > 0.05) {
     u_fric3 = Viscous_positive3*Omega3 + Coulomb_positive3;
    } else if (Omega3 < -0.05) {
     u_fric3 = Viscous_negative3*Omega3 + Coulomb_negative3;
    } else {
     u_fric3 = v_co_3*Omega3;
    }

    //rotation to new frame
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

    // Final Project Trajectory Generation Following Point List
    t = mycount * 0.001;

    // Set parameters based on desired movement mode
    if (my_points[i+1].mode == 0) {  // Mode 0: Normal speed, no weakening
        speed = d_speed;
        KPx = 500.0;
        KPy = 500.0;
        KPz = 500.0;
        KDx = 10.0;
        KDy = 10.0;
        KDz = 10.0;
    } else if (my_points[i+1].mode == 1) {   // Mode 1: Waiting 1 second
        speed = 0.0;
        KPx = 500.0;
        KPy = 500.0;
        KPz = 500.0;
        KDx = 10.0;
        KDy = 10.0;
        KDz = 10.0;
    } else if (my_points[i+1].mode == 2) {   // Mode 2: Weakening in x and y
        speed = d_speed;
        KPx = 50.0;
        KPy = 50.0;
        KPz = 500.0;
        KDx = 1.0;
        KDy = 1.0;
        KDz = 10.0;
    }

    xa = my_points[i].xb;
    ya = my_points[i].yb;
    za = my_points[i].zb;
    xb = my_points[i+1].xb;
    yb = my_points[i+1].yb;
    zb = my_points[i+1].zb;

    dirx = xb - xa;
    diry = yb - ya;
    dirz = zb - za;
    if (my_points[i+1].mode == 1) {
        t_total = 5.0;
    } else {
        t_total = sqrt(dirx*dirx + diry*diry + dirz*dirz)/speed;
    }
    xdw = dirx * (t-t_start) / t_total + xa;
    ydw = diry * (t-t_start) / t_total + ya;
    zdw = dirz * (t-t_start) / t_total + za;

    if((t-t_start) > t_total) {
        if (i < num_points-1) {
            i = i + 1;
            t_start = t;
        } else {
            end = 1;
        }

    }

    //Task Space PD Control rotation
    xd = RT11*xdw+RT12*ydw+RT13*zdw;
    yd = RT21*xdw+RT22*ydw+RT23*zdw;
    zd = RT31*xdw+RT32*ydw+RT33*zdw;

    xw = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    yw= (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    zw = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;

    x = RT11*xw+RT12*yw+RT13*zw;
    y = RT21*xw+RT22*yw+RT23*zw;
    z = RT31*xw+RT32*yw+RT33*zw;

    x_dot = (x - x_old)/0.001;
    x_dot = (x_dot + x_dot_old + x_dot_old2)/3.0;
    x_old = x;
    x_dot_old2 = x_dot_old;
    x_dot_old = x_dot;

    y_dot = (y - y_old)/0.001;
    y_dot = (y_dot + y_dot_old + y_dot_old2)/3.0;
    y_old = y;
    y_dot_old2 = y_dot_old;
    y_dot_old = y_dot;

    z_dot = (z - z_old)/0.001;
    z_dot = (z_dot + z_dot_old + z_dot_old2)/3.0;
    z_old = z;
    z_dot_old2 = z_dot_old;
    z_dot_old = z_dot;

    Fx = KPxn*(xd-x) + KDxn*(xd_dot - x_dot);
    Fy = KPyn*(yd-y) + KDyn*(yd_dot - y_dot);
    Fz = KPzn*(zd-z) + KDzn*(zd_dot - z_dot);

    //rotation back to the world
    FxN= R11*Fx+R12*Fy+R13*Fz;
    FyN= R21*Fx+R22*Fy+R23*Fz;
    FzN= R31*Fx+R32*Fy+R33*Fz;

    JF1 = (-0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor)))*FxN
            + (0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor)))*FyN + 0*FzN;
    JF2 = (0.254*cos(theta1motor)*(cos(theta2motor)-sin(theta3motor)))*FxN
            + (0.254*sin(theta1motor)*(cos(theta2motor)-sin(theta3motor)))*FyN
            - 0.254*(cos(theta3motor)+sin(theta2motor))*FzN;
    JF3 = - 0.254*sin(theta3motor)*cos(theta1motor)*FxN - 0.254*sin(theta1motor)*sin(theta3motor)*FyN
            - 0.254*cos(theta3motor)*FzN;

    JFZ1 = 0*FZcmd/Kt;
    JFZ2 = -0.254*(cos(theta3motor)+sin(theta2motor))*FZcmd/Kt;
    JFZ3 = -0.254*cos(theta3motor)*FZcmd/Kt;

    if (end == 1) {
        *tau1 = JFZ1;
        *tau2 = -0.254*(cos(theta3motor)+sin(theta2motor))*FZcmd/Kt;
        *tau3 = -0.254*cos(theta3motor)*FZcmd/Kt;
    } else {
        *tau1 = JF1 + ff*u_fric1;
        *tau2 = JF2 + ff*u_fric2;
        *tau3 = JF3 + ff*u_fric3;
    }

    // When we want to plot error
    Simulink_PlotVar1 = xd-x;
    Simulink_PlotVar2 = yd-y;
    Simulink_PlotVar3 = zd-z;
//    Simulink_PlotVar4 = theta1d;

    // When we want to plot actual vs desired angles
//    Simulink_PlotVar1 = theta1motor;
//    Simulink_PlotVar2 = theta2motor;
//    Simulink_PlotVar3 = theta3motor;
//    Simulink_PlotVar4 = theta1d;

    mycount++;

}

void printing(void){
//    serial_printf(&SerialA, "%.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f  \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI, x,y,z, motortheta1*180/PI, motortheta2*180/PI, motortheta3*180/PI, theta1*180/PI, theta2*180/PI, theta3*180/PI);
//    serial_printf(&SerialA, "tau2: %.2f tau3: %.2f \n\r", ptau2, ptau3);
}

