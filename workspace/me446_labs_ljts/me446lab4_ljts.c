#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.4454;
float offset_Enc3_rad = 0.2436;

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

// Encoder motor vars
float motortheta1 = 0;
float motortheta2 = 0;
float motortheta3 = 0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// Calculated acceleration of joint 1 
float thetaddot = 0;

// Time var
float t = 0.0;

// Angular Velocity Vars
float Omega1 = 0;
float Omega2 = 0;
float Omega3 = 0;

// Friction coefficients vars; tuned in lab 3 and carried over
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

// Friction coefficient vars; slope between lines used for low joint velocities 
float v_co_1 = 3.6;
float v_co_2 = 3.6;
float v_co_3 = 3.3;

// Friction comp control effort vars
float u_fric1 = 0.0;
float u_fric2 = 0.0;
float u_fric3 = 0.0;

// Outside loop control vars
float at2 = 0.0;
float at3 = 0.0;

// Desired trajectory vars (desired angular position, velocity, and acceleration respectively)
float qd = 0;
float qddot = 0;
float qdddot = 0;

// Trig placeholders vars used to decrease calculation time
float sintheta1 = 0;
float sintheta32 = 0;
float costheta32 = 0;

// Old theta and omega storage vars used for derivative control calculations
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;

// Placeholder vars for end effector position in task space
float x = 0;
float y = 0;
float z = 0;

//proportion control effort in task space base frame (before rotation)
float KPx = 500.0;
float KPy = 500.0;
float KPz = 500.0;

//derivative control effort in task space base frame (before rotation)
float KDx = 10.0;
float KDy = 10.0;
float KDz = 10.0;

// Task space velocity vars
float x_dot = 0.0;
float y_dot = 0.0;
float z_dot = 0.0;

// Task space desired velocity vars
float xd_dot = 0.0;
float yd_dot = 0.0;
float zd_dot = 0.0;

// Initialize task space variables to keep track of old vals
float x_old = 0;
float x_dot_old = 0;
float x_dot_old2 = 0;

float y_old = 0;
float y_dot_old = 0;
float y_dot_old2 = 0;

float z_old = 0;
float z_dot_old = 0;
float z_dot_old2 = 0;

//desired location in task space
float xd = 0;
float yd = 0;
float zd = 0;

// Calculated control effort in task space vars
float Fx = 0.0;
float Fy = 0.0;
float Fz = 0.0;

// Jacobian multiplied by the task space control effort 
float JF1 = 0.0;
float JF2 = 0.0;
float JF3 = 0.0;

// Friction multiplier var
float ff = 0.8; 
float Kt = 6.0;

// Applied force in z direction
float FZcmd = 12.0; 

// Jacobian multiplied by applied z force
float JFZ1 = 0.0;
float JFZ2 = 0.0;
float JFZ3 = 0.0;

//rotation matrix used to convert task space to a new frame
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

// Rotation trig values used to decrease calculation time
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

// Rotation about x,y, and z for task space control. These are set here
float thetaz = 0;
float thetax = 0;
float thetay = 0;

// Control effort in x,y, and z after rotation
float FxN= 0;
float FyN= 0;
float FzN= 0;

// Position in new coordinate frame
float xn = 0;
float yn = 0;
float zn = 0;

// position and desired position in world frame
float xw = 0;
float yw = 0;
float zw = 0;
float xdw = 0;
float ydw = 0;
float zdw = 0;

//proportional control gains in rotated n frame
float KPxn = 500.0;
float KPyn = 500.0;
float KPzn = 500.0;

//derivative control gains in rotated n frame
float KDxn = 50.0;
float KDyn = 50.0;
float KDzn = 50.0;

//straight line trajectory distance in x,y,z; used to determine how to move end effector
float dirx = 0.0;
float diry = 0.0;
float dirz = 0.0;

//point a of straight line trajectory
float xa = 0.2;
float ya = 0.1;
float za = 0.3;

//point b of straight line trajectory
float xb = 0.4;
float yb = -0.1;
float zb = 0.3;

//movement speed
float speed = 0.15; 
float t_start = 0.0;
float t_total = 0.0;
int state = 0; //0 is heading to b, 1 is heading to a


/*
    General Overview:
   
    This is our main lab function, which is called every 1ms and contains 
    all of our calculations and control laws inside of it. In short, it uses
    task space control to achieve a straight line trajectory, and is able to 
    do so with a coordinate frame rotation.
    
    After some setup functions, the main logic of our code starts with
    calculating the velocity of each joint, and saving the previous values.
    These old values are used to create a smoothed velocity to filter out noise.
    While the joint velocites are not needed for task space control, they are used
    for friction compensation calculations.
    
    Next, we implement friction compensation using the coefficients and logic from 
    lab 3. In summary, our friction comp control effort factors in viscous friction,
    which is dependent on joint speed and direction, and coulombic friction, which
    is only dependent on joint movement direction.
    
    
    Straight Line Trajectory Overview: 
    
    In preparation for rotated task space control, we first calculate our rotation matrix that 
    will convert from our new N frame to the world frame based off given theta x,y, and z values.
    
    For task space control, we start by calculating the current time 't' in seconds, 
    which we get from the timer variable mycount, which counts the number of calls to 
    the lab function, which is essentially the number of milliseconds elapsed. Based on the
    time, we determine if we need to change states and head to a new position. Since our 
    straight line trajectory calculates the expected time of the straight line movement
    t_total, we compare t-t_start to t_total to determine if we've reached our destination. 
    If so, we change states to make it so that the robot is now heading to the other point.
    
    The straight line trajectory aims to move the end effector between two points, which
    we've defined as a and b. Their coordinates are xa,ya,za and xb,yb,zb. Our desired 
    direction is the vector from a to b, which we get from subtracting a from b for each 
    coordinate direction. We calculate the total expected time by finding the euclidian 
    distance and dividing it by the sped with the formula: 
    
    t_total = sqrt(dirx*dirx + diry*diry + dirz*dirz)/speed;
    
    Next, we find the desired position along our line at time t with the equations: 
    
    xdw = dirx * (t-t_start) / t_total + xa;
    ydw = diry * (t-t_start) / t_total + ya;
    zdw = dirz * (t-t_start) / t_total + za;
    
    These equations multiply our direction vector by our proportion of time elapsed
    for the current straight line trajectory and add this to the starting point, which
    is point a for the trajectory to point b. We use the same equation for the reverse
    trajectory back to point a, except the starting point is point b.
    
    
    Task Space Control With Coordinate Frame Rotation Overview:
    
    In order to implement a coordinate frame rotation, each of our desired points in world 
    coordinates are multiplied by the transpose of the rotation matrix to get their value 
    in our new coordinate frame. The same thing is done for the world coordinates 
    of the end effector. 
    
    Next, we implement task space PD control. This requires calculating the velocity of the 
    end effector in x,y, and z, and saving these old values for our smoothed velocity calculation.
    These calculations are all done in our rotated coordinate frame. 
    
    We calculate a control effort using PD control in our rotated task space with the equations:
    
    FxN = KPxn*(xd-x) + KDxn*(xd_dot - x_dot);
    FyN = KPyn*(yd-y) + KDyn*(yd_dot - y_dot);
    FzN = KPzn*(zd-z) + KDzn*(zd_dot - z_dot);
    
    These equations used proportional and derivative gains that are specific to our rotated axes in 
    the N frame and error that is calculated in our rotated coordinate N frame.
    
    Finally, we rotate our control effort forces back to world coordinates, and they are mulptipled by 
    the jacobian's transpose to produce tau1, tau2, and tau3. Finally, we add friction compensation
    control efforts, which are scaled down with a friction factor to avoid bouncing behavior. 
    
    Input args:
        theta1motor : Encoder position of motor 1 (radians)
        theta2motor : Encoder position of motor 2 (radians)
        theta3motor : Encoder position of motor 3 (radians)
        *tau1       : Pointer used to set desired torque for motor 1
        *tau2       : Pointer used to set desired torque for motor 2
        *tau3       : Pointer used to set desired torque for motor 3
        error       : Unused variable
*/

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

    // Calculate angular velocites and save old values for smoothing
    Omega1 = (theta1motor - Theta1_old)/0.001;
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    Omega2 = (theta2motor - Theta2_old)/0.001;
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;

    Omega3 = (theta3motor - Theta3_old)/0.001;
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
    Theta3_old = theta3motor;
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;

    // Friction Compensation Joint 1
    if (Omega1 > 0.1) { // Positive velocity and outside low velocity threshld 
     u_fric1 = Viscous_positive1*Omega1 + Coulomb_positive1;
    } else if (Omega1 < -0.1) { // Negative velocity and outside low velocity threshold 
     u_fric1 = Viscous_negative1*Omega1 + Coulomb_negative1;
    } else { // Within low velocity threshold
     u_fric1 = v_co_1*Omega1;
    }

    // Friction Compensation Joint 2
    if (Omega2 > 0.05) {
     u_fric2 = Viscous_positive2*Omega2 + Coulomb_positive2;
    } else if (Omega2 < -0.05) {
     u_fric2 = Viscous_negative2*Omega2 + Coulomb_negative2;
    } else {
     u_fric2 = v_co_2*Omega2;
    }

    // Friction Compensation Joint 3
    if (Omega3 > 0.05) {
     u_fric3 = Viscous_positive3*Omega3 + Coulomb_positive3;
    } else if (Omega3 < -0.05) {
     u_fric3 = Viscous_negative3*Omega3 + Coulomb_negative3;
    } else {
     u_fric3 = v_co_3*Omega3;
    }

    // Rotation matrix
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


//    Task Space PD Control (unused)
//    This is commented out because it is no longer used, but is kept in case we need to refer to it later.
//    The Task Space PD Control with rotation is used instead.    
    
//    desired worldframe coordinates    
//    xd = 0.254; 
//    yd = 0.254;
//    zd = 0.254;
//
//    //calculating acutal world frame coordinates
//    x = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
//    y = (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
//    z = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;
//
//    //calculating speed
//    x_dot = (x - x_old)/0.001;
//    x_dot = (x_dot + x_dot_old + x_dot_old2)/3.0;
//    x_old = x;
//    x_dot_old2 = x_dot_old;
//    x_dot_old = x_dot;
//
//    y_dot = (y - y_old)/0.001;
//    y_dot = (y_dot + y_dot_old + y_dot_old2)/3.0;
//    y_old = y;
//    y_dot_old2 = y_dot_old;
//    y_dot_old = y_dot;
//
//    z_dot = (z - z_old)/0.001;
//    z_dot = (z_dot + z_dot_old + z_dot_old2)/3.0;
//    z_old = z;
//    z_dot_old2 = z_dot_old;
//    z_dot_old = z_dot;
//
//    Fx = KPx*(xd-x) + KDx*(xd_dot - x_dot);
//    Fy = KPy*(yd-y) + KDy*(yd_dot - y_dot);
//    Fz = KPz*(zd-z) + KDz*(zd_dot - z_dot);
//
//    //Jacobian multiply by the Fxyz
//    JF1 = (-0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor)))*Fx
//            + (0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor)))*Fy + 0*Fz;
//    JF2 = (0.254*cos(theta1motor)*(cos(theta2motor)-sin(theta3motor)))*Fx
//            + (0.254*sin(theta1motor)*(cos(theta2motor)-sin(theta3motor)))*Fy
//            - 0.254*(cos(theta3motor)+sin(theta2motor))*Fz;
//    JF3 = - 0.254*sin(theta3motor)*cos(theta1motor)*Fx - 0.254*sin(theta1motor)*sin(theta3motor)*Fy
//            - 0.254*cos(theta3motor)*Fz;

//    //Zcmd force
//    JFZ1 = 0*FZcmd/Kt;
//    JFZ2 = -0.254*(cos(theta3motor)+sin(theta2motor))*FZcmd/Kt;
//    JFZ3 = -0.254*cos(theta3motor)*FZcmd/Kt;


    /* Straight Line Trajectory
    
    t is calculated in seconds
    The first if statement switches between two states when one end of point is reached, 
    state zero goes from point a to point b
    state one goes from point b to point a
    
    */
    
    t = mycount * 0.001;
    // Starts a new line when reaching the goal
    if((t-t_start) > t_total) {
        if(state == 0) { // We got to point b
            state = 1; // Now heading to a
        } else{
            state = 0;
        }
        t_start = t;
    }
    
    // point a to point b
    if (state == 0) {
        //distance from b to a
        dirx = xb - xa;
        diry = yb - ya;
        dirz = zb - za;
        //total time it would take to travel from two points.
        //We use distance formula to find the distance and divide it by desired speed to find the total time
        t_total = sqrt(dirx*dirx + diry*diry + dirz*dirz)/speed;
        //current desired location, multiply by the average speed in each direction mulitply by the amount of time since state switch, and add the result to point a
        xdw = dirx * (t-t_start) / t_total + xa;
        ydw = diry * (t-t_start) / t_total + ya;
        zdw = dirz * (t-t_start) / t_total + za;
    // point b to point a
    } else if (state == 1) {
        //distance from a to b
        dirx = xa - xb;
        diry = ya - yb;
        dirz = za - zb;
        //total time it would take to travel from two points.
        //We use distance formula to find the distance and divide it by desired speed to find the total time
        t_total = sqrt(dirx*dirx + diry*diry + dirz*dirz)/speed;
        //current desired location, multiply by the average speed in each direction mulitply by the amount of time since state switch, and add the result to point b
        xdw = dirx * (t-t_start) / t_total + xb;
        ydw = diry * (t-t_start) / t_total + yb;
        zdw = dirz * (t-t_start) / t_total + zb;
    }


    /*
    Task Space PD Control rotation
    
    In this section, we calculate end effector position and desired position in our rotated
    coordinate frame, used PD control to determine control effort forces in our rotated coordinate 
    frame, and then convert this effort back to the world frame. We multiply cartesian forces 
    by the jacobian to determine the appropriate motor torques, and we add friction compensation 
    control effort to each joint to neglect the effects of friction.
    
    */
    
    // desired position in rotated frame (world coordinates multiplied by rotation matrix)
    xd = RT11*xdw+RT12*ydw+RT13*zdw;
    yd = RT21*xdw+RT22*ydw+RT23*zdw;
    zd = RT31*xdw+RT32*ydw+RT33*zdw;
    
    // Calculating acutal end effector position in world coordinates using forward kinematics
    // and known angles of each motor from encoder values
    xw = (127.0*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    yw= (127.0*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor)))/500.0;
    zw = (127.0*cos(theta2motor))/500.0 - (127.0*sin(theta3motor))/500.0 + 127.0/500.0;

    // position in rotated frame (acutal position in world coordinates multiplied by rotation matrix)
    x = RT11*xw+RT12*yw+RT13*zw;
    y = RT21*xw+RT22*yw+RT23*zw;
    z = RT31*xw+RT32*yw+RT33*zw;
    
    //calculating velocities in rotated frame and saving old ones in order to calculate averaged velocity
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
    
    // Control Effort in rotated N frame 
    FxN = KPxn*(xd-x) + KDxn*(xd_dot - x_dot);
    FyN = KPyn*(yd-y) + KDyn*(yd_dot - y_dot);
    FzN = KPzn*(zd-z) + KDzn*(zd_dot - z_dot);
    
    // Control effort after rotation back to the world frame 
    Fx= R11*FxN+R12*FyN+R13*FzN;
    Fy= R21*FxN+R22*FyN+R23*FzN;
    Fz= R31*FxN+R32*FyN+R33*FzN;

    // Multiplying cartesian forces by jacobian to get motor torques
    JF1 = (-0.254*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor)))*Fx
            + (0.254*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor)))*Fy + 0*Fz;
    JF2 = (0.254*cos(theta1motor)*(cos(theta2motor)-sin(theta3motor)))*Fx
            + (0.254*sin(theta1motor)*(cos(theta2motor)-sin(theta3motor)))*Fy
            - 0.254*(cos(theta3motor)+sin(theta2motor))*Fz;
    JF3 = - 0.254*sin(theta3motor)*cos(theta1motor)*Fx - 0.254*sin(theta1motor)*sin(theta3motor)*Fy
            - 0.254*cos(theta3motor)*Fz;

    // Desired torques are a combination of our Task Space PD Control and friction compensation
    *tau1 = JF1 + ff*u_fric1;
    *tau2 = JF2 + ff*u_fric2;
    *tau3 = JF3 + ff*u_fric3;

    // When we want to plot error
    Simulink_PlotVar1 = xd-x;
    Simulink_PlotVar2 = yd-y;
    Simulink_PlotVar3 = zd-z;
//    Simulink_PlotVar4 = theta1d;

    mycount++; // Keep track of times lab has been called
}

// Unused since we didn't need Tera Term for this lab
void printing(void){
//    serial_printf(&SerialA, "%.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f  \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI, x,y,z, motortheta1*180/PI, motortheta2*180/PI, motortheta3*180/PI, theta1*180/PI, theta2*180/PI, theta3*180/PI);
//    serial_printf(&SerialA, "tau2: %.2f tau3: %.2f \n\r", ptau2, ptau3);
}

