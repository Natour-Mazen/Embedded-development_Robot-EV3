/*
 * workers.c
 *
 *  Created on: 26 janv. 2017
 *      Author: grolleau
 */
#include <ev3.h>
#include <ev3_port.h>
#include <ev3_tacho.h>
#include <ev3_sensor.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "myev3.h"


int min(int x,int y) {
	return x<y?x:y;
}

/**
 * Constants used for the deadreckoning
 */
#define DR_INTERSPACE 16.0
#define DR_PERIM 13.7

/**
 * Initializes the deadReckoning, call it once \
 * 	before calling deadRWorker
 */
void deadRWorkerInit() {
	set_tacho_command_inx(MY_LEFT_TACHO,TACHO_RESET);
	set_tacho_command_inx(MY_RIGHT_TACHO,TACHO_RESET);
	set_tacho_stop_action_inx (MY_LEFT_TACHO, TACHO_BRAKE);
	set_tacho_stop_action_inx (MY_RIGHT_TACHO, TACHO_BRAKE);
}

/**
 * On iteration of the deadReckoning
 * Requires: deadRWorkerInit has been called once before
 * \param[in] prevX previous value of x
 * \param[in] prevY previous value of y
 * \param[in] prevA previous value of alpha in RADIAN
 * \param[out] X new value of x
 * \param[out] Y new value of y
 * \param[out] A new value of alpha in RADIAN
 */
void deadRWorker(double prevX, double prevY, double prevA, double *X, double *Y, double* A) {
	static int prevRight=0;
	static int prevLeft=0;
	int tachoRight, tachoLeft;
	double distRight, distLeft;
	get_tacho_position(MY_LEFT_TACHO,&tachoRight);
	get_tacho_position(MY_RIGHT_TACHO,&tachoLeft);
	distRight=(DR_PERIM*(tachoRight-prevRight))/360.0;
	distLeft=(DR_PERIM*(tachoLeft-prevLeft))/360.0;
	prevRight=tachoRight;
	prevLeft=tachoLeft;
	if (fabs(distRight-distLeft)<1.0e-6) {
		*A=prevA;
		*X=prevX+cos(prevA)*distRight;
		*Y=prevY+sin(prevA)*distRight;
	} else {
	    double R = DR_INTERSPACE * (distLeft + distRight) / (2 * (distLeft-distRight));
	    double wd = (distLeft-distRight) / DR_INTERSPACE;
		*X=prevX+R*sin(wd+prevA)-R*sin(prevA);
		*Y=prevY-R*cos(wd+prevA)+R*cos(prevA);
		*A=prevA+wd;
		if (*A>M_PI)
			*A-=2*M_PI;
		else if (*A<-M_PI)
			*A+=2*M_PI;
	}
}

static int normalize(int value, int min, int max) {
	if (value>max) {
		return max;
	} else if (value<min) {
		return min;
	}
	return value;
}

// OLD function.
///*
// * Does one iteration going to (x,y) in a straight line
// * \param[in] x,y,a current position and heading of the robot, a in radian
// * \param[in] targetX,targetY coordinates of the target
// * \parma[in] power maximum power used
// * \return the error in cm between (x,y) and (targetX,targetY)
// */
//double deadreckoningGoTo(const double x, const double y, const double a, const double targetX, const double targetY, int power) {
//	double gamma; // Heading angle of (x,y) compared to actual position
//	const int gain=10;
//	const int gaina=286;
//	double error;
//	double errora;
//	double diffy=targetY-y;
//	double diffx=targetX-x;
//	gamma=atan2(diffy,diffx);
//	errora=gamma-a;
//	if (errora > M_PI) {
//		errora-=M_PI;
//	} else if (errora<=-M_PI) {
//		errora+=M_PI;
//	}
//	error=sqrt(diffy*diffy+diffx*diffx);
//	int cmdR,cmdL;
//	int cmd=normalize(gain*error,0,power);
//	cmdR=normalize(errora*gaina+cmd,-power,power);
//	cmdL=normalize(-errora*gaina+cmd,-power,power);
//	set_tacho_duty_cycle_sp(MY_RIGHT_TACHO,cmdR);
//	set_tacho_duty_cycle_sp(MY_LEFT_TACHO,cmdL);
//	set_tacho_command_inx(MY_RIGHT_TACHO,TACHO_RUN_DIRECT);
//	set_tacho_command_inx(MY_LEFT_TACHO,TACHO_RUN_DIRECT);
//	return error;
//}

/*
 * Does one iteration going to (x,y) in a straight line
 * \param[in] x,y,a current position and heading of the robot, a in radian
 * \param[in] targetX,targetY coordinates of the target
 * \parma[in] power maximum power used
 * \return the error in cm between (x,y) and (targetX,targetY)
 */
double deadreckoningGoTo(const double x, const double y, const double a, const double targetX, const double targetY, int power) {
    // Control constants
    const int gainDistance = 10;      // Proportional gain for distance.
    const int gainAngle = 286;        // Proportional gain for angle.
    const int minMovePower = 20;      // Minimum power to ensure movement.
    const double slowDownDistance = 1.0; // Distance at which to start slowing down.

    // Position difference calculation.
    double diffY = targetY - y;
    double diffX = targetX - x;

    // Target angle calculation relative to X axis.
    double targetAngle = atan2(diffY, diffX);

    // Angular error calculation (difference between target angle and current angle).
    double angleError = targetAngle - a;

    // Normalizing angular error between -PI and PI.
    if (angleError > M_PI) {
        angleError -= 2 * M_PI;
    } else if (angleError <= -M_PI) {
        angleError += 2 * M_PI;
    }

    // Distance to target calculation.
    double distanceError = sqrt(diffY * diffY + diffX * diffX);

    // Linear velocity command calculation based on distance.
    int linearCmd;
    if (distanceError < slowDownDistance) {
        // Gradually slow down when approaching the target.
        linearCmd = normalize(gainDistance * distanceError * (distanceError / slowDownDistance), minMovePower, power);
    } else {
        linearCmd = normalize(gainDistance * distanceError, minMovePower, power);
    }

    // If very close to the target, prioritize angular correction.
    if (distanceError < 5.0) {
        linearCmd = minMovePower;
    }

    // Motor commands calculation with angular correction.
    int rightCmd = normalize(angleError * gainAngle + linearCmd, -power, power);
    int leftCmd = normalize(-angleError * gainAngle + linearCmd, -power, power);

    // Ensure minimum power to overcome inertia and friction.
    if (rightCmd > 0 && rightCmd < minMovePower) rightCmd = minMovePower;
    else if (rightCmd < 0 && rightCmd > -minMovePower) rightCmd = -minMovePower;

    if (leftCmd > 0 && leftCmd < minMovePower) leftCmd = minMovePower;
    else if (leftCmd < 0 && leftCmd > -minMovePower) leftCmd = -minMovePower;

    // Send commands to motors.
    set_tacho_duty_cycle_sp(MY_RIGHT_TACHO, rightCmd);
    set_tacho_duty_cycle_sp(MY_LEFT_TACHO, leftCmd);
    set_tacho_command_inx(MY_RIGHT_TACHO, TACHO_RUN_DIRECT);
    set_tacho_command_inx(MY_LEFT_TACHO, TACHO_RUN_DIRECT);

    return distanceError;
}

