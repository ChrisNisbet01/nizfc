/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
#include "kalman.h"

/*
	getAngle():
	Given new readings from the accelerometer and gyro, return the estimated angle
	newAngle is in degrees, newRate is in degrees/sec, dt is the interval in seconds.
	returns the angle in degrees.
*/
static float getAngle(Kalman *p, float newAngle, float newRate, float dt)
{
	float temp;

	if ( p->run_once == 0 )
	{
		p->run_once = 1;
		p->angle = newAngle;
	}
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    p->rate = newRate - p->bias;
    p->angle += dt * p->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    p->P[0][0] += dt * (dt*p->P[1][1] - p->P[0][1] - p->P[1][0] + p->Q_angle);
	temp = dt * p->P[1][1];
    p->P[0][1] -= temp;
    p->P[1][0] -= temp;
    p->P[1][1] += p->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    p->S = p->P[0][0] + p->R_measure;
    /* Step 5 */
    p->K[0] = p->P[0][0] / p->S;
    p->K[1] = p->P[1][0] / p->S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    p->y = newAngle - p->angle;
    /* Step 6 */
    p->angle += p->K[0] * p->y;
    p->bias += p->K[1] * p->y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    p->P[0][0] -= p->K[0] * p->P[0][0];
    p->P[0][1] -= p->K[0] * p->P[0][1];
    p->P[1][0] -= p->K[1] * p->P[0][0];
    p->P[1][1] -= p->K[1] * p->P[0][1];

    return p->angle;
}

// Used to set angle, this should be set as the starting angle
static void setAngle(Kalman *p, float newAngle)
{
	p->angle = newAngle;
}

// Return the unbiased rate
static float getRate(Kalman *p)
{
	return p->rate;
}

/* These are used to tune the Kalman filter */
static void setQangle(Kalman *p, float newQ_angle)
{
	p->Q_angle = newQ_angle;
}

static void setQbias(Kalman *p, float newQ_bias)
{
	p->Q_bias = newQ_bias;
}

static void setRmeasure(Kalman *p, float newR_measure)
{
	p->R_measure = newR_measure;
}

static float getQangle(Kalman *p)
{
	return p->Q_angle;
}

static float getQbias(Kalman *p)
{
	return p->Q_bias;
}

static float getRmeasure(Kalman *p)
{
	return p->R_measure;
}

void KalmanInit( Kalman *p )
{
	p->run_once = 0;

	p->angle = 0.0f; 	// Reset the angle
	p->bias = 0.0f; 	// Reset bias

	p->P[0][0] = 0.0f; 	// Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	p->P[0][1] = 0.0f;
	p->P[1][0] = 0.0f;
	p->P[1][1] = 0.0f;

	p->getAngle = getAngle;
	p->setAngle = setAngle;
	p->getRate = getRate;
	p->setQangle = setQangle;
	p->setQbias = setQbias;
	p->setRmeasure = setRmeasure;
	p->getQangle = getQangle;
	p->getQbias = getQbias;
	p->getRmeasure = getRmeasure;

}


