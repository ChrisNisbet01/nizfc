#ifndef __KALMAN_H__
#define __KALMAN_H__

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

typedef struct Kalman
{
    /* Kalman filter variables */
    float Q_angle; 		// Process noise variance for the accelerometer
    float Q_bias; 		// Process noise variance for the gyro bias
    float R_measure; 	// Measurement noise variance - this is actually the variance of the measurement noise

    float angle; 		// The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; 		// The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; 		// Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2];      // Error covariance matrix - This is a 2x2 matrix
    float K[2];         // Kalman gain - This is a 2x1 vector
    float y;            // Angle difference
    float S;            // Estimate error

	int run_once;		// Private variable. Non-zero after getAngle has run at least once after initialisation
	/*
		getAngle():
		Given new readings from the accelerometer and gyro, return the estimated angle
		newAngle is in degrees, newRate is in degrees/sec, dt is the interval in seconds.
		returns the angle in degrees.
	*/
    float (*getAngle)(struct Kalman *p, float newAngle, float newRate, float dt);

	// Used to set angle, this should be set as the starting angle
    void (*setAngle)(struct Kalman *p, float newAngle);

	// Return the unbiased rate
	float (*getRate)(struct Kalman *p);

	/* These are used to tune the Kalman filter */
	void (*setQangle)(struct Kalman *p, float newQ_angle);
	void (*setQbias)(struct Kalman *p, float newQ_bias) ;
	void (*setRmeasure)(struct Kalman *p, float newR_measure) ;
	float (*getQangle)(struct Kalman *p);
	float (*getQbias)(struct Kalman *p);
	float (*getRmeasure)(struct Kalman *p);

} Kalman;

void KalmanInit( Kalman *p );

#endif /* __KALMAN_H__ */
