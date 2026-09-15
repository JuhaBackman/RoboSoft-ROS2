/*
 *  This file is part of VIATOC
 *
 *  VIATOC is a software package designed for generating nonlinear
 *  model predictive controllers.
 *  Copyright 2013 Jouko Kalmari.
 *
 *  VIATOC is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  VIATOC is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with VIATOC. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef EKF_H
#define EKF_H


#define REAL_TYPE double

class EKFProblemAbstract
{
public:    

    virtual ~EKFProblemAbstract() {}

    //measurement prediction
    virtual void predictMeasurementAndDerivatives(REAL_TYPE *value, REAL_TYPE *H, REAL_TYPE *x_pred) = 0;

    //state prediction
    virtual void predictStateAndDerivatives(REAL_TYPE *x_pred, REAL_TYPE *A, REAL_TYPE *B) = 0;

    //runs the Kalman filter to estimate states
    void estimateStates();

    //variables
    int numStates, numLinearStates, numControls, numParameters;
    int numEKFStates, numEKFMeasurements;
    REAL_TYPE dt;

    //states, controls, measurments and parameters
    REAL_TYPE *x; // NUM_EKF_STATES
    REAL_TYPE *u; // NUM_CONTROLS
    REAL_TYPE *y; // NUM_EKF_MEASUREMENTS
    REAL_TYPE *p; // NUM_PARAMETERS

    //Kalman filter covariance matrices
    REAL_TYPE *Q; // NUM_EKF_STATES*NUM_EKF_STATES
    REAL_TYPE *R; // NUM_EKF_MEASUREMENTS*NUM_EKF_MEASUREMENTS
    REAL_TYPE *P; // NUM_EKF_STATES*NUM_EKF_STATES
};


#endif
