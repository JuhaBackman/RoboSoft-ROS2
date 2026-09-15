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

#include "ekf.h"
#include "math.h"
#include <iostream>
#include <limits>

/*m1 is an n*m matrix*/
/*m2 is an n*m matrix*/
/*m_out (= m1 + m2) is an n*m matrix (can be at same or at different memory location than m1 or m2)*/
void addMatrixMatrix(REAL_TYPE *m_out, REAL_TYPE *m1, REAL_TYPE *m2, int n, int m)
{
    int i,j;

    for(i = 0; i < n; i++)
        for(j = 0; j < m; j++)
        {
            m_out[i*m + j] = m1[i*m + j] + m2[i*m + j];
        }
}

/*m1 is an n*m matrix*/
/*m2 is an n*m matrix*/
/*m_out (= m1 - m2) is an n*m matrix (can be at same or at different memory location than m1 or m2)*/
void subsMatrixMatrix(REAL_TYPE *m_out, REAL_TYPE *m1, REAL_TYPE *m2, int n, int m)
{
    int i,j;

    for(i = 0; i < n; i++)
        for(j = 0; j < m; j++)
        {
            m_out[i*m + j] = m1[i*m + j] - m2[i*m + j];
        }
}

/*m1 is an n*m matrix*/
/*m2 is an m*p matrix*/
/*m_out (= m1*m2) is an n*p matrix (must be at a different memory location than m1 and m2!)*/
void multiplyMatrixMatrix(REAL_TYPE *m_out, REAL_TYPE *m1, REAL_TYPE *m2, int n, int m, int p)
{
    int i,j,k;

    for(i = 0; i < n; i++)
        for(j = 0; j < p; j++)
        {
            REAL_TYPE temp = 0.0;

            for(k = 0; k < m; k++)
                temp += m1[i*m + k]*m2[k*p + j];

            m_out[i*p + j] = temp;
        }
}

/*m1 is an n*m matrix*/
/*m2 is an p*m matrix*/
/*m_out (= m1*m2') is an n*p matrix (must be at a different memory location than m1 and m2!)*/
void multiplyMatrixMatrixT(REAL_TYPE *m_out, REAL_TYPE *m1, REAL_TYPE *m2, int n, int m, int p)
{
    int i,j,k;

    for(i = 0; i < n; i++)
        for(j = 0; j < p; j++)
        {
            REAL_TYPE temp = 0.0;

            for(k = 0; k < m; k++)
                temp += m1[i*m + k]*m2[j*m + k];

            m_out[i*p + j] = temp;
        }
}


/*m1 is n*m matrix*/
/*m2 is m*m symmetric matrix*/
/*m_out (= m1*m2*m1') is an n*n matrix*/
/*temp is n*m matrix*/
/*function will preserve symmetry and positive definiteness*/
void multiplySymmetricMatrixFromBothSides(REAL_TYPE *m_out, REAL_TYPE *m1, REAL_TYPE *m2, REAL_TYPE *temp, int n, int m)
{
   int i,j,k;
   
   /*temp = m1*m2*/
   multiplyMatrixMatrix(temp, m1, m2, n, m, m);

   /*calculate upper triangular matrix of temp*m1' and mirror it to m_out*/
   for(i = 0; i < n; i++)
        for(j = i; j < n; j++)
        {
            REAL_TYPE t = 0.0;

            for(k = 0; k < m; k++)
                t += temp[i*m + k]*m1[j*m + k];

            m_out[j*n + i] = m_out[i*n + j] = t;
        }
}

/*m1 is an m*n matrix*/
/*m2 is an m*m matrix*/
/*m_out (= m1'*inv(m2)) is an n*m matrix*/
/*temp is an m*(m+n) matrix*/
/*return value: 1 = ok. 0 = matrix is singular*/
int multiplyMatrixTMatrixInv(REAL_TYPE *m_out, REAL_TYPE *m1, REAL_TYPE *m2, REAL_TYPE *temp, int n, int m)
{
    int nm = n+m; /*n + m is total number of columns!*/
    int i,j,k;

    /*copy m2' into beginning of temp*/
    for(i = 0; i < m ; i++)
        for(j = 0; j < m ; j++)
            temp[i*nm + j] = m2[j*m + i];

    /*copy m1 into the end of temp*/
    for(i = 0; i < m ; i++)
        for(j = 0; j < n ; j++)
            temp[i*nm + m + j] = m1[i*n + j];

    /*convert temp to row-echelon form*/
    for(k = 0; k < m; k++)
    {
        /*find pivot*/
        double val_max = fabs(temp[k*nm + k]);
        int i_max = k;
        for(i = k+1; i < m; i++)
            if( fabs(temp[i*nm + k]) > val_max )
            {
                i_max = i;
                val_max = fabs(temp[i*nm + k]);
            }

        /*Error! Matrix is singular*/
        if(val_max == 0.0)
            return 0;

        /*pivot if necassery*/
        if(i_max != k)
            for(i = k; i < nm; i++)
            {
                double t = temp[k*nm + i];
                temp[k*nm + i] = temp[i_max*nm + i];
                temp[i_max*nm + i] = t;
            }

        /*combine rows*/
        for(i = k+1; i < m; i++)
        {
            double t = temp[i*nm + k] / temp[k*nm + k];

            for(j = k; j < nm; j++)
                temp[i*nm + j] = temp[i*nm + j] - temp[k*nm + j] * t;

            temp[i*nm + k] = 0;
        }
    }

    /*Back substitution*/
    for(k = m-1; k > 0; k--)
    {
        for(i = 0; i < k; i++)
        {
            double t = temp[i*nm + k] / temp[k*nm + k];

            for(j = k-1; j < nm; j++)
                temp[i*nm + j] = temp[i*nm + j] - temp[k*nm + j] * t;
        }
    }

    /*Scale so that begining of temp is a identity matrix*/
    for(k = 0; k < m; k++)
    {
        double t = 1.0 / temp[k*nm + k];

        for(j = m; j < nm; j++)
            temp[k*nm + j] *= t;

        /*temp[k*nm + k] = 1.0;*/ /*Not actually necassery!*/
    }

    /*copy end of temp as a m_out'*/
    for(i = 0; i < m ; i++)
        for(j = 0; j < n ; j++)
            m_out[j*m + i] = temp[i*nm + m + j];

    return 1;
}

void EKFProblemAbstract::estimateStates()
{    
    //F: sensitivity of state prediction in respect to previous state
    REAL_TYPE *F = new REAL_TYPE[numEKFStates*numEKFStates];

    //F: sensitivity of state prediction in respect to contol
    REAL_TYPE *B = new REAL_TYPE[numEKFStates*numControls];

    //state prediction
    REAL_TYPE *x_pred = new REAL_TYPE[numEKFStates];

    //state prediction covariance matrix
    REAL_TYPE *P_p = new REAL_TYPE[numEKFStates*numEKFStates];

    //predicted measurement and difference between prediction and measurement
    REAL_TYPE *y_pred = new REAL_TYPE[numEKFMeasurements];
    REAL_TYPE *y_err = new REAL_TYPE[numEKFMeasurements];

    //Kalman gain
    REAL_TYPE *K = new REAL_TYPE[numEKFStates*numEKFMeasurements];

    REAL_TYPE *S = new REAL_TYPE[numEKFMeasurements*numEKFMeasurements];

    //sensitivity of measurement respect to state
    REAL_TYPE *H = new REAL_TYPE[numEKFMeasurements*numEKFStates];


    //FIXME all of these temps are not required?! (note temp2 and temp4 are used simultaniously!)
    REAL_TYPE *temp1 = new REAL_TYPE[numEKFStates*numEKFStates];
    REAL_TYPE *temp2 = new REAL_TYPE[numEKFStates*numEKFMeasurements];
    REAL_TYPE *temp3 = new REAL_TYPE[numEKFMeasurements];
    REAL_TYPE *temp4 = new REAL_TYPE[numEKFMeasurements*(numEKFMeasurements+numEKFStates)];
    REAL_TYPE *temp5 = new REAL_TYPE[numEKFStates*numEKFStates];

    REAL_TYPE *I = new REAL_TYPE[numEKFStates*numEKFStates];
    for(int i = 0; i < numEKFStates*numEKFStates; i++)
        I[i] = 0.0;
    for(int i = 0; i < numEKFStates; i++)
        I[i + i*numEKFStates] = 1.0;


    REAL_TYPE *R_active = this->R;

    //Check what measurements can be used (aka active measurements)
    //Measurements with a Inf variances will be omitted
    int numActiveMeasurements = 0;
    int *activeMeasurementIndices = new int[numEKFMeasurements];
    for(int i = 0; i < numEKFMeasurements; i++)        
        if(R[i*(numEKFMeasurements+1)] <= std::numeric_limits<REAL_TYPE>::max())
        {
            activeMeasurementIndices[numActiveMeasurements] = i;
            numActiveMeasurements++;
        }


    //STATE PREDICTION
    //------------

    //calculate state prediction and derivatives
    predictStateAndDerivatives(x_pred, F, B);

    //P_p = F * P * F' + Q
    multiplySymmetricMatrixFromBothSides(P_p, F, this->P, temp1, numEKFStates, numEKFStates);
    addMatrixMatrix(P_p, P_p, this->Q, numEKFStates, numEKFStates);


    //STATE UPDATE
    //------------
    
    //calculate predicted measurement and its sensitivity
    predictMeasurementAndDerivatives(y_pred, H, x_pred);

    //y_err = y - y_pred
    subsMatrixMatrix(y_err, this->y, y_pred, numEKFMeasurements, 1);

    //remap y_pred, H and R to accomodate active measurments
    if(numActiveMeasurements < numEKFMeasurements)
    {
        R_active = new REAL_TYPE[numActiveMeasurements*numActiveMeasurements];

        for(int i = 0; i < numActiveMeasurements; i++)
        {
            int iOld = activeMeasurementIndices[i];
            y_err[i] = y_err[iOld];

            for(int j = 0; j < numEKFStates; j++)
            {
                H[i*numEKFStates + j] = H[iOld*numEKFStates + j];
            }

            for(int j = 0; j < numActiveMeasurements; j++)
            {
                int jOld = activeMeasurementIndices[j];
                R_active[i*numActiveMeasurements + j] = this->R[iOld*numEKFMeasurements + jOld];
            }
        }
    }

    //S = H*P_p*H' + R
    multiplySymmetricMatrixFromBothSides(S, H, P_p, temp2, numActiveMeasurements, numEKFStates);
    addMatrixMatrix(S, S, R_active, numActiveMeasurements, numActiveMeasurements);

    //K = P_p*H'*inv(S)
    //FIXME what if S is singular? (error or set K = 0?)
    multiplyMatrixTMatrixInv(temp2, H, S, temp4, numEKFStates, numActiveMeasurements); /*temp2 = H'*inv(S)*/
    multiplyMatrixMatrix(K, P_p, temp2, numEKFStates, numEKFStates, numActiveMeasurements);
        
    //new state estimate x = x_pred + K*y_err
    multiplyMatrixMatrix(temp1, K, y_err, numEKFStates, numActiveMeasurements, 1);
    addMatrixMatrix(this->x, x_pred, temp1, numEKFStates, 1);
   
#ifdef USE_JOSEPH_FORM
    //Joseph form covariance update P = (I - K*H)*P_p*(I - K*H)' + K*R*K', (numerically better)
    multiplyMatrixMatrix(temp1, K, H, numEKFStates, numActiveMeasurements, numEKFStates);
    subsMatrixMatrix(temp1, I, temp1, numEKFStates, numEKFStates);

    multiplySymmetricMatrixFromBothSides(this->P, temp1, P_p, temp5, numEKFStates, numEKFStates);
    multiplySymmetricMatrixFromBothSides(temp1, K, R_active, temp2, numEKFStates, numActiveMeasurements);

    addMatrixMatrix(this->P, this->P, temp1, numEKFStates, numEKFStates);

#else
    //state covariance estimate P = (I - K*H)*P_p (not a numerically good way)
    /*
    multiplyMatrixMatrix(temp1, K, H, numEKFStates, numActiveMeasurements, numEKFStates);
    subsMatrixMatrix(temp1, I, temp1, numEKFStates, numEKFStates);
    multiplyMatrixMatrix(this->P, temp1, P_p, numEKFStates, numEKFStates, numEKFStates);
    */

    //state covariance estimate P = P_p - K*S*K'
    multiplySymmetricMatrixFromBothSides(this->P, K, S, temp2, numEKFStates, numActiveMeasurements);
    subsMatrixMatrix(this->P, P_p, this->P, numEKFStates, numEKFStates);
#endif

    if(numActiveMeasurements < numEKFMeasurements)
    {
        delete [] R_active;
    }
    
    delete [] F;
    delete [] B;
    delete [] x_pred;
    delete [] P_p;
    delete [] y_pred;
    delete [] y_err;
    delete [] K;
    delete [] S;
    delete [] H;
    delete [] temp1;
    delete [] temp2;
    delete [] temp3;
    delete [] temp4;
    delete [] temp5;
    delete [] I;
    delete [] activeMeasurementIndices;
}

