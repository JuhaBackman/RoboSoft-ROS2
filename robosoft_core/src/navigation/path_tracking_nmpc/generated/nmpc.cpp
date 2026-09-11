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

#include "nmpc.h"
#include "solver.h"
#include <cmath>

//shifts controls and states so that u(N-1) = u(N-2) ... u(1) = u(2), u(0) = u(1). etc
void NMPCAbstract::shiftControlsStates()
{
   int i,j;
   for(i=0; i < numSteps-1; i++)
   {
      for(j=0; j < numControls; j++)
         this->u[numControls*i + j] = this->u[numControls*(i+1) + j];

      for(j=0; j < numStates; j++)
         this->x[numStates*i + j] = this->x[numStates*(i+1) + j];
   }
}


void NMPCAbstract::optimize(int iters)
{
    solver->optimize(iters);
}

REAL_TYPE NMPCAbstract::calculateObjective()
{
    return calculateObjective(this->x, this->u);
}

REAL_TYPE NMPCAbstract::calculateObjectiveAndDerivatives(REAL_TYPE *dG_dx, REAL_TYPE *dG_du)
{
    return calculateObjectiveAndDerivatives(dG_dx, dG_du, this->x, this->u);
}

void NMPCAbstract::integrateSystem()
{
    integrateSystem(this->x, this->u);
}

//assumes that the A and B matrices are calculated! dx[0] is omitted (assumed zero)
void NMPCAbstract::calculateDuToDx(REAL_TYPE *dx, REAL_TYPE *du)
{
    int i,j,k;
    /*actual dx[0] is zero! -> dx["0"] = B[0]*du[0]*/
    for(j=0; j < this->numStates; j++)
    {
        dx[j] = 0.0;
        for(k=0; k < this->numControls; k++)
        {
            dx[j] += this->B_all[j*this->numControls + k] * du[k];
        }
    }

    /*rest*/
    /*dx["i"] = A[i]*dx["i-1"] + B[i]*du[j]*/
    for(i=1; i < this->numSteps; i++)
    {
        for(j=0; j < this->numStates; j++)
        {
            dx[i*this->numStates + j] = 0.0;

            for(k=0; k < this->numStates; k++)
            {
                dx[i*this->numStates + j] += this->A_all[i*(this->numStates*this->numStates) + j*this->numStates + k] * dx[(i-1)*this->numStates + k];
            }

            for(k=0; k < this->numControls; k++)
            {
                dx[i*this->numStates + j] += this->B_all[i*(this->numStates*this->numControls) + j*this->numControls + k] * du[i*this->numControls + k];
            }
        }
    }
}

// L*U = A(p,:) 
// FIXME: L and U (and even A) could actually fit the same matrix!
int NMPCAbstract::calculateLUPFactorization(REAL_TYPE *A, REAL_TYPE *L, REAL_TYPE *U, int *p, int n)
{
    int i,j,k;

    /*U = A*/
    for(i = 0; i < n*n; i++)
        U[i] = A[i];

    /*L = eye(N) (FIXME initializing L should not be needed)*/
    for(i=0; i < n*n; i++)
        L[i] = 0.0;
    for(i=0; i < n; i++)
        L[i*(n+1)] = 1.0;

    /*p = 1:N*/
    for(i=0; i < n; i++)
        p[i] = i;

    /*start editing column by column*/
    for(k = 0; k < n-1; k++)
    {
        /*find pivot*/
        double val_max = fabs(U[k*n + k]);
        int i_max = k;
        for(i = k+1; i < n; i++)
            if( fabs(U[i*n + k]) > val_max )
            {
                i_max = i;
                val_max = fabs(U[i*n + k]);
            }

        /*Error! Matrix is singular*/
        if(val_max == 0.0)
            return 0;

        /*pivot if necassery*/
        if(i_max != k)
        {
            /*swap p*/
            int temp = p[k];
            p[k] = p[i_max];
            p[i_max] = temp;

            /*swap U rows*/
            for(i = k; i < n; i++)
            {
                double temp = U[k*n + i];
                U[k*n + i]  = U[i_max*n + i];
                U[i_max*n + i] = temp;
            }
        }

        /*update L (note the sign!)*/
        for(i = k+1; i < n; i++)
            L[i*n + k] = U[i*n + k]/U[k*n + k];

        /*update U*/
        for(i = k+1; i < n; i++)
        {
            /*first elements are set to zero*/
            U[i*n + k] = 0.0;

            for(j = k+1; j < n; j++)
                U[i*n + j] -= L[i*n + k]*U[k*n + j];
        }

        /*pivot L if necassery*/
        if(i_max != k)
        {
            /*Note that only beginning is pivoted!*/
            for(i = 0; i < k; i++)
            {
                double temp = L[k*n + i];
                L[k*n + i] = L[i_max*n + i];
                L[i_max*n + i] = temp;
            }
        }
    }

    /*Ok!*/
    return 1;
}

//solves X from : L*U*X = B(p). X, Y and B are matrices of size (nxm)
//L should have ones in the diagonal
int NMPCAbstract::solveAXBMatrix(REAL_TYPE *X, REAL_TYPE *L, REAL_TYPE *U, REAL_TYPE *B, int *p, REAL_TYPE *Y, int n, int m)
{
    int i,j,k;
    double invU;

    /*calculate Y (L*Y = B(p,:))*/
    for(k = 0; k < m; k++)
       Y[k] = B[p[0]*m + k];

    for(i = 1; i < n; i++)
    {
	for(k = 0; k < m; k++)	
	   Y[i*m + k] = B[p[i]*m + k];

        for(j = 0; j < i; j++)
           for(k = 0; k < m; k++)
               Y[i*m + k] -= L[i*n + j] * Y[j*m + k];
    }

    /*calculate x (Ux = y)*/
    invU = 1.0/U[n*n-1];

    for(k = 0; k < m; k++)
       X[(n-1)*m+k] = Y[(n-1)*m+k]*invU; 

    for(i = n-2; i >= 0; i--)
    {
        invU = 1.0/U[i*n + i];

        for(k = 0; k < m; k++)
        {
           double temp = Y[i*m + k];
           for(j = i+1; j < n; j++)
              temp -= U[i*n + j]*X[j*m + k];
           X[i*m + k] = temp*invU;
        }
    }    

    return 1;
}

//solves x from : L*U*x = b(p), y should be a vector of the same size as x (nx1)
// should have ones in the diagonal
int NMPCAbstract::solveAxb(REAL_TYPE *x, REAL_TYPE *L, REAL_TYPE *U, REAL_TYPE *b, int *p, REAL_TYPE *y, int n)
{
    int i,j;

    /*calculate y (Ly = Pb)*/
    y[0] = b[p[0]];

    for(i = 1; i < n; i++)
    {
        y[i] = b[p[i]];
        for(j = 0; j < i; j++)
           y[i] -= L[i*n + j] * y[j];
    }

    /*calculate x (Ux = y)*/
    x[n-1] = y[n-1] / U[n*n-1];
    for(i = n-2; i >= 0; i--)
    {
        double temp = y[i];
        for(j = i+1; j < n; j++)
            temp -= U[i*n + j]*x[j];
        x[i] = temp/U[i*n + i];
    }   

    return 1;
}

