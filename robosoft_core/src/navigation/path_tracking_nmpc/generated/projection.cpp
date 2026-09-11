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


#include "projection.h"
#include <math.h>

#define MAX(a,b) (((a) > (b))?a:b)
#define MIN(a,b) (((a) < (b))?a:b)


//FIXME set all these parameters as variables in the class!
/*more accurate, maybe a bit slower convergence... or not!*/
#define CORRECTION_LIMIT 1e-8
#define CORRECTION_STEP 1.0
#define PROJECTION_LIMIT 1e-6

/*#define C_PARAM 1.0*/
#define C_PARAM 10

Projection::Projection(NMPCAbstract *nmpcAbstract)
{

    this->nmpc = nmpcAbstract;

    nmpc->numControls;
    nmpc->numStates;
    nmpc->numSteps;

    int maxConstraints = nmpc->numSteps*nmpc->numControls;

    dG_duProj = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    N = new REAL_TYPE[nmpc->numSteps*nmpc->numSteps*nmpc->numControls*nmpc->numControls];
    T0= new REAL_TYPE[nmpc->numSteps*nmpc->numSteps*nmpc->numControls*nmpc->numControls];
    T1 = new REAL_TYPE[nmpc->numSteps*nmpc->numControls]; /*N'*n*/
    T2 = new REAL_TYPE[nmpc->numSteps*nmpc->numControls]; /*inv(N'*N)*N'*n*/
    T3 = new REAL_TYPE[nmpc->numSteps*nmpc->numSteps*nmpc->numControls*nmpc->numControls]; /*inv(N'*N)*N'*n*(inv(N'*N)*N'*n)'*/

    constraints = 0;

    constraintType = new int[maxConstraints];
    constraintIndexI = new int[maxConstraints];
    constraintIndexJ = new int[maxConstraints];

    xActiveConstraint = new int[nmpc->numSteps*nmpc->numStates];
    uActiveConstraint = new int[nmpc->numSteps*nmpc->numControls];
}

Projection::~Projection()
{
    delete [] dG_duProj;
    delete [] N;
    delete [] T0;
    delete [] T1;
    delete [] T2;
    delete [] T3;
    delete [] constraintType;
    delete [] constraintIndexI;
    delete [] constraintIndexJ;
    delete [] xActiveConstraint;
    delete [] uActiveConstraint;
}

void Projection::clearAllConstraints()
{
    int i,j;

    constraints = 0;

    //mark all constraint inactive!?
    for(i=0; i <nmpc->numSteps*nmpc->numControls; i++)
        uActiveConstraint[i] = 0;

    for(i=0; i <nmpc->numSteps*nmpc->numStates; i++)
        xActiveConstraint[i] = 0;
}

int Projection::addNormalConstraint(REAL_TYPE *n, int timeIndex)
{
    int i,j;

    double T4inv;
    REAL_TYPE T4; /*n'*n - (N'*n)'*inv(N'*N)*N'*n*/

    if(constraints >= nmpc->numSteps*nmpc->numControls)
        return 0;

    if(constraints == 0)
    {
        REAL_TYPE sum = 0.0;

        for(i = 0; i < (timeIndex+1)*nmpc->numControls; i++)
            sum += n[i]*n[i];

        /*(n should't ever be a zero vector!)*/
        if(sum < 1e-12)
            return 0;

        /*Normalize n*/
        sum = 1.0/sqrt(sum);
        for(i = 0; i < (timeIndex+1)*nmpc->numControls; i++)
            n[i]*=sum;

        T0[0] = 1.0;

    } else
    {
        REAL_TYPE len = 0.0;
        for(i = 0; i < (timeIndex+1)*nmpc->numControls; i++)
            len += n[i]*n[i];

        /*(n should't ever be a zero vector!)*/
        if(len < 1e-12)
            return 0;

        /*Normalize n*/
        len = 1.0/sqrt(len);
        for(i = 0; i < (timeIndex+1)*nmpc->numControls; i++)
            n[i]*=len;

        /*T1 = N'*n*/
        for(i = 0; i < constraints; i++)
        {
            REAL_TYPE sum = 0.0;

            /*FIXME min (timeIndex+1)*nmpc->numControls, (constraintIndexI[i]+1)*nmpc->numControls*/
        /*FIXME optimize also if N has control constraints*/

            for(j = 0; j < (timeIndex+1)*nmpc->numControls; j++)
                sum += N[i*nmpc->numSteps*nmpc->numControls + j]*n[j];

            T1[i] = sum;

            /*printf("T1[i] = %f\n", T1[i]);*/
        }

        /*T2 = T0 * T1*/
        for(i = 0; i < constraints; i++)
        {
            REAL_TYPE sum = 0.0;
            for(j = 0; j < constraints; j++)
                sum += T0[i*nmpc->numSteps*nmpc->numControls + j]*T1[j];

            T2[i] = sum;

            /*printf("T2[i] = %f\n", T2[i]);*/
        }

        /*T3 = T2' * T2*/
        for(i = 0; i < constraints; i++)
            for(j = 0; j < constraints; j++)
                T3[i*constraints + j] = T2[i]*T2[j];

        /*printf("T3[0] = %f\n", T3[0]);*/

        /*T4 = n'*n - T1'*T2*/
        /*T4 = 0.0;
        for(i = 0; i < (timeIndex+1)*nmpc->numControls; i++)
            T4 += n[i]*n[i];*/
        T4 = 1.0; /*n is normalized!*/

        for(i = 0; i < constraints; i++)
            T4 -= T1[i]*T2[i];

        /*printf("T4 = %f\n", T4);*/

        /*if the T4 is zero the new set of constraint is not linearly independent*/
        if(T4 < 1e-12)
            return 0;

        /*inverse*/
        T4inv = 1.0/T4;

        /*upper left corner T0 = T0 + T3/T4*/
        for(i = 0; i < constraints; i++)
            for(j = 0; j < constraints; j++)
                T0[i*nmpc->numSteps*nmpc->numControls + j] += T3[i*constraints + j] * T4inv;

        /*right T0 = -T2/T4 and low T0 = -T2'/T4*/
        for(i = 0; i < constraints; i++)
            T0[constraints*nmpc->numSteps*nmpc->numControls + i] = T0[i*nmpc->numSteps*nmpc->numControls + constraints] = -T2[i]*T4inv;

        /*right low T0 = 1/T4*/
        T0[constraints*nmpc->numSteps*nmpc->numControls + constraints] = T4inv;
    }

    /*add the new constraint to the list of constraints. The whole vector should be created (if all the places where N is used are not fixed)!*/
    for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
        N[nmpc->numSteps*nmpc->numControls*constraints + i] = n[i];

    return 1;
}


//optimized for a u constraint. Sign should be 1.0 or -1.0
int Projection::addUConstraint(int uIndex, int timeIndex, REAL_TYPE sign)
{
    int i,j;

    int nIndex = timeIndex*nmpc->numControls + uIndex;

    double T4inv;
    REAL_TYPE T4; /*n'*n - (N'*n)'*inv(N'*N)*N'*n*/

    if(constraints >= nmpc->numSteps*nmpc->numControls)
        return 0;

    if(constraints == 0)
    {
        T0[0] = 1.0;

    } else
    {
        //T1 = N'*n (FIXME doesn't need to be created)
        for(i = 0; i < constraints; i++)
        {
            T1[i] = sign*N[i*nmpc->numSteps*nmpc->numControls + nIndex];
        }

        //T2 = T0 * T1
        for(i = 0; i < constraints; i++)
        {
            REAL_TYPE sum = 0.0;
            for(j = 0; j < constraints; j++)
                sum += T0[i*nmpc->numSteps*nmpc->numControls + j]*T1[j];

            T2[i] = sum;
        }

        //T3 = T2 * T2' (?)
        for(i = 0; i < constraints; i++)
            for(j = 0; j < constraints; j++)
                T3[i*constraints + j] = T2[i]*T2[j];


        //T4 = n'*n - T1'*T2
        T4 = 1.0;

        for(i = 0; i < constraints; i++)
            T4 -= T1[i]*T2[i];

        /*if the T4 is zero the new set of constraint is not linearly independent*/
        if(T4 < 1e-12)
            return 0;

        /*inverse*/
        T4inv = 1.0/T4;

        /*upper left corner T0 = T0 + T3/T4*/
        for(i = 0; i < constraints; i++)
            for(j = 0; j < constraints; j++)
                T0[i*nmpc->numSteps*nmpc->numControls + j] += T3[i*constraints + j] * T4inv;

        /*right T0 = -T2/T4 and low T0 = -T2/T4'*/
        for(i = 0; i < constraints; i++)
            T0[constraints*nmpc->numSteps*nmpc->numControls + i] = T0[i*nmpc->numSteps*nmpc->numControls + constraints] = -T2[i]*T4inv;

        /*right low T0 = 1/T4*/
        T0[constraints*nmpc->numSteps*nmpc->numControls + constraints] = T4inv;
    }

    /*add the new constraint to the list of constraints. The whole vector should be created (if all the places where N is used are not fixed)!*/
    for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
        N[nmpc->numSteps*nmpc->numControls*constraints + i] = 0.0;

    N[nmpc->numSteps*nmpc->numControls*constraints + nIndex] = sign;

    return 1;
}


//calculates dG_du_proj = dG_du - N*T0*N'*dG_du and lagrange multipliers (T0*N'*dG_du), returns the norm of dG_duProj
REAL_TYPE Projection::calculateProjection(REAL_TYPE *dG_du, REAL_TYPE *lagrange)
{
    /*temporary FIXME should be defined elsewhere*/
    REAL_TYPE *temp1 = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];

    int i,j;
    REAL_TYPE norm = 0.0;

    //temp1 = N' * dG_du
    for(i = 0; i < constraints; i++)
    {
        REAL_TYPE sum = 0.0;

        if(constraintType[i] == 0)
        {
            for(j = 0; j < (constraintIndexI[i]+1)*nmpc->numControls; j++)
                sum += N[i*nmpc->numSteps*nmpc->numControls + j]*dG_du[j];
        } else if(constraintType[i] == 1)
        {
            //FIXME!
        } else
        {
            j = constraintIndexI[i]*nmpc->numControls + constraintIndexJ[i];
            sum += N[i*nmpc->numSteps*nmpc->numControls + j]*dG_du[j];
        }


        temp1[i] = sum;
    }

    //lagrange multipliers
    //lagrange = T0 * temp1
    for(i = 0; i < constraints; i++)
    {
        REAL_TYPE sum = 0.0;
        for(j = 0; j < constraints; j++)
            sum += T0[i*nmpc->numSteps*nmpc->numControls + j]*temp1[j];

        lagrange[i] = sum;
    }

    //dG_du_proj = dG_du - N * lagrange
    for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
    {
        REAL_TYPE sum = 0.0;
        for(j = 0; j < constraints; j++)
            sum += N[j*nmpc->numSteps*nmpc->numControls + i]*lagrange[j];

       this->dG_duProj[i] = dG_du[i] - sum;
    }

    //calculate the norm of the dG_du_proj
    norm = 0.0;
    for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
        norm += this->dG_duProj[i]*this->dG_duProj[i];

    norm = sqrt(norm);

    delete [] temp1;

    return norm;
}


void Projection::integrateAdjoint(REAL_TYPE *du, REAL_TYPE *lambda, int lastI)
{
    int i,j,k;

    //FIXME allocate elswhere!
    REAL_TYPE *lambdaTemp = new REAL_TYPE[nmpc->numStates];

    /*backwards in time! (start index OK?)*/
    for(i = lastI; i >= 0; i--)
    {
        /*calculate du*/
        for(j = 0; j < nmpc->numControls; j++)
        {
            /*du[i*nmpc->numControls + j] = 0.0;*/ /* should this be here?*/

            for(k = 0; k < nmpc->numStates; k++)
            {
                du[i*nmpc->numControls + j] += lambda[k] * nmpc->B_all[i*(nmpc->numStates*nmpc->numControls) + k*nmpc->numControls + j];
            }
        }

        for(j = 0; j < nmpc->numStates; j++)
        {
            lambdaTemp[j] = 0.0;

            for(k = 0; k < nmpc->numStates; k++)
            {
                lambdaTemp[j] += nmpc->A_all[i*(nmpc->numStates*nmpc->numStates) + k*nmpc->numStates + j] * lambda[k];
            }
        }

        for(j = 0; j < nmpc->numStates; j++)
            lambda[j] = lambdaTemp[j];
    }

    delete [] lambdaTemp;
}


//calculates du = N*T0*dx
void Projection::calculateCorrection(REAL_TYPE *du, REAL_TYPE *dx, REAL_TYPE *lagrange)
{
    int i,j;

    /*lagrange = T0 * dx*/
    for(i = 0; i < constraints; i++)
    {
        REAL_TYPE sum = 0.0;
        for(j = 0; j < constraints; j++)
            sum += T0[i*nmpc->numSteps*nmpc->numControls + j]*dx[j];

        lagrange[i] = sum;
    }

    /*du = N * lagrange*/
    for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
    {
        REAL_TYPE sum = 0.0;
        for(j = 0; j < constraints; j++)
            sum += N[j*nmpc->numSteps*nmpc->numControls + i]*lagrange[j];

       du[i] = sum;
    }
}

//Adds one constraint at a time, until no constraints are broken
//NOTE might not work if there are constraints that are linearily dependent!
int Projection::makeCorrection(REAL_TYPE *uNew, REAL_TYPE *xNew)
{
    int i,j,k,cont;

    //FIXME allocate elswhere
    REAL_TYPE *n = new REAL_TYPE[nmpc->numSteps*nmpc->numControls]; //new constraint vector
    REAL_TYPE *lagrange = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *correction = new REAL_TYPE [nmpc->numSteps*nmpc->numControls]; //how much should be corrected
    REAL_TYPE *uTemp = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *xTemp  = new REAL_TYPE[nmpc->numSteps*nmpc->numStates];

    REAL_TYPE *du = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *dx = new REAL_TYPE[nmpc->numSteps*nmpc->numStates];
    REAL_TYPE *lambda = new REAL_TYPE[nmpc->numStates];

    REAL_TYPE error, len, maxErr, maxCorr, maxDir;

    int maxI, maxJ, maxType;

    constraints = 0;

    /*uTemp = u, xTemp = x, clear xActiveConstraint and uActiveConstraint*/
    for(i=0; i <nmpc->numSteps*nmpc->numControls; i++) {
        uTemp[i] = nmpc->u[i];
        uActiveConstraint[i] = 0; }

    for(i=0; i <nmpc->numSteps*nmpc->numStates; i++) {
        xTemp[i] = nmpc->x[i + nmpc->numStates];
        xActiveConstraint[i] = 0; }



    /*Add allways the largest broken constraint*/
    cont = 1;

    while(cont)
    {
        maxErr = 0.0; /*set to a small negative if number near binding should be corrected*/

        /*Check which constraints are broken*/
        for(i = 0; i < nmpc->numSteps; i++)
        {
            for(j = 0; j < nmpc->numStates; j++)
            {
                if(xActiveConstraint[i*nmpc->numStates + j] == 0)
                {
                    /*compare to the projected x*/
                    if( (xTemp[i*nmpc->numStates + j] - nmpc->xMin[j]) <= -CORRECTION_LIMIT)
                    {
                        error = nmpc->xMin[j] - xTemp[i*nmpc->numStates + j];
                        if(error > maxErr)
                        {
                            maxErr = error;
                            maxCorr = nmpc->xMin[j] - nmpc->x[(i+1)*nmpc->numStates + j]; /*correction is made in respect to the original x*/
                            maxDir = -1.0;
                            maxI = i;
                            maxJ = j;
                            maxType = 0;
                        }
                    }

                    else if( (xTemp[i*nmpc->numStates + j] - nmpc->xMax[j]) >= CORRECTION_LIMIT)
                    {
                        error = xTemp[i*nmpc->numStates + j] - nmpc->xMax[j];
                        if(error > maxErr)
                        {
                            maxErr = error;
                            maxCorr = nmpc->x[(i+1)*nmpc->numStates + j] - nmpc->xMax[j]; /*correction is made in respect to the original x*/
                            maxDir = 1.0;
                            maxI = i;
                            maxJ = j;
                            maxType = 0;
                        }
                    }
                }
            }
        }

        for(i = 0; i < nmpc->numSteps; i++)
        {
            for(j = 0; j < nmpc->numControls; j++)
            {
                if(uActiveConstraint[i*nmpc->numControls + j] == 0)
                {
                    /*compare to the projected x*/
                    if( (uTemp[i*nmpc->numControls + j] - nmpc->uMin[j]) <= -CORRECTION_LIMIT)
                    {
                        error = nmpc->uMin[j] - uTemp[i*nmpc->numControls + j];
                        if(error > maxErr)
                        {
                            maxErr = error;
                            maxCorr = nmpc->uMin[j] - nmpc->u[i*nmpc->numControls + j]; /*correction is made in respect to the original u*/
                            maxDir = -1.0;
                            maxI = i;
                            maxJ = j;
                            maxType = 2;
                        }

            /*printf("u-constraint (%i,%i) broken by:%f\n", i,j,error);*/
                    }

                    else if( (uTemp[i*nmpc->numControls + j] - nmpc->uMax[j]) >= CORRECTION_LIMIT)
                    {
                        error = uTemp[i*nmpc->numControls + j] - nmpc->uMax[j];
                        if(error > maxErr)
                        {
                            maxErr = error;
                            maxCorr = nmpc->u[i*nmpc->numControls + j] -  nmpc->uMax[j] ; /*correction is made in respect to the original u*/
                            maxDir = 1.0;
                            maxI = i;
                            maxJ = j;
                            maxType = 2;
                        }

            /*printf("u-constraint (%i,%i) broken by:%f\n", i,j,error);*/
                    }
                }
            }
        }

        if(maxErr > 0.0)
        {
            /*printf("maxType %i, maxI %i, maxJ %i, maxCorr %f, maxDir %f\n", maxType, maxI, maxJ, maxCorr, maxDir);*/

            /*add a x-constraint using an adjoint value*/
            if(maxType == 0)
            {                
                for(k = 0; k < nmpc->numStates; k++)
                    lambda[k] = 0.0;

                /*lamda defines the sign of the constraint*/
                lambda[maxJ] = maxDir;

                /*set all n to zero (not required if all places where N is used are fixed?)*/
                for(k = 0; k < nmpc->numSteps*nmpc->numControls; k++)
                    n[k] = 0.0;

                /*calculate n*/
                integrateAdjoint(n, lambda, maxI);

                /*calculate the norm of n (This is because addNormalConstraint normalizes the vector!)*/
                len = 0.0;
                for(k = 0; k < (maxI+1)*nmpc->numControls; k++)
                    len += n[k]*n[k];
                len = sqrt(len);

                if(addNormalConstraint(n,maxI))
                {
                    constraintType[constraints] = 0;
                    constraintIndexI[constraints] = maxI;
                    constraintIndexJ[constraints] = maxJ;
                    correction[constraints] = -maxCorr / len; /*sign ok??*/
                    constraints++;
                }
                xActiveConstraint[maxI*nmpc->numStates + maxJ] = 1;
            }

            /*add u-constrain*/
            if(maxType == 2)
            {
                if(addUConstraint(maxJ, maxI, maxDir))
                {
                    constraintType[constraints] = 2;
                    constraintIndexI[constraints] = maxI;
                    constraintIndexJ[constraints] = maxJ;
                    correction[constraints] = -maxCorr; /*sign ok??*/
                    constraints++;
                }
                uActiveConstraint[maxI*nmpc->numControls + maxJ] = 1;
            }

            /*project and calculate uTemp and xTemp*/
            calculateCorrection(du, correction, lagrange);

            /*uTemp = u + du*/
            for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
                uTemp[i] = nmpc->u[i] + du[i];

            /*du -> dx*/
            nmpc->calculateDuToDx(dx, du);

            /*xTemp = x + dx*/
            for(i=0; i <nmpc->numSteps*nmpc->numStates; i++)
                xTemp[i] = nmpc->x[i+nmpc->numStates] + dx[i];

        } else
        {
            cont = 0;
        }

        /*TESTING, maximum of one constraint fixed...*/
        /*cont = 0;*/
    }

    /*FIXME remove unnesasery constraints. Start from the largest positive lagrange multiplier (how often does this happend?)*/


#ifdef DEBUG_PRINT
    printf(" broken %i", constraints);
#endif

    /*uNew = uTemp*/
    for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
        uNew[i] = uTemp[i];

    /*xNew = xTemp*/
    for(i = 0; i < nmpc->numStates; i++)
        xNew[i] = nmpc->x[i];
    for(i = 0; i < nmpc->numSteps*nmpc->numStates; i++)
        xNew[i+nmpc->numStates] = xTemp[i];

    delete [] du;
    delete [] dx;
    delete [] lambda;

    delete [] n;
    delete [] lagrange;
    delete [] correction;
    delete [] uTemp;
    delete [] xTemp;

    return constraints > 0;
}

//Faster implementation, switches the order constraints
void Projection::removeConstraint(int removeIndex)
{
    int i,j;
    REAL_TYPE T4;

    if(removeIndex >= constraints)
        return; /*ERROR!*/

    /*Mark the constraint removed*/
    if(constraintType[removeIndex] == 0)
        xActiveConstraint[constraintIndexI[removeIndex]*nmpc->numStates + constraintIndexJ[removeIndex]] = 0;
    /*else if(constraintType[removeIndex] == 1) FIXME*/
    else if(constraintType[removeIndex] == 2)
        uActiveConstraint[constraintIndexI[removeIndex]*nmpc->numControls + constraintIndexJ[removeIndex]] = 0;

    if(constraints == 1)
    {
        constraints = 0;
        return;
    }

    /*T4_new = 1/T0(removeIndex,removeIndex)*/
    T4 = 1.0/T0[removeIndex*nmpc->numSteps*nmpc->numControls + removeIndex];

    /*T2_new = -T0(removeIndex,1:end-1)*T4;*/
    for(i = 0; i < (constraints-1); i++)
        T2[i] = -T0[removeIndex*nmpc->numSteps*nmpc->numControls + i]*T4;

    /*T2(removeIndex) = -T(removeIndex,end)*T4;*/
    if(removeIndex < (constraints-1))
        T2[removeIndex] = -T0[removeIndex*nmpc->numSteps*nmpc->numControls + constraints-1]*T4; /*re-ordering...*/

    /*T3_new = T2_new' * T2_new*/
    for(i = 0; i < (constraints-1); i++)
        for(j = 0; j < (constraints-1); j++)
            T3[i*(constraints-1) + j] = T2[i]*T2[j];

    /*re-order the constraints so that last constraint and all data involved is moved in place of the removed one*/
    if(removeIndex < (constraints-1))
    {
        constraintIndexI[removeIndex] = constraintIndexI[constraints-1];
        constraintIndexJ[removeIndex] = constraintIndexJ[constraints-1];
        constraintType[removeIndex] = constraintType[constraints-1];
    }

    /*move the last row and colum. Move last value to the correct position*/
    if(removeIndex < (constraints-1))
    {
        for(i = 0; i < (constraints-1); i++)
        {
            T0[removeIndex*nmpc->numSteps*nmpc->numControls + i] =
            T0[i*nmpc->numSteps*nmpc->numControls + removeIndex] = T0[(constraints-1)*nmpc->numSteps*nmpc->numControls + i];
        }
        T0[removeIndex*nmpc->numSteps*nmpc->numControls + removeIndex] = T0[(constraints-1)*nmpc->numSteps*nmpc->numControls + (constraints-1)];
    }

    /*move n*/
    if(removeIndex < (constraints-1))
        for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
            N[nmpc->numSteps*nmpc->numControls*removeIndex + i] = N[nmpc->numSteps*nmpc->numControls*(constraints-1) + i];

    /*Calculate T0_new = T0 - T3*T4*/
    for(i = 0; i < (constraints-1); i++)
        for(j = 0; j < (constraints-1); j++)
            T0[i*nmpc->numSteps*nmpc->numControls + j] -= T3[i*(constraints-1) + j] / T4;

    constraints--;
}

//calculates the maximum step taking into account uActiveConstraint xActiveConstraint
REAL_TYPE Projection::calculateMaximumStep()
{
    //FIXME allocate elsewhere
    REAL_TYPE *dG_duTodx = new REAL_TYPE[nmpc->numSteps*nmpc->numStates];

    int i,j;

    double maxStep = 1e6;

    /*simulate how the dG_du_proj affects the dx*/
    nmpc->calculateDuToDx(dG_duTodx, this->dG_duProj);

    /*check how large step can be made regarding the u-constraints*/
    for(i = 0; i < nmpc->numSteps; i++)
    {
        for(j = 0; j < nmpc->numControls; j++)
        {
            if( !uActiveConstraint[i*nmpc->numControls + j] && this->dG_duProj[i*nmpc->numControls + j] < 0.0)
            {
                double temp = (nmpc->uMin[j] - nmpc->u[i*nmpc->numControls + j]) / this->dG_duProj[i*nmpc->numControls + j];
                maxStep = MIN(maxStep,temp);
            }

            if( !uActiveConstraint[i*nmpc->numControls + j] && this->dG_duProj[i*nmpc->numControls + j] > 0.0)
            {
                double temp = (nmpc->uMax[j] - nmpc->u[i*nmpc->numControls + j]) / this->dG_duProj[i*nmpc->numControls + j];
                maxStep = MIN(maxStep,temp);
            }
        }
    }

    /*check how large step can be made regarding the x-constraints*/
    for(i = 0; i < nmpc->numSteps; i++)
    {
        for(j = 0; j < nmpc->numStates; j++)
        {
            if( !xActiveConstraint[i*nmpc->numStates + j] && dG_duTodx[i*nmpc->numStates + j] < 0.0)
            {
                double temp = (nmpc->xMin[j] - nmpc->x[(i+1)*nmpc->numStates + j]) / dG_duTodx[i*nmpc->numStates + j];
                maxStep = MIN(maxStep,temp);
            }

            if( !xActiveConstraint[i*nmpc->numStates + j] && dG_duTodx[i*nmpc->numStates + j] > 0.0)
            {
                double temp = (nmpc->xMax[j] - nmpc->x[(i+1)*nmpc->numStates + j]) / dG_duTodx[i*nmpc->numStates + j];
                maxStep = MIN(maxStep,temp);
            }
        }
    }

    delete [] dG_duTodx;

    return maxStep;
}

//can be called after projectGradient if the direction has changed and some constraints might need to be removed
REAL_TYPE Projection::updateAndRemoveProjectGradient(REAL_TYPE *dG_du)
{
    int i,j,k, lMinIndex;

    //FIXME allocate elsewhere?
    REAL_TYPE *n = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *lagrange = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *lambda = new REAL_TYPE[nmpc->numStates];

    REAL_TYPE dG_duNorm, lMin;
    REAL_TYPE maxStep;

    /*Add state constraints*/
    for(i = 0; i < nmpc->numSteps; i++)
    {
        for(j = 0; j < nmpc->numStates; j++)
        {
            if(xActiveConstraint[i*nmpc->numStates + j] == 0)
            {
                REAL_TYPE xx = nmpc->x[(i+1)*nmpc->numStates + j];
                if( (xx <= (nmpc->xMin[j] + PROJECTION_LIMIT)) || (xx >= (nmpc->xMax[j] - PROJECTION_LIMIT)))
                {
                    /*add a x-constraint using an adjoint value*/                    
                    for(k = 0; k < nmpc->numStates; k++)
                        lambda[k] = 0.0;

                    /*lamda defines the sign of the constraint*/
                    if(xx >= (nmpc->xMax[j] - PROJECTION_LIMIT))
                        lambda[j] = 1.0;
                    else
                        lambda[j] = -1.0;

                    /*set all n to zero (not required if all places where N is used are fixed?)*/
                    for(k = 0; k < nmpc->numSteps*nmpc->numControls; k++)
                        n[k] = 0.0;

                    /*calculate n*/
                    integrateAdjoint(n, lambda, i);

                    if(addNormalConstraint(n,i))
                    {
                        constraintType[constraints] = 0;
                        constraintIndexI[constraints] = i;
                        constraintIndexJ[constraints] = j;
                        constraints++;
                    }

                    xActiveConstraint[i*nmpc->numStates + j] = 1;
                }
            }
        }

    }

    /*Add control constraints*/
    for(i = 0; i < nmpc->numSteps; i++)
    {
        for(j = 0; j < nmpc->numControls; j++)
        {
            if( uActiveConstraint[i*nmpc->numControls + j] == 0 )
            {
                REAL_TYPE uu = nmpc->u[i*nmpc->numControls + j];
                if( (uu <= (nmpc->uMin[j] + PROJECTION_LIMIT)) || (uu >= (nmpc->uMax[j] - PROJECTION_LIMIT)))
                {
                    /*sign of the constraint*/
                    REAL_TYPE sign;
                    if(uu >= (nmpc->uMax[j] - PROJECTION_LIMIT))
                        sign = 1.0;
                    else
                        sign = -1.0;

                    if(addUConstraint(j, i, sign))
                    {
                        constraintType[constraints] = 2;
                        constraintIndexI[constraints] = i;
                        constraintIndexJ[constraints] = j;
                        constraints++;
                    }

                    uActiveConstraint[i*nmpc->numControls + j] = 1;
                }
            }
        }

    }


    /*calculate actual projection (should work also with 0 constraints!)*/
    dG_duNorm = calculateProjection(dG_du, lagrange);

    /*find the smallest negative lagrange multiplier (lMinIndex = -1 if no negative multipliers)*/
    lMin = 0.0;
    lMinIndex = -1;
    for(i = 0; i < constraints; i++)
        if(lagrange[i] < lMin)
        {
            lMin = lagrange[i];
            lMinIndex = i;
        }

    /*compare the norm to the smallest negative multiplier*/
    if(lMinIndex >= 0 && dG_duNorm <= -lMin*C_PARAM)
    {
        /*remove one constraint*/
#ifdef DEBUG_PRINT
        printf("remove constraint: %i ", lMinIndex);
#endif
        removeConstraint(lMinIndex);

        /*calculate new projection*/
        dG_duNorm = calculateProjection(dG_du, lagrange);
    }

    maxStep = calculateMaximumStep();

    delete [] lambda;
    delete [] n;
    delete [] lagrange;

    return maxStep;
}

/*can be called after projectGradient if the direction or linearization has not changed*/
REAL_TYPE Projection::updateProjectGradient(REAL_TYPE *dG_du)
{
    int i,j,k;

    /*FIXME allocate elsewhere?*/
    REAL_TYPE *n = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *lagrange = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *lambda = new REAL_TYPE[nmpc->numStates];

    REAL_TYPE dG_duNorm;
    REAL_TYPE maxStep;

    /*Add state constraints*/
    for(i = 0; i < nmpc->numSteps; i++)
    {
        for(j = 0; j < nmpc->numStates; j++)
        {
            if(xActiveConstraint[i*nmpc->numStates + j] == 0)
            {
                REAL_TYPE xx = nmpc->x[(i+1)*nmpc->numStates + j];
                if( (xx <= (nmpc->xMin[j] + PROJECTION_LIMIT)) || (xx >= (nmpc->xMax[j] - PROJECTION_LIMIT)))
                {
                    /*add a x-constraint using an adjoint value*/                    
                    for(k = 0; k < nmpc->numStates; k++)
                        lambda[k] = 0.0;

                    /*lamda defines the sign of the constraint*/
                    if(xx >= (nmpc->xMax[j] - PROJECTION_LIMIT))
                        lambda[j] = 1.0;
                    else
                        lambda[j] = -1.0;

                    /*set all n to zero (not required if all places where N is used are fixed?)*/
                    for(k = 0; k < nmpc->numSteps*nmpc->numControls; k++)
                        n[k] = 0.0;

                    /*calculate n*/
                    integrateAdjoint(n, lambda, i);

                    if(addNormalConstraint(n,i))
                    {
                        constraintType[constraints] = 0;
                        constraintIndexI[constraints] = i;
                        constraintIndexJ[constraints] = j;
                        constraints++;
                    }

                    xActiveConstraint[i*nmpc->numStates + j] = 1;
                }
            }
        }

    }

    /*Add control constraints*/
    for(i = 0; i < nmpc->numSteps; i++)
    {
        for(j = 0; j < nmpc->numControls; j++)
        {
            if( uActiveConstraint[i*nmpc->numControls + j] == 0 )
            {
                REAL_TYPE uu = nmpc->u[i*nmpc->numControls + j];
                if( (uu <= (nmpc->uMin[j] + PROJECTION_LIMIT)) || (uu >= (nmpc->uMax[j] - PROJECTION_LIMIT)))
                {
                    /*sign of the constraint*/
                    REAL_TYPE sign;
                    if(uu >= (nmpc->uMax[j] - PROJECTION_LIMIT))
                        sign = 1.0;
                    else
                        sign = -1.0;

                    if(addUConstraint(j, i, sign))
                    {
                        constraintType[constraints] = 2;
                        constraintIndexI[constraints] = i;
                        constraintIndexJ[constraints] = j;
                        constraints++;
                    }

                    uActiveConstraint[i*nmpc->numControls + j] = 1;
                }
            }
        }

    }


    /*calculate actual projection (should work also with 0 constraints!)*/
    dG_duNorm = calculateProjection(dG_du, lagrange);

    maxStep = calculateMaximumStep();

    delete [] lambda;
    delete [] n;
    delete [] lagrange;

    return maxStep;
}
