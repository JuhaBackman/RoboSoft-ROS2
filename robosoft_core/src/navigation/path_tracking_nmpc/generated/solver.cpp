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

#include "solver.h"
#include "projection.h"

#include <iostream>

#include <math.h>

//FIXME all parameters should be in the class as variables!

#define LINEARIZATION_FREQUENCY 10
//#define USE_GOLDENSECTION 1

#define MAX_NEW_DIRECT_CONSTRAINTS 5
#define MAX_DIRECT_STEP 0.0001
#define LINE_SEARCH_ITERATIONS 16

//the line search is stopped when uncertainty smaller than this compared to step where the objective function still gets smaller
#define STOP_LINESEARCH 0.1


Solver::Solver(NMPCAbstract *nmpcAbstract)
{
    this->nmpc = nmpcAbstract;

    this->projection = new Projection(nmpcAbstract);

    this->dG_dx = new REAL_TYPE [nmpc->numSteps*nmpc->numStates];
    this->dG_du = new REAL_TYPE [nmpc->numSteps*nmpc->numControls];
}

Solver::~Solver()
{
    delete this->projection;
    delete [] this->dG_dx;
    delete [] this->dG_du;
}

double Solver::evalLinearObjectiveAt(REAL_TYPE k, REAL_TYPE *dx)
{
    //FIXME allocate elsewhere
    REAL_TYPE *uTemp = new REAL_TYPE [nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *xTemp = new REAL_TYPE [(nmpc->numSteps+1)*nmpc->numStates];

    REAL_TYPE objective, err;

    int i,j,h;

    /*first state cannot change*/
    for(i = 0; i < nmpc->numStates; i++)
        xTemp[i] = nmpc->x[i];

    for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
         uTemp[i] = nmpc->u[i] + k*projection->dG_duProj[i];

    for(i = 0; i < nmpc->numSteps*nmpc->numStates; i++)
        xTemp[nmpc->numStates + i] = nmpc->x[nmpc->numStates + i] + k*dx[i];

    objective = nmpc->calculateObjective(xTemp, uTemp);

    delete [] uTemp;
    delete [] xTemp;

    return objective;
}

void Solver::calculateDG_du(REAL_TYPE *dG_du, REAL_TYPE *dG_dx)
{
    int i,j,k;

    REAL_TYPE *lambda = new REAL_TYPE[nmpc->numStates];
    REAL_TYPE *lambdaTemp = new REAL_TYPE[nmpc->numStates];

    /*convert the jacobian to direction in u using adjoint values*/
    for(j = 0; j < nmpc->numStates; j++)
        lambda[j] = 0.0;

    /*backwards in time! FIXME validate this!*/
    for(i = nmpc->numSteps-1; i >= 0; i--)
    {
        /* New, this should be correct*/
        for(j = 0; j < nmpc->numStates; j++)
            lambda[j] += dG_dx[i*nmpc->numStates + j];

        for(j = 0; j < nmpc->numControls; j++)
        {
            for(k = 0; k < nmpc->numStates; k++)
            {
                dG_du[i*nmpc->numControls + j] += lambda[k] * nmpc->B_all[i*(nmpc->numStates*nmpc->numControls) + k*nmpc->numControls + j];
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

    delete [] lambda;
    delete [] lambdaTemp;
}

REAL_TYPE Solver::checkMaxStep(REAL_TYPE &objectiveEnd, REAL_TYPE maxStep, REAL_TYPE *dx)
{
    int i;

    REAL_TYPE dotProd = 0.0;

    REAL_TYPE *dG_dxTemp = new REAL_TYPE[ nmpc->numSteps*nmpc->numStates ];
    REAL_TYPE *dG_duTemp = new REAL_TYPE[ nmpc->numSteps*nmpc->numControls ];
    REAL_TYPE *uTemp = new REAL_TYPE[nmpc->numSteps*nmpc->numControls];
    REAL_TYPE *xTemp = new REAL_TYPE[(nmpc->numSteps+1)*nmpc->numStates];

    //calculate the dG_du at the end
    for(i = 0; i < nmpc->numStates; i++)
        xTemp[i] = nmpc->x[i];
    for(i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
        uTemp[i] = nmpc->u[i] + maxStep*projection->dG_duProj[i];
    for(i = 0; i < nmpc->numSteps*nmpc->numStates; i++)
        xTemp[nmpc->numStates + i] = nmpc->x[nmpc->numStates + i] + maxStep*dx[i];

    objectiveEnd = nmpc->calculateObjectiveAndDerivatives(dG_dxTemp, dG_duTemp, xTemp, uTemp);

    calculateDG_du(dG_duTemp, dG_dxTemp);

    //calculate the dot product between dG_duTemp and direction
    for(i=0; i < nmpc->numSteps*nmpc->numControls; i++)
        dotProd += dG_duTemp[i]*projection->dG_duProj[i];

    delete [] dG_dxTemp;
    delete [] dG_duTemp;
    delete [] uTemp;
    delete [] xTemp;

    return dotProd;
}

void Solver::optimize(int iterations)
{
    REAL_TYPE objective=0.0, objectiveEnd = 0.0;
    int iter;
    int i,j, correctionMade;

    REAL_TYPE *dx = new REAL_TYPE [nmpc->numSteps*nmpc->numStates];

    REAL_TYPE maxStep,a,b,c,d,aObjective,bObjective,cObjective,dObjective, bestObjective, bestK, omega, dotProd, stepMaxMonotone;

    for(iter = 0; iter < iterations; iter++)
    {
        if(iter % LINEARIZATION_FREQUENCY == 0)
        {

            //simulate system and calculate sensitivities
            nmpc->integrateSystemSensitivities();

            //correct if there are constraints that are broken (these should be small...)

#ifdef USE_RELAXATION
        projection->clearAllConstraints();
        makeCorrectionRelax(&nmpcVariables);

#else
            //also restarts the constraints
            correctionMade = projection->makeCorrection(nmpc->u, nmpc->x);
#endif

            //calculate jacobians
            objective = nmpc->calculateObjectiveAndDerivatives(this->dG_dx, this->dG_du);

            //calculate total jacobian dG_du
            calculateDG_du(this->dG_du, this->dG_dx);

        } else
        {
            /*Linear iteration*/
            objective = nmpc->calculateObjectiveAndDerivatives(this->dG_dx, this->dG_du);
            calculateDG_du(this->dG_du, this->dG_dx);
        }

#ifdef DEBUG_PRINT
        printf("objective: %f\n", objective);

        printf("iteration: %i ", iter);
#endif
        //adds new constraints and removes max one unnecessary constraint, projects the gradient
        maxStep =  projection->updateAndRemoveProjectGradient(this->dG_du);

        //if(iter % LINEARIZATION_FREQUENCY == 0)
        //    std::cout << "KKTNorm: " << getKKTNorm() << std::endl;

#ifdef DEBUG_PRINT
        printf("maxStep: %f", maxStep);
#endif

        //negative step shouldn't happen. But can for numerical reasons?
        if(maxStep <= 0.0)
        {
            /*
            printf("\n negative step: %f, cannot go further!\n", maxStep);
            return;
            */

            maxStep = 0.0;
        } else
        {

            //calculate how much x values change when u:s change
            nmpc->calculateDuToDx(dx, projection->dG_duProj);

            /*FIXME if MAX_NEW_DIRECT_CONSTRAINTS > 0*/

            //calculate if the maximum step should be taken
            dotProd = checkMaxStep(objectiveEnd, maxStep, dx);

            //if dot product is positive, then maxStep is taken
            if(dotProd > 0.0 && objectiveEnd < objective)
            {
                REAL_TYPE kSum = maxStep;
                int endIterations = 1, contin = 1;

                bestK = maxStep;
                bestObjective = objectiveEnd;

    #ifdef DEBUG_PRINT
                printf(" directly to end (maxStep: %f, dotProd: %f)", maxStep, dotProd);
    #endif
                //calculate u- and x-trajectories (linear)
                for(i = 0; i < nmpc->numSteps; i++)
                {
                    for(j = 0; j < nmpc->numControls; j++)
                    {
                        nmpc->u[i*nmpc->numControls + j] += bestK * projection->dG_duProj[i*nmpc->numControls + j];
                    }
                }

                for(i = 0; i < nmpc->numSteps*nmpc->numStates; i++)
                    nmpc->x[nmpc->numStates + i] += bestK * dx[i];

                //Try to do the same thing again...
                while(kSum < MAX_DIRECT_STEP && endIterations < MAX_NEW_DIRECT_CONSTRAINTS && contin)
                {
                    //new constraints are added, not removed!
                    maxStep = projection->updateProjectGradient(this->dG_du);

                    if(maxStep <= 0.0)
                    {                        
                        std::cout << "negative step, cannot go further! maxStep : " << maxStep << std::endl;
                        return;
                    }

                    //calculate how much x values change when u:s change
                    nmpc->calculateDuToDx(dx, projection->dG_duProj);

                    //calculate if the maximum step should be taken
                    dotProd = checkMaxStep(objectiveEnd, maxStep, dx);

    #ifdef DEBUG_PRINT
                printf(" (maxStep: %f, dotProd: %f, objective %f, objectiveEnd: %f)", maxStep, dotProd, objective, objectiveEnd);
    #endif

                    if(dotProd > 0.0 && objectiveEnd < objective)
                    {
                        kSum += maxStep;
                        endIterations++;

                        bestK = maxStep;
                        bestObjective = objectiveEnd;

    #ifdef DEBUG_PRINT
                        printf(" directly to end.");
    #endif
                        //calculate u- and x-trajectories (linear)
                        for(i = 0; i < nmpc->numSteps; i++)
                        {
                            for(j = 0; j < nmpc->numControls; j++)
                            {
                                nmpc->u[i*nmpc->numControls + j] += bestK * projection->dG_duProj[i*nmpc->numControls + j];
                            }
                        }

                        for(i = 0; i < nmpc->numSteps*nmpc->numStates; i++)
                            nmpc->x[nmpc->numStates + i] += bestK * dx[i];
                    } else
                    {
                        contin = 0;
                    }

                }

            } else
            {


                int iter2 = 0;
                int contin = 1;
                omega = 0.3820;

                stepMaxMonotone = 0; //stepTest is the largest step size d where the objective is smaller than at the beginning (FIXME does it work with Quadratic-fit?)

    #ifndef USE_GOLDENSECTION
                //Quadratic-fit line search
                a = 0.0;
                b = 0.5*maxStep;
                c = maxStep;

                aObjective = objective;
                bObjective = evalLinearObjectiveAt(b, dx);
                cObjective = evalLinearObjectiveAt(c, dx);


                for(;iter2 < LINE_SEARCH_ITERATIONS && contin; iter2++)
                {
                    /*upate stepTest to correct value*/
                    if(cObjective < objective && c > stepMaxMonotone)
                        stepMaxMonotone = c;

                    /*To fit a curve correctly, the point in the middle must be smaller than the endpoints*/
                    if(bObjective < aObjective && bObjective < cObjective)
                    {
                        REAL_TYPE temp = (a*(bObjective-cObjective) + b*(cObjective-aObjective) + c*(aObjective-bObjective));

                        if(temp == 0.0)
                            std::cout << " ERROR: Golden section: temp = 0!" << std::endl;

                        d = 0.5*(a*a*(bObjective-cObjective) + b*b*(cObjective-aObjective) + c*c*(aObjective-bObjective)) / temp;

                        /*d shouldn't ever be bigger than c*/
                        if(d > c)
                            d = c;

                        /*d shouldn't ever be smaller than a*/
                        if(d < a)
                            d = a;

                        dObjective = evalLinearObjectiveAt(d, dx);

                        if(d > b)
                        {
                            if(dObjective >= bObjective)
                            {
                                c = d; cObjective = dObjective;
                            } else
                            {
                                a = b; aObjective = bObjective;
                                b = d; bObjective = dObjective;
                            }

                        } else if (d < b)
                        {
                            if(dObjective >= bObjective)
                            {
                                a = d; aObjective = dObjective;
                            } else
                            {
                                c = b; cObjective = bObjective;
                                b = d; bObjective = dObjective;
                            }
                        } else /*d == b, quite rare?*/
                        {
                            /*Problem. Use derivative at d to decide!*/

                            /*Or, change to Golden section method*/
                            d = c; dObjective = cObjective;
                            b = (1.0 - omega)*a + omega*d; bObjective = evalLinearObjectiveAt(b, dx);
                            c = (1.0 - omega)*d + omega*a; cObjective = evalLinearObjectiveAt(c, dx);
                            contin = 0;
                        }

                    } else if(bObjective >= aObjective)
                    {
                        /*OK?*/
                        c = b; cObjective = bObjective;
                        b = (b+a)/2.0; bObjective = evalLinearObjectiveAt(b, dx);

                        d = c; dObjective = cObjective; /*Get the final comparison working...*/
                    } else /*bObjective >= cObjective && bObjective < aObjective*/
                    {
                        /*Is this ok, not allways...*/
                        /*
                        a = b; aObjective = bObjective;
                        b = 0.5*(c-a); bObjective = evalLinearObjectiveAt(b, dx);
                        */

                        /*Problem... continue with a Golden section search (OK?)*/
                        contin = 0;
                    }

                    /*stop the line search if the uncertainty is small enough*/
                    if((c-a) < STOP_LINESEARCH*stepMaxMonotone)
                        contin = 0;
                }

                /*Choose the smalles one of a,b,c*/
                bestObjective = aObjective; bestK = a;

                if( bObjective < bestObjective) {bestObjective = bObjective; bestK = b;}
                if( cObjective < bestObjective) {bestObjective = cObjective; bestK = c;}

                /*printf(" a %f, b %f, c %f, d %f\n", a, b, c, d);*/
    #endif

    #ifdef USE_GOLDENSECTION

                a = 0.0;
                b = omega*maxStep;
                c = (1.0 - omega)*maxStep;
                d = maxStep;

                aObjective = objective;
                bObjective = evalLinearObjectiveAt(b, dx);
                cObjective = evalLinearObjectiveAt(c, dx);
                dObjective = evalLinearObjectiveAt(d, dx);
#else

                /*calculate a,b,c and from quadratic fit*/
                d = c;
                b = (1.0 - omega)*a + omega*d;
                c = (1.0 - omega)*d + omega*a;

                bObjective = evalLinearObjectiveAt(b, dx);
                cObjective = evalLinearObjectiveAt(c, dx);
                dObjective = cObjective;
#endif
                /*Golden section search*/
                for(;iter2 < LINE_SEARCH_ITERATIONS && (d > a) && ((d-a) > STOP_LINESEARCH*stepMaxMonotone) ; iter2++)
                {
                    /*upate stepTest to correct value*/
                    if(dObjective < objective && d > stepMaxMonotone)
                        stepMaxMonotone = d;

                    if(bObjective < cObjective)
                    {
                        d = c; dObjective = cObjective;
                        c = b; cObjective = bObjective;
                        b = (1.0-omega)*b + omega*a;

                        bObjective = evalLinearObjectiveAt(b, dx);

                    } else
                    {
                        a = b; aObjective = bObjective;
                        b = c; bObjective = cObjective;
                        c = (1.0-omega)*c + omega*d;

                        cObjective = evalLinearObjectiveAt(c, dx);

                    }
                }

                bestObjective = aObjective; bestK = a;

                if( bObjective < bestObjective) {bestObjective = bObjective; bestK = b;}
                if( cObjective < bestObjective) {bestObjective = cObjective; bestK = c;}
                if( dObjective < bestObjective) {bestObjective = dObjective; bestK = d;}

                /*calculate u- and x-trajectories (linear)*/
                for(i = 0; i < nmpc->numSteps; i++)
                {
                    for(j = 0; j < nmpc->numControls; j++)
                    {
                        nmpc->u[i*nmpc->numControls + j] += bestK * projection->dG_duProj[i*nmpc->numControls + j];
                    }
                }

                for(i = 0; i < nmpc->numSteps*nmpc->numStates; i++)
                    nmpc->x[nmpc->numStates + i] += bestK * dx[i];
            }

    #ifdef DEBUG_PRINT
            bestObjective = evalLinearObjectiveAt(bestK, dx);
            printf(" actualStep: %f, (linear objective %f) ", bestK, bestObjective);
    #endif

        }

    }

    /*calculate new trajectory and calculate objective*/
    nmpc->integrateSystem();

    objective = nmpc->calculateObjective();

#ifdef DEBUG_PRINT
    printf("Final objective: %f\n", objective);
#endif

    delete [] dx;
}

//Norm of projected gradient should be same as "norm of KKT"
REAL_TYPE Solver::getKKTNorm()
{
    REAL_TYPE norm = 0.0;
    for(int i = 0; i < nmpc->numSteps*nmpc->numControls; i++)
        norm += projection->dG_duProj[i]*projection->dG_duProj[i];

    return sqrt(norm);
}
