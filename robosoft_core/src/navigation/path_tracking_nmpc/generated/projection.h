/*
 *  This file is part of VIATOC
 *
 *  VIATOC is a software package designed for generating nonlinear
 *  model predictive controllers.
 *  Copyright 2013-2014 Jouko Kalmari.
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

#ifndef PROJECTION_H
#define PROJECTION_H

#include "nmpc.h"

class Projection
{
public:

    Projection(NMPCAbstract *nmpcAbstract);    
    ~Projection();

    void clearAllConstraints();
    int makeCorrection(REAL_TYPE *uNew, REAL_TYPE *xNew);
    REAL_TYPE updateAndRemoveProjectGradient(REAL_TYPE *dG_du);
    REAL_TYPE updateProjectGradient(REAL_TYPE *dG_du);

    //projected gradient
    REAL_TYPE *dG_duProj;

private:

    int addNormalConstraint(REAL_TYPE *n, int timeIndex);
    int addUConstraint(int uIndex, int timeIndex, REAL_TYPE sign);
    REAL_TYPE calculateProjection(REAL_TYPE *dG_du, REAL_TYPE *lagrange);
    void integrateAdjoint(REAL_TYPE *du, REAL_TYPE *lambda, int lastI);
    void calculateCorrection(REAL_TYPE *du, REAL_TYPE *dx, REAL_TYPE *lagrange);
    void removeConstraint(int removeIndex);
    REAL_TYPE calculateMaximumStep();

    NMPCAbstract *nmpc;

    //vectors of active constraints (actually a transpose of N)
    REAL_TYPE *N;

    REAL_TYPE *T0; //inv(N'*N)

    //temporary
    REAL_TYPE *T1; //N'*n
    REAL_TYPE *T2; //inv(N'*N)*N'*n
    REAL_TYPE *T3; //inv(N'*N)*N'*n*(inv(N'*N)*N'*n)'

    //number of constraints
    int constraints;

    /*
    There are three types of constraints
    0: Normal:
    1: Sparse:  One control is constrainted (FIXME not implemented!)
    2: Control: One control at a one time instant is constrainted
    */
    int *constraintType; //[MAX_CONSTRAINTS];

    /*last time index that the constraint affects*/
    int *constraintIndexI; //[MAX_CONSTRAINTS];
    /*which control index the constraint affects*/
    int *constraintIndexJ; //[MAX_CONSTRAINTS];

    /*which constraints are currently active*/
    int *xActiveConstraint; //[NUM_STEPS*NUM_STATES];
    int *uActiveConstraint; //[NUM_STEPS*NUM_CONTROLS];

};

#endif
