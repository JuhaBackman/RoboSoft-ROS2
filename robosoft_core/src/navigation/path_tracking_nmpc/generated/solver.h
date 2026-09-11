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

#ifndef SOLVER_H
#define SOLVER_H

#include "nmpc.h"
#include "projection.h"

class Solver
{

public:

    Solver(NMPCAbstract *nmpcAbstract);
    ~Solver();

    void optimize(int iters);

    REAL_TYPE getKKTNorm();


private:

    double evalLinearObjectiveAt(REAL_TYPE k, REAL_TYPE *dx);
    void calculateDG_du(REAL_TYPE *dG_du, REAL_TYPE *dG_dx);
    REAL_TYPE checkMaxStep(REAL_TYPE &objectiveEnd, REAL_TYPE maxStep, REAL_TYPE *dx);

    NMPCAbstract *nmpc;
    Projection *projection;

    REAL_TYPE *dG_dx; // numSteps*numStates
    REAL_TYPE *dG_du; // numSteps*numControls

};

#endif
