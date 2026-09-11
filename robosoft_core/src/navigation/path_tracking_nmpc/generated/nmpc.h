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

#ifndef NMPC_H
#define NMPC_H

//forward declaration of solver
class Solver;

#define REAL_TYPE double

class NMPCAbstract
{

enum SolverType {NORMAL, BARZILAI_BORWAIN};

public:    

    virtual ~NMPCAbstract() {}
 
    //calculate objective function value
    REAL_TYPE calculateObjective();
    REAL_TYPE calculateObjectiveAndDerivatives(REAL_TYPE *dG_dx, REAL_TYPE *dG_du);

    virtual REAL_TYPE calculateObjective(REAL_TYPE *x, REAL_TYPE *u) = 0;
    virtual REAL_TYPE calculateObjectiveAndDerivatives(REAL_TYPE *dG_dx, REAL_TYPE *dG_du, REAL_TYPE *x, REAL_TYPE *u) = 0;

    //integrate system
    void integrateSystem();
    virtual void integrateSystemSensitivities() = 0;
    virtual void integrateSystem(REAL_TYPE *x, REAL_TYPE *u) = 0;

    //runs the the NMPC for given iterations
    void optimize(int iters = 10);
    void shiftControlsStates();

    //calculates dx using a given du and previously calculated sensitivities
    void calculateDuToDx(REAL_TYPE *dx, REAL_TYPE *du);

    //LUP factorization and related methods
    int calculateLUPFactorization(REAL_TYPE *A, REAL_TYPE *L, REAL_TYPE *U, int *p, int n);
    int solveAXBMatrix(REAL_TYPE *X, REAL_TYPE *L, REAL_TYPE *U, REAL_TYPE *B, int *p, REAL_TYPE *Y, int n, int m);
    int solveAxb(REAL_TYPE *x, REAL_TYPE *L, REAL_TYPE *U, REAL_TYPE *b, int *p, REAL_TYPE *y, int n);


    //constant
    int numStates, numLinearStates, numControls, numParameters, numObjectiveStates, numObjectiveEndStates;
    int numSteps;
    REAL_TYPE dt;
    
    //states, controls, measurments and parameters
    REAL_TYPE *x, *x_ref; // numStates*numSteps
    REAL_TYPE *u, *u_ref; // numControls*numSteps
    REAL_TYPE *p; //numParameters
    REAL_TYPE *h_ref; //numObjectiveStates*numSteps
    REAL_TYPE *h_ref_end; //numObjectiveEndStates

    //objective function weighting matrices (might be NULL if not editable)
    REAL_TYPE *Q; //numStates*numStates
    REAL_TYPE *R; //numStates*numControls
    REAL_TYPE *P; //numStates*numStates
    REAL_TYPE *S; //numObjectiveStates*numObjectiveStates
    REAL_TYPE *S_END; //numObjectiveEndStates*numObjectiveEndStates

    //state and control constraints
    REAL_TYPE *xMin, *xMax; // numStates
    REAL_TYPE *uMin, *uMax; // numControls

    //current sensitivity calculated with integrateSystemSensitivities
    REAL_TYPE *A_all; //numSteps*numStates*numStates
    REAL_TYPE *B_all; //numSteps*numStates*numControls

protected:

    SolverType solverType;
    Solver *solver;

    //In gradient projection?
    //REAL_TYPE dG_du_proj[ NUM_STEPS*NUM_CONTROLS ];

};


#endif
