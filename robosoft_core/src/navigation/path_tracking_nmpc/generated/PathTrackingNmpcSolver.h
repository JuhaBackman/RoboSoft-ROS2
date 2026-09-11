#ifndef PATHTRACKINGNMPCSOLVER_H
#define PATHTRACKINGNMPCSOLVER_H

#include "nmpc.h"
#include <cmath>

class PathTrackingNmpcSolver : public NMPCAbstract
{

public:
    PathTrackingNmpcSolver();
    ~PathTrackingNmpcSolver();

    REAL_TYPE calculateObjective(REAL_TYPE *x, REAL_TYPE *u);
    REAL_TYPE calculateObjectiveAndDerivatives(REAL_TYPE *dG_dx, REAL_TYPE *dG_du, REAL_TYPE *x, REAL_TYPE *u);
    void integrateSystemSensitivities();
    void integrateSystem(REAL_TYPE *x, REAL_TYPE *u);

private:
    void evaluateDynamics(REAL_TYPE *value, REAL_TYPE *x, REAL_TYPE *u);
    void evaluateDynamicsAndDerivatives(REAL_TYPE *value, REAL_TYPE *df_dx, REAL_TYPE *df_du, REAL_TYPE *x, REAL_TYPE *u);
    void evaluateDynamicsLinearDerivatives(REAL_TYPE *df_dx, REAL_TYPE *df_du);
    void integrateSystemSensitivitiesOneStep(REAL_TYPE *x_new, REAL_TYPE *x, REAL_TYPE *u, REAL_TYPE *A, REAL_TYPE *B);
    void initializeIntegrator();
    REAL_TYPE *ALin, *BLin, *ATemp, *BTemp, *MTemp;
    REAL_TYPE *xTemp;
    REAL_TYPE *k1, *k2, *k3, *k4;
    REAL_TYPE *dk1_dx, *dk2_dx, *dk3_dx, *dk4_dx;
    REAL_TYPE *dk1_du, *dk2_du, *dk3_du, *dk4_du;
};

#endif
