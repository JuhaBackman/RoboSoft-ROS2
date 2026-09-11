#ifndef VEHICLEEKF_H
#define VEHICLEEKF_H

#include "ekf.h"
#include <cmath>

class VehicleEKF : public EKFProblemAbstract
{

public:
    VehicleEKF();
    ~VehicleEKF();

    void initializeCovariances();
    void predictMeasurementAndDerivatives(REAL_TYPE *value, REAL_TYPE *H, REAL_TYPE *x_pred);
    void predictStateAndDerivatives(REAL_TYPE *x_pred, REAL_TYPE *A, REAL_TYPE *B);

private:
    void evaluateDynamics(REAL_TYPE *value, REAL_TYPE *x, REAL_TYPE *u);
    void evaluateDynamicsAndDerivatives(REAL_TYPE *value, REAL_TYPE *df_dx, REAL_TYPE *df_du, REAL_TYPE *x, REAL_TYPE *u);
    void evaluateDynamicsLinearDerivatives(REAL_TYPE *df_dx, REAL_TYPE *df_du);
    void integrateSystemSensitivitiesOneStep(REAL_TYPE *x_new, REAL_TYPE *x, REAL_TYPE *u, REAL_TYPE *A, REAL_TYPE *B);
    void initializeIntegrator();
    REAL_TYPE *ALin, *BLin;
};

#endif
