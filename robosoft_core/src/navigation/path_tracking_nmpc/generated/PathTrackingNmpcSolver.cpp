#include "PathTrackingNmpcSolver.h"
#include "solver.h"

PathTrackingNmpcSolver::PathTrackingNmpcSolver()
{
    this->numStates = 9;
    this->numLinearStates = 0;
    this->numControls = 2;
    this->numParameters = 5;
    this->numObjectiveStates = 0;
    this->numObjectiveEndStates = 0;
    this->numSteps = 40;
    this->dt = 0.1;

    this->solver = new Solver(this);

    this->x = new REAL_TYPE [this->numStates*(this->numSteps+1)];
    this->u = new REAL_TYPE [this->numControls*this->numSteps];
    this->x_ref = new REAL_TYPE [this->numStates*this->numSteps];
    this->u_ref = new REAL_TYPE [this->numControls*this->numSteps];
    this->h_ref = new REAL_TYPE [this->numObjectiveStates*this->numSteps];
    this->h_ref_end = new REAL_TYPE [this->numObjectiveEndStates];
    this->p = new REAL_TYPE [this->numParameters];

    this->A_all = new REAL_TYPE [this->numStates*this->numStates*this->numSteps];
    this->B_all = new REAL_TYPE [this->numStates*this->numControls*this->numSteps];

    this->Q = new REAL_TYPE [this->numStates*this->numStates];
    this->R = new REAL_TYPE [this->numStates*this->numControls];
    this->P = new REAL_TYPE [this->numStates*this->numStates];
    this->S = 0;
    this->S_END = 0;

    this->Q[0] = 8;
    this->Q[10] = 8;
    this->Q[20] = 25;
    this->Q[30] = 4;
    this->Q[40] = 12;
    this->Q[50] = 0.2;
    this->Q[60] = 0.2;
    this->Q[70] = 0.2;
    this->Q[80] = 0.2;
    this->R[0] = 4;
    this->R[3] = 12;
    this->P[0] = 24;
    this->P[10] = 24;
    this->P[20] = 75;
    this->P[30] = 12;
    this->P[40] = 36;
    this->P[50] = 0.6;
    this->P[60] = 0.6;
    this->P[70] = 0.6;
    this->P[80] = 0.6;

    this->p[ 0 ] = 4.5;
    this->p[ 1 ] = 1.4;
    this->p[ 2 ] = 0.1;
    this->p[ 3 ] = 5.6;
    this->p[ 4 ] = 0.28;
    this->xMin = new REAL_TYPE [this->numStates];
    this->xMax = new REAL_TYPE [this->numStates];
    this->uMin = new REAL_TYPE [this->numControls];
    this->uMax = new REAL_TYPE [this->numControls];

    this->xMin[0] = -1.79769e+308;
    this->xMax[0] = 1.79769e+308;
    this->xMin[1] = -1.79769e+308;
    this->xMax[1] = 1.79769e+308;
    this->xMin[2] = -1.79769e+308;
    this->xMax[2] = 1.79769e+308;
    this->xMin[3] = -1.2;
    this->xMax[3] = 2.3;
    this->xMin[4] = -0.6;
    this->xMax[4] = 0.6;
    this->xMin[5] = -1.2;
    this->xMax[5] = 2.3;
    this->xMin[6] = -0.6;
    this->xMax[6] = 0.6;
    this->xMin[7] = -1.2;
    this->xMax[7] = 2.3;
    this->xMin[8] = -0.6;
    this->xMax[8] = 0.6;

    this->uMin[0] = -5.2;
    this->uMax[0] = 5.6;
    this->uMin[1] = -0.28;
    this->uMax[1] = 0.28;

    initializeIntegrator();

}

PathTrackingNmpcSolver::~PathTrackingNmpcSolver()
{
    delete this->solver;
    delete[] this->x;
    delete[] this->u;
    delete[] this->x_ref;
    delete[] this->u_ref;
    delete[] this->h_ref;
    delete[] this->h_ref_end;
    delete[] this->p;
    delete[] this->A_all;
    delete[] this->B_all;
    delete[] this->Q;
    delete[] this->R;
    delete[] this->P;
    delete[] this->S;
    delete[] this->S_END;
    delete[] this->xMin;
    delete[] this->xMax;
    delete[] this->uMin;
    delete[] this->uMax;
    delete[] ALin;
    delete[] ATemp;
    delete[] BLin;
    delete[] BTemp;
    delete[] MTemp;
    delete[] xTemp;
    delete[] k1;
    delete[] dk1_dx;
    delete[] dk1_du;
    delete[] k2;
    delete[] dk2_dx;
    delete[] dk2_du;
    delete[] k3;
    delete[] dk3_dx;
    delete[] dk3_du;
    delete[] k4;
    delete[] dk4_dx;
    delete[] dk4_du;
}
void PathTrackingNmpcSolver::evaluateDynamics(REAL_TYPE *value, REAL_TYPE *x, REAL_TYPE *u)
{
    REAL_TYPE u_0 = u[0];
    REAL_TYPE u_1 = u[1];
    REAL_TYPE x_0 = x[0];
    REAL_TYPE x_1 = x[1];
    REAL_TYPE x_2 = x[2];
    REAL_TYPE x_3 = x[3];
    REAL_TYPE x_4 = x[4];
    REAL_TYPE x_5 = x[5];
    REAL_TYPE x_6 = x[6];
    REAL_TYPE x_7 = x[7];
    REAL_TYPE x_8 = x[8];
    REAL_TYPE p_0 = this->p[0];
    REAL_TYPE p_1 = this->p[1];
    REAL_TYPE p_2 = this->p[2];
    REAL_TYPE p_3 = this->p[3];
    REAL_TYPE p_4 = this->p[4];


    value[0] = (x_3 * cos(x_2));
    value[1] = (x_3 * sin(x_2));
    value[2] = (x_3 * x_4);
    value[3] = (p_3 * tanh(((x_5 - x_3) / (p_0 * p_3))));
    value[4] = (p_4 * tanh(((x_6 - x_4) / (p_1 * p_4))));
    value[5] = ((x_7 - x_5) / p_2);
    value[6] = ((x_8 - x_6) / p_2);
    value[7] = u_0;
    value[8] = u_1;
}

void PathTrackingNmpcSolver::evaluateDynamicsAndDerivatives(REAL_TYPE *value, REAL_TYPE *df_dx, REAL_TYPE *df_du, REAL_TYPE *x, REAL_TYPE *u)
{
    REAL_TYPE u_0 = u[0];
    REAL_TYPE u_1 = u[1];
    REAL_TYPE x_0 = x[0];
    REAL_TYPE x_1 = x[1];
    REAL_TYPE x_2 = x[2];
    REAL_TYPE x_3 = x[3];
    REAL_TYPE x_4 = x[4];
    REAL_TYPE x_5 = x[5];
    REAL_TYPE x_6 = x[6];
    REAL_TYPE x_7 = x[7];
    REAL_TYPE x_8 = x[8];
    REAL_TYPE p_0 = this->p[0];
    REAL_TYPE p_1 = this->p[1];
    REAL_TYPE p_2 = this->p[2];
    REAL_TYPE p_3 = this->p[3];
    REAL_TYPE p_4 = this->p[4];



    value[0] = (x_3 * cos(x_2));
    value[1] = (x_3 * sin(x_2));
    value[2] = (x_3 * x_4);
    value[3] = (p_3 * tanh(((x_5 - x_3) / (p_0 * p_3))));
    value[4] = (p_4 * tanh(((x_6 - x_4) / (p_1 * p_4))));
    value[5] = ((x_7 - x_5) / p_2);
    value[6] = ((x_8 - x_6) / p_2);
    value[7] = u_0;
    value[8] = u_1;

    df_dx[0] = 0;
    df_dx[1] = 0;
    df_dx[2] = (x_3 *  (-sin(x_2)));
    df_dx[3] = cos(x_2);
    df_dx[4] = 0;
    df_dx[5] = 0;
    df_dx[6] = 0;
    df_dx[7] = 0;
    df_dx[8] = 0;
    df_dx[9] = 0;
    df_dx[10] = 0;
    df_dx[11] = (x_3 * cos(x_2));
    df_dx[12] = sin(x_2);
    df_dx[13] = 0;
    df_dx[14] = 0;
    df_dx[15] = 0;
    df_dx[16] = 0;
    df_dx[17] = 0;
    df_dx[18] = 0;
    df_dx[19] = 0;
    df_dx[20] = 0;
    df_dx[21] = x_4;
    df_dx[22] = x_3;
    df_dx[23] = 0;
    df_dx[24] = 0;
    df_dx[25] = 0;
    df_dx[26] = 0;
    df_dx[27] = 0;
    df_dx[28] = 0;
    df_dx[29] = 0;
    df_dx[30] = (p_3 * (( (-(p_0 * p_3)) / ((p_0 * p_3) * (p_0 * p_3))) * (1 - (tanh(((x_5 - x_3) / (p_0 * p_3))) * tanh(((x_5 - x_3) / (p_0 * p_3)))))));
    df_dx[31] = 0;
    df_dx[32] = (p_3 * (((p_0 * p_3) / ((p_0 * p_3) * (p_0 * p_3))) * (1 - (tanh(((x_5 - x_3) / (p_0 * p_3))) * tanh(((x_5 - x_3) / (p_0 * p_3)))))));
    df_dx[33] = 0;
    df_dx[34] = 0;
    df_dx[35] = 0;
    df_dx[36] = 0;
    df_dx[37] = 0;
    df_dx[38] = 0;
    df_dx[39] = 0;
    df_dx[40] = (p_4 * (( (-(p_1 * p_4)) / ((p_1 * p_4) * (p_1 * p_4))) * (1 - (tanh(((x_6 - x_4) / (p_1 * p_4))) * tanh(((x_6 - x_4) / (p_1 * p_4)))))));
    df_dx[41] = 0;
    df_dx[42] = (p_4 * (((p_1 * p_4) / ((p_1 * p_4) * (p_1 * p_4))) * (1 - (tanh(((x_6 - x_4) / (p_1 * p_4))) * tanh(((x_6 - x_4) / (p_1 * p_4)))))));
    df_dx[43] = 0;
    df_dx[44] = 0;
    df_dx[45] = 0;
    df_dx[46] = 0;
    df_dx[47] = 0;
    df_dx[48] = 0;
    df_dx[49] = 0;
    df_dx[50] = ( (-p_2) / (p_2 * p_2));
    df_dx[51] = 0;
    df_dx[52] = (p_2 / (p_2 * p_2));
    df_dx[53] = 0;
    df_dx[54] = 0;
    df_dx[55] = 0;
    df_dx[56] = 0;
    df_dx[57] = 0;
    df_dx[58] = 0;
    df_dx[59] = 0;
    df_dx[60] = ( (-p_2) / (p_2 * p_2));
    df_dx[61] = 0;
    df_dx[62] = (p_2 / (p_2 * p_2));
    df_dx[63] = 0;
    df_dx[64] = 0;
    df_dx[65] = 0;
    df_dx[66] = 0;
    df_dx[67] = 0;
    df_dx[68] = 0;
    df_dx[69] = 0;
    df_dx[70] = 0;
    df_dx[71] = 0;
    df_dx[72] = 0;
    df_dx[73] = 0;
    df_dx[74] = 0;
    df_dx[75] = 0;
    df_dx[76] = 0;
    df_dx[77] = 0;
    df_dx[78] = 0;
    df_dx[79] = 0;
    df_dx[80] = 0;

    df_du[0] = 0;
    df_du[1] = 0;
    df_du[2] = 0;
    df_du[3] = 0;
    df_du[4] = 0;
    df_du[5] = 0;
    df_du[6] = 0;
    df_du[7] = 0;
    df_du[8] = 0;
    df_du[9] = 0;
    df_du[10] = 0;
    df_du[11] = 0;
    df_du[12] = 0;
    df_du[13] = 0;
    df_du[14] = 1;
    df_du[15] = 0;
    df_du[16] = 0;
    df_du[17] = 1;
}

void PathTrackingNmpcSolver::evaluateDynamicsLinearDerivatives(REAL_TYPE *df_dx, REAL_TYPE *df_du)
{
    REAL_TYPE p_0 = this->p[0];
    REAL_TYPE p_1 = this->p[1];
    REAL_TYPE p_2 = this->p[2];
    REAL_TYPE p_3 = this->p[3];
    REAL_TYPE p_4 = this->p[4];


}

void PathTrackingNmpcSolver::initializeIntegrator()
{
    ALin = new REAL_TYPE[numStates*numStates];
    ATemp = new REAL_TYPE[numStates*numStates];
    MTemp = new REAL_TYPE[numStates*(numStates > numControls ? numStates : numControls)];
    dk1_dx = new REAL_TYPE[numStates*numStates];
    dk2_dx = new REAL_TYPE[numStates*numStates];
    dk3_dx = new REAL_TYPE[numStates*numStates];
    dk4_dx = new REAL_TYPE[numStates*numStates];
    BTemp = new REAL_TYPE[numStates*numControls];
    BLin = new REAL_TYPE[numStates*numControls];
    dk1_du = new REAL_TYPE[numStates*numControls];
    dk2_du = new REAL_TYPE[numStates*numControls];
    dk3_du = new REAL_TYPE[numStates*numControls];
    dk4_du = new REAL_TYPE[numStates*numControls];
    xTemp = new REAL_TYPE[numStates];
    k1 = new REAL_TYPE[numStates];
    k2 = new REAL_TYPE[numStates];
    k3 = new REAL_TYPE[numStates];
    k4 = new REAL_TYPE[numStates];

    int i,j,k;

    for(i = 0; i < numStates*numStates; i++)
        ALin[i] = ATemp[i] = MTemp[i]  = dk1_dx[i] = dk2_dx[i] = dk3_dx[i] = dk4_dx[i] = 0;

    for(i = 0; i < numStates*numControls; i++)
        BLin[i] = BTemp[i] = MTemp[i]  = dk1_du[i] = dk2_du[i] = dk3_du[i] = dk4_du[i] = 0;

    /*Initialize constant parts*/
    /*dk1/dx = ATemp = df(x,u)/dx*/
    /*dk1/du = BTemp = df(x,u)/du*/
    evaluateDynamicsLinearDerivatives(dk1_dx, dk1_du);
    evaluateDynamicsLinearDerivatives(ATemp, BTemp);


    /*MTemp = I + dt*(0.5 * dk1/dx)*/
    for(i = 0; i < numStates*numStates; i++)
        MTemp[i] = dt*(0.5 * dk1_dx[i]);
    for(i = 0; i < numStates; i++)
        MTemp[i*(numStates+1)] += 1.0;

    /*dk2/dx = ATemp*MTemp*/
    for(i = 0; i < numLinearStates; i++)
        for(j = 0; j < numStates; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numStates + j];
            dk2_dx[i*numStates + j] = t;
        }

    /*MTemp = dt*(0.5 * dk1/du)*/
    for(i = 0; i < numStates*numControls; i++)
        MTemp[i] = dt*(0.5 * dk1_du[i]);

    /*dk2/du = ATemp*MTemp + BTemp*/
    for(i = 0; i < numLinearStates; i++)
        for(j = 0; j < numControls; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numControls + j];
            dk2_du[i*numControls + j] = t;
        }
    for(i = 0; i < numLinearStates*numControls; i++)
            dk2_du[i] += BTemp[i];

    /*MTemp = I + dt*(0.5 * dk2/dx)*/
    for(i = 0; i < numStates*numStates; i++)
        MTemp[i] = dt*(0.5 * dk2_dx[i]);
    for(i = 0; i < numStates; i++)
        MTemp[i*(numStates+1)] += 1.0;

    /*dk3/dx = ATemp*MTemp*/
    for(i = 0; i < numLinearStates; i++)
        for(j = 0; j < numStates; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numStates + j];
            dk3_dx[i*numStates + j] = t;
        }

    /*MTemp = dt*(0.5 * dk2/du)*/
    for(i = 0; i < numStates*numControls; i++)
        MTemp[i] = dt*(0.5 * dk2_du[i]);

    /*dk3/du = ATemp*MTemp + BTemp*/
    for(i = 0; i < numLinearStates; i++)
        for(j = 0; j < numControls; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numControls + j];
            dk3_du[i*numControls + j] = t;
        }
    for(i = 0; i < numLinearStates*numControls; i++)
            dk3_du[i] += BTemp[i];

    /*MTemp = I + dt*(1 * dk3/dx)*/
    for(i = 0; i < numStates*numStates; i++)
        MTemp[i] = dt*(1 * dk3_dx[i]);
    for(i = 0; i < numStates; i++)
        MTemp[i*(numStates+1)] += 1.0;

    /*dk4/dx = ATemp*MTemp*/
    for(i = 0; i < numLinearStates; i++)
        for(j = 0; j < numStates; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numStates + j];
            dk4_dx[i*numStates + j] = t;
        }

    /*MTemp = dt*(1 * dk3/du)*/
    for(i = 0; i < numStates*numControls; i++)
        MTemp[i] = dt*(1 * dk3_du[i]);

    /*dk4/du = ATemp*MTemp + BTemp*/
    for(i = 0; i < numLinearStates; i++)
        for(j = 0; j < numControls; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numControls + j];
            dk4_du[i*numControls + j] = t;
        }
    for(i = 0; i < numLinearStates*numControls; i++)
            dk4_du[i] += BTemp[i];

    /*Calculate sensitivities*/
    /*Non-linear parts*/
    for(i = 0; i < numLinearStates*numStates; i++)
       ALin[i] = dt*(0.166667 * dk1_dx[i] + 0.333333 * dk2_dx[i] + 0.333333 * dk3_dx[i] + 0.166667 * dk4_dx[i]);
    for(i = 0; i < numLinearStates; i++)
        ALin[i*(numStates+1)] += 1.0;

    for(i = 0; i < numLinearStates*numControls; i++)
       BLin[i] = dt*(0.166667 * dk1_du[i] + 0.333333 * dk2_du[i] + 0.333333 * dk3_du[i] + 0.166667 * dk4_du[i]);
}

void PathTrackingNmpcSolver::integrateSystemSensitivitiesOneStep(REAL_TYPE *x_new, REAL_TYPE *x, REAL_TYPE *u, REAL_TYPE *A, REAL_TYPE *B)
{
    int i,j,k;

    /*k1 = f(x,u)*/
    /*dk1/dx = df(x,u)/dx*/
    /*dk1/du = df(x,u)/du*/
    evaluateDynamicsAndDerivatives(k1, dk1_dx, dk1_du, x, u);

    /*k2 = f(x + dt*(0.5 * k1))*/
    for(j=0; j < numStates; j++)
       xTemp[j] = x[j] + dt*(0.5 * k1[j]);

    evaluateDynamicsAndDerivatives(k2, ATemp, BTemp, xTemp, u);

    /*MTemp = I + dt*(0.5 * dk1/dx)*/
    for(i = 0; i < numStates*numStates; i++)
        MTemp[i] = dt*(0.5 * dk1_dx[i]);
    for(i = 0; i < numStates; i++)
        MTemp[i*(numStates+1)] += 1.0;

    /*dk2/dx = ATemp*MTemp*/
    for(i = numLinearStates; i < numStates; i++)
        for(j = 0; j < numStates; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numStates + j];
            dk2_dx[i*numStates + j] = t;
        }

    /*MTemp = dt*(0.5 * dk1/du)*/
    for(i = 0; i < numStates*numControls; i++)
        MTemp[i] = dt*(0.5 * dk1_du[i]);

    /*dk2/du = ATemp*MTemp + BTemp*/
    for(i = numLinearStates; i < numStates; i++)
        for(j = 0; j < numControls; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numControls + j];
            dk1_du[i*numControls + j] = t;
        }
    for(i = numLinearStates*numControls; i < numStates*numControls; i++)
            dk1_du[i] += BTemp[i];

    /*k3 = f(x + dt*(0.5 * k2))*/
    for(j=0; j < numStates; j++)
       xTemp[j] = x[j] + dt*(0.5 * k2[j]);

    evaluateDynamicsAndDerivatives(k3, ATemp, BTemp, xTemp, u);

    /*MTemp = I + dt*(0.5 * dk2/dx)*/
    for(i = 0; i < numStates*numStates; i++)
        MTemp[i] = dt*(0.5 * dk2_dx[i]);
    for(i = 0; i < numStates; i++)
        MTemp[i*(numStates+1)] += 1.0;

    /*dk3/dx = ATemp*MTemp*/
    for(i = numLinearStates; i < numStates; i++)
        for(j = 0; j < numStates; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numStates + j];
            dk3_dx[i*numStates + j] = t;
        }

    /*MTemp = dt*(0.5 * dk2/du)*/
    for(i = 0; i < numStates*numControls; i++)
        MTemp[i] = dt*(0.5 * dk2_du[i]);

    /*dk3/du = ATemp*MTemp + BTemp*/
    for(i = numLinearStates; i < numStates; i++)
        for(j = 0; j < numControls; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numControls + j];
            dk2_du[i*numControls + j] = t;
        }
    for(i = numLinearStates*numControls; i < numStates*numControls; i++)
            dk2_du[i] += BTemp[i];

    /*k4 = f(x + dt*(1 * k3))*/
    for(j=0; j < numStates; j++)
       xTemp[j] = x[j] + dt*(1 * k3[j]);

    evaluateDynamicsAndDerivatives(k4, ATemp, BTemp, xTemp, u);

    /*MTemp = I + dt*(1 * dk3/dx)*/
    for(i = 0; i < numStates*numStates; i++)
        MTemp[i] = dt*(1 * dk3_dx[i]);
    for(i = 0; i < numStates; i++)
        MTemp[i*(numStates+1)] += 1.0;

    /*dk4/dx = ATemp*MTemp*/
    for(i = numLinearStates; i < numStates; i++)
        for(j = 0; j < numStates; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numStates + j];
            dk4_dx[i*numStates + j] = t;
        }

    /*MTemp = dt*(1 * dk3/du)*/
    for(i = 0; i < numStates*numControls; i++)
        MTemp[i] = dt*(1 * dk3_du[i]);

    /*dk4/du = ATemp*MTemp + BTemp*/
    for(i = numLinearStates; i < numStates; i++)
        for(j = 0; j < numControls; j++)
        {
            REAL_TYPE t = 0.0;
            for(k=0; k < numStates; k++)
                t += ATemp[i*numStates + k]*MTemp[k*numControls + j];
            dk3_du[i*numControls + j] = t;
        }
    for(i = numLinearStates*numControls; i < numStates*numControls; i++)
            dk3_du[i] += BTemp[i];

    /*Calculate new state vector*/
    /*x_new = x + dt*(0.166667 * k1 + 0.333333 * k2 + 0.333333 * k3 + 0.166667 * k4)*/
    for(i=0; i < numStates; i++)
       x_new[i] = x[i] + dt*(0.166667 * k1[i] + 0.333333 * k2[i] + 0.333333 * k3[i] + 0.166667 * k4[i]);

    /*Linear parts*/
    for(i=0; i < numLinearStates*numStates; i++)
        A[i] = ALin[i];

    for(i=0; i < numLinearStates*numControls; i++)
        B[i] = BLin[i];

    /*Calculate sensitivities*/
    /*Non-linear parts*/
    for(i=numLinearStates*numStates; i < numStates*numStates; i++)
       A[i] = dt*(0.166667 * dk1_dx[i] + 0.333333 * dk2_dx[i] + 0.333333 * dk3_dx[i] + 0.166667 * dk4_dx[i]);
    for(i = numLinearStates; i < numStates; i++)
        A[i*(numStates+1)] += 1.0;

    for(i=numLinearStates*numControls; i < numStates*numControls; i++)
       B[i] = dt*(0.166667 * dk1_du[i] + 0.333333 * dk2_du[i] + 0.333333 * dk3_du[i] + 0.166667 * dk4_du[i]);
}

void PathTrackingNmpcSolver::integrateSystemSensitivities()
{
    int i,j;

    for(j=0; j < numSteps; j++)
    {
        integrateSystemSensitivitiesOneStep(&x[(j+1)*numStates], &x[j*numStates], &u[j*numControls], &A_all[j*numStates*numStates], &B_all[j*numStates*numControls]);
    }
}

void PathTrackingNmpcSolver::integrateSystem(REAL_TYPE *x, REAL_TYPE *u)
{
    int i,j;

    for(j=0; j < numSteps; j++)
    {
        /*FIXME doesn't work for exact linear states!*/
        /*k1 = f(x,u)*/
        evaluateDynamics(k1, &x[j*numStates], &u[j*numControls]);

        /*k2 = f(x + dt*(0.5 * k1))*/
        for(i=0; i < numStates; i++)
           xTemp[i] = x[j*numStates + i] + dt*(0.5 * k1[i]);

        evaluateDynamics(k2, xTemp, &u[j*numControls]);

        /*k3 = f(x + dt*(0.5 * k2))*/
        for(i=0; i < numStates; i++)
           xTemp[i] = x[j*numStates + i] + dt*(0.5 * k2[i]);

        evaluateDynamics(k3, xTemp, &u[j*numControls]);

        /*k4 = f(x + dt*(1 * k3))*/
        for(i=0; i < numStates; i++)
           xTemp[i] = x[j*numStates + i] + dt*(1 * k3[i]);

        evaluateDynamics(k4, xTemp, &u[j*numControls]);

        /*Calculate new state vector*/
        /*x_new = x + dt*(0.166667 * k1 + 0.333333 * k2 + 0.333333 * k3 + 0.166667 * k4)*/
        for(i=0; i < numStates; i++)
           x[(j+1)*numStates + i] = x[j*numStates + i] + dt*(0.166667 * k1[i] + 0.333333 * k2[i] + 0.333333 * k3[i] + 0.166667 * k4[i]);

    }
}

REAL_TYPE PathTrackingNmpcSolver::calculateObjective(REAL_TYPE *x, REAL_TYPE *u)
{
    double objective = 0.0;

    double x_err[ 9 ];
    double u_err[ 2 ];
    int i,j;
    for(i=0; i < this->numSteps; i++)
    {

        for(j=0; j < 9; j++)
        {
            x_err[j] = x[(i+1)*9 + j] - x_ref[i*9 + j];
        }

        for(j=0; j < 2; j++)
        {
            u_err[j] = u[i*2 + j] - u_ref[i*2 + j];
        }

        objective += x_err[0] * x_err[0] * this->Q[0];
        objective += x_err[1] * x_err[1] * this->Q[10];
        objective += x_err[2] * x_err[2] * this->Q[20];
        objective += x_err[3] * x_err[3] * this->Q[30];
        objective += x_err[4] * x_err[4] * this->Q[40];
        objective += x_err[5] * x_err[5] * this->Q[50];
        objective += x_err[6] * x_err[6] * this->Q[60];
        objective += x_err[7] * x_err[7] * this->Q[70];
        objective += x_err[8] * x_err[8] * this->Q[80];

        objective += u_err[0] * u_err[0] * this->R[0];
        objective += u_err[1] * u_err[1] * this->R[3];
    }

    objective += x_err[0] * x_err[0] * this->P[0];
    objective += x_err[1] * x_err[1] * this->P[10];
    objective += x_err[2] * x_err[2] * this->P[20];
    objective += x_err[3] * x_err[3] * this->P[30];
    objective += x_err[4] * x_err[4] * this->P[40];
    objective += x_err[5] * x_err[5] * this->P[50];
    objective += x_err[6] * x_err[6] * this->P[60];
    objective += x_err[7] * x_err[7] * this->P[70];
    objective += x_err[8] * x_err[8] * this->P[80];

    return objective;
}

REAL_TYPE PathTrackingNmpcSolver::calculateObjectiveAndDerivatives(REAL_TYPE *dG_dx, REAL_TYPE *dG_du, REAL_TYPE *x, REAL_TYPE *u)
{
    REAL_TYPE objective = 0.0;

    REAL_TYPE x_err[ 9 ];
    REAL_TYPE u_err[ 2 ];
    int i,j;
    for(i=0; i < this->numSteps; i++)
    {

        for(j=0; j < 9; j++)
        {
            x_err[j] = x[(i+1)*9 + j] - x_ref[i*9 + j];
        }

        for(j=0; j < 2; j++)
        {
            u_err[j] = u[i*2 + j] - u_ref[i*2 + j];
        }

        objective += x_err[0] * x_err[0] * this->Q[0];
        objective += x_err[1] * x_err[1] * this->Q[10];
        objective += x_err[2] * x_err[2] * this->Q[20];
        objective += x_err[3] * x_err[3] * this->Q[30];
        objective += x_err[4] * x_err[4] * this->Q[40];
        objective += x_err[5] * x_err[5] * this->Q[50];
        objective += x_err[6] * x_err[6] * this->Q[60];
        objective += x_err[7] * x_err[7] * this->Q[70];
        objective += x_err[8] * x_err[8] * this->Q[80];

        objective += u_err[0] * u_err[0] * this->R[0];
        objective += u_err[1] * u_err[1] * this->R[3];

        dG_dx[i*9 + 0] = x_err[0] * -(this->Q[0] + this->Q[0]);
        dG_dx[i*9 + 1] = x_err[1] * -(this->Q[10] + this->Q[10]);
        dG_dx[i*9 + 2] = x_err[2] * -(this->Q[20] + this->Q[20]);
        dG_dx[i*9 + 3] = x_err[3] * -(this->Q[30] + this->Q[30]);
        dG_dx[i*9 + 4] = x_err[4] * -(this->Q[40] + this->Q[40]);
        dG_dx[i*9 + 5] = x_err[5] * -(this->Q[50] + this->Q[50]);
        dG_dx[i*9 + 6] = x_err[6] * -(this->Q[60] + this->Q[60]);
        dG_dx[i*9 + 7] = x_err[7] * -(this->Q[70] + this->Q[70]);
        dG_dx[i*9 + 8] = x_err[8] * -(this->Q[80] + this->Q[80]);

        dG_du[i*2 + 0] = u_err[0] * -(this->R[0] + this->R[0]);
        dG_du[i*2 + 1] = u_err[1] * -(this->R[3] + this->R[3]);
    }

    objective += x_err[0] * x_err[0] * this->P[0];
    objective += x_err[1] * x_err[1] * this->P[10];
    objective += x_err[2] * x_err[2] * this->P[20];
    objective += x_err[3] * x_err[3] * this->P[30];
    objective += x_err[4] * x_err[4] * this->P[40];
    objective += x_err[5] * x_err[5] * this->P[50];
    objective += x_err[6] * x_err[6] * this->P[60];
    objective += x_err[7] * x_err[7] * this->P[70];
    objective += x_err[8] * x_err[8] * this->P[80];

    dG_dx[(this->numSteps-1)*9 + 0] += x_err[0] * -(this->P[0] + this->P[0]);
    dG_dx[(this->numSteps-1)*9 + 1] += x_err[1] * -(this->P[10] + this->P[10]);
    dG_dx[(this->numSteps-1)*9 + 2] += x_err[2] * -(this->P[20] + this->P[20]);
    dG_dx[(this->numSteps-1)*9 + 3] += x_err[3] * -(this->P[30] + this->P[30]);
    dG_dx[(this->numSteps-1)*9 + 4] += x_err[4] * -(this->P[40] + this->P[40]);
    dG_dx[(this->numSteps-1)*9 + 5] += x_err[5] * -(this->P[50] + this->P[50]);
    dG_dx[(this->numSteps-1)*9 + 6] += x_err[6] * -(this->P[60] + this->P[60]);
    dG_dx[(this->numSteps-1)*9 + 7] += x_err[7] * -(this->P[70] + this->P[70]);
    dG_dx[(this->numSteps-1)*9 + 8] += x_err[8] * -(this->P[80] + this->P[80]);

    return objective;
}

