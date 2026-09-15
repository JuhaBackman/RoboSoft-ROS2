#include "VehicleEKF.h"

VehicleEKF::VehicleEKF()
{
    this->numStates = 15;
    this->numLinearStates = 0;
    this->numControls = 2;
    this->numParameters = 0;
    this->dt = 0.1;
    this->numEKFStates = 15;
    this->numEKFMeasurements = 15;

    this->x = new REAL_TYPE [this->numEKFStates];
    this->u = new REAL_TYPE [this->numControls];
    this->y = new REAL_TYPE [this->numEKFMeasurements];
    this->p = new REAL_TYPE [this->numParameters];

    this->Q = new REAL_TYPE [this->numEKFStates*this->numEKFStates];
    this->R = new REAL_TYPE [this->numEKFMeasurements*this->numEKFMeasurements];
    this->P = new REAL_TYPE [this->numEKFStates*this->numEKFStates];

    initializeCovariances();

    initializeIntegrator();
}

VehicleEKF::~VehicleEKF()
{
    delete[] this->x;
    delete[] this->u;
    delete[] this->y;
    delete[] this->p;
    delete[] this->Q;
    delete[] this->R;
    delete[] this->P;
    delete[] ALin;
    delete[] BLin;
}
void VehicleEKF::initializeCovariances()
{
    int i;
    for(i = 0; i < 15*15; i++)
        this->Q[i] = 0.0;

    this->Q[ 0*15 + 0 ] = 1e-07;
    this->Q[ 1*15 + 1 ] = 1e-07;
    this->Q[ 2*15 + 2 ] = 1e-05;

    for(i = 0; i < 15*15; i++)
        this->R[i] = 0.0;

    this->R[ 0*15 + 0 ] = 2e-05;
    this->R[ 1*15 + 1 ] = 2e-05;
    this->R[ 2*15 + 2 ] = 1e-06;
    this->R[ 3*15 + 3 ] = 0.000225;
    this->R[ 4*15 + 4 ] = 0.000225;
    this->R[ 5*15 + 5 ] = 0.000225;
    this->R[ 6*15 + 6 ] = 0.000225;
    this->R[ 7*15 + 7 ] = 0.000225;
    this->R[ 8*15 + 8 ] = 0.000225;
    this->R[ 9*15 + 9 ] = 0.000225;
    this->R[ 10*15 + 10 ] = 0.000225;
    this->R[ 11*15 + 11 ] = 0.000225;
    this->R[ 12*15 + 12 ] = 0.000225;
    this->R[ 13*15 + 13 ] = 0.000225;
    this->R[ 14*15 + 14 ] = 0.000225;

    for(i = 0; i < 15*15; i++)
        this->P[i] = 0.0;

    this->P[ 0*15 + 0 ] = 1;
    this->P[ 1*15 + 1 ] = 1;
    this->P[ 2*15 + 2 ] = 1;

}
void VehicleEKF::evaluateDynamics(REAL_TYPE *value, REAL_TYPE *x, REAL_TYPE *u)
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
    REAL_TYPE x_9 = x[9];
    REAL_TYPE x_10 = x[10];
    REAL_TYPE x_11 = x[11];
    REAL_TYPE x_12 = x[12];
    REAL_TYPE x_13 = x[13];
    REAL_TYPE x_14 = x[14];


    value[0] = (x_0 + ((0.1 * u_0) * cos(x_2)));
    value[1] = (x_1 + ((0.1 * u_0) * sin(x_2)));
    value[2] = (x_2 + ((0.1 * u_0) * u_1));
    value[3] = x_3;
    value[4] = x_4;
    value[5] = x_5;
    value[6] = x_6;
    value[7] = x_7;
    value[8] = x_8;
    value[9] = x_9;
    value[10] = x_10;
    value[11] = x_11;
    value[12] = x_12;
    value[13] = x_13;
    value[14] = x_14;
}

void VehicleEKF::evaluateDynamicsAndDerivatives(REAL_TYPE *value, REAL_TYPE *df_dx, REAL_TYPE *df_du, REAL_TYPE *x, REAL_TYPE *u)
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
    REAL_TYPE x_9 = x[9];
    REAL_TYPE x_10 = x[10];
    REAL_TYPE x_11 = x[11];
    REAL_TYPE x_12 = x[12];
    REAL_TYPE x_13 = x[13];
    REAL_TYPE x_14 = x[14];



    value[0] = (x_0 + ((0.1 * u_0) * cos(x_2)));
    value[1] = (x_1 + ((0.1 * u_0) * sin(x_2)));
    value[2] = (x_2 + ((0.1 * u_0) * u_1));
    value[3] = x_3;
    value[4] = x_4;
    value[5] = x_5;
    value[6] = x_6;
    value[7] = x_7;
    value[8] = x_8;
    value[9] = x_9;
    value[10] = x_10;
    value[11] = x_11;
    value[12] = x_12;
    value[13] = x_13;
    value[14] = x_14;

    df_dx[0] = 1;
    df_dx[1] = 0;
    df_dx[2] = ((0.1 * u_0) *  (-sin(x_2)));
    df_dx[3] = 0;
    df_dx[4] = 0;
    df_dx[5] = 0;
    df_dx[6] = 0;
    df_dx[7] = 0;
    df_dx[8] = 0;
    df_dx[9] = 0;
    df_dx[10] = 0;
    df_dx[11] = 0;
    df_dx[12] = 0;
    df_dx[13] = 0;
    df_dx[14] = 0;
    df_dx[15] = 0;
    df_dx[16] = 1;
    df_dx[17] = ((0.1 * u_0) * cos(x_2));
    df_dx[18] = 0;
    df_dx[19] = 0;
    df_dx[20] = 0;
    df_dx[21] = 0;
    df_dx[22] = 0;
    df_dx[23] = 0;
    df_dx[24] = 0;
    df_dx[25] = 0;
    df_dx[26] = 0;
    df_dx[27] = 0;
    df_dx[28] = 0;
    df_dx[29] = 0;
    df_dx[30] = 0;
    df_dx[31] = 0;
    df_dx[32] = 1;
    df_dx[33] = 0;
    df_dx[34] = 0;
    df_dx[35] = 0;
    df_dx[36] = 0;
    df_dx[37] = 0;
    df_dx[38] = 0;
    df_dx[39] = 0;
    df_dx[40] = 0;
    df_dx[41] = 0;
    df_dx[42] = 0;
    df_dx[43] = 0;
    df_dx[44] = 0;
    df_dx[45] = 0;
    df_dx[46] = 0;
    df_dx[47] = 0;
    df_dx[48] = 1;
    df_dx[49] = 0;
    df_dx[50] = 0;
    df_dx[51] = 0;
    df_dx[52] = 0;
    df_dx[53] = 0;
    df_dx[54] = 0;
    df_dx[55] = 0;
    df_dx[56] = 0;
    df_dx[57] = 0;
    df_dx[58] = 0;
    df_dx[59] = 0;
    df_dx[60] = 0;
    df_dx[61] = 0;
    df_dx[62] = 0;
    df_dx[63] = 0;
    df_dx[64] = 1;
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
    df_dx[80] = 1;
    df_dx[81] = 0;
    df_dx[82] = 0;
    df_dx[83] = 0;
    df_dx[84] = 0;
    df_dx[85] = 0;
    df_dx[86] = 0;
    df_dx[87] = 0;
    df_dx[88] = 0;
    df_dx[89] = 0;
    df_dx[90] = 0;
    df_dx[91] = 0;
    df_dx[92] = 0;
    df_dx[93] = 0;
    df_dx[94] = 0;
    df_dx[95] = 0;
    df_dx[96] = 1;
    df_dx[97] = 0;
    df_dx[98] = 0;
    df_dx[99] = 0;
    df_dx[100] = 0;
    df_dx[101] = 0;
    df_dx[102] = 0;
    df_dx[103] = 0;
    df_dx[104] = 0;
    df_dx[105] = 0;
    df_dx[106] = 0;
    df_dx[107] = 0;
    df_dx[108] = 0;
    df_dx[109] = 0;
    df_dx[110] = 0;
    df_dx[111] = 0;
    df_dx[112] = 1;
    df_dx[113] = 0;
    df_dx[114] = 0;
    df_dx[115] = 0;
    df_dx[116] = 0;
    df_dx[117] = 0;
    df_dx[118] = 0;
    df_dx[119] = 0;
    df_dx[120] = 0;
    df_dx[121] = 0;
    df_dx[122] = 0;
    df_dx[123] = 0;
    df_dx[124] = 0;
    df_dx[125] = 0;
    df_dx[126] = 0;
    df_dx[127] = 0;
    df_dx[128] = 1;
    df_dx[129] = 0;
    df_dx[130] = 0;
    df_dx[131] = 0;
    df_dx[132] = 0;
    df_dx[133] = 0;
    df_dx[134] = 0;
    df_dx[135] = 0;
    df_dx[136] = 0;
    df_dx[137] = 0;
    df_dx[138] = 0;
    df_dx[139] = 0;
    df_dx[140] = 0;
    df_dx[141] = 0;
    df_dx[142] = 0;
    df_dx[143] = 0;
    df_dx[144] = 1;
    df_dx[145] = 0;
    df_dx[146] = 0;
    df_dx[147] = 0;
    df_dx[148] = 0;
    df_dx[149] = 0;
    df_dx[150] = 0;
    df_dx[151] = 0;
    df_dx[152] = 0;
    df_dx[153] = 0;
    df_dx[154] = 0;
    df_dx[155] = 0;
    df_dx[156] = 0;
    df_dx[157] = 0;
    df_dx[158] = 0;
    df_dx[159] = 0;
    df_dx[160] = 1;
    df_dx[161] = 0;
    df_dx[162] = 0;
    df_dx[163] = 0;
    df_dx[164] = 0;
    df_dx[165] = 0;
    df_dx[166] = 0;
    df_dx[167] = 0;
    df_dx[168] = 0;
    df_dx[169] = 0;
    df_dx[170] = 0;
    df_dx[171] = 0;
    df_dx[172] = 0;
    df_dx[173] = 0;
    df_dx[174] = 0;
    df_dx[175] = 0;
    df_dx[176] = 1;
    df_dx[177] = 0;
    df_dx[178] = 0;
    df_dx[179] = 0;
    df_dx[180] = 0;
    df_dx[181] = 0;
    df_dx[182] = 0;
    df_dx[183] = 0;
    df_dx[184] = 0;
    df_dx[185] = 0;
    df_dx[186] = 0;
    df_dx[187] = 0;
    df_dx[188] = 0;
    df_dx[189] = 0;
    df_dx[190] = 0;
    df_dx[191] = 0;
    df_dx[192] = 1;
    df_dx[193] = 0;
    df_dx[194] = 0;
    df_dx[195] = 0;
    df_dx[196] = 0;
    df_dx[197] = 0;
    df_dx[198] = 0;
    df_dx[199] = 0;
    df_dx[200] = 0;
    df_dx[201] = 0;
    df_dx[202] = 0;
    df_dx[203] = 0;
    df_dx[204] = 0;
    df_dx[205] = 0;
    df_dx[206] = 0;
    df_dx[207] = 0;
    df_dx[208] = 1;
    df_dx[209] = 0;
    df_dx[210] = 0;
    df_dx[211] = 0;
    df_dx[212] = 0;
    df_dx[213] = 0;
    df_dx[214] = 0;
    df_dx[215] = 0;
    df_dx[216] = 0;
    df_dx[217] = 0;
    df_dx[218] = 0;
    df_dx[219] = 0;
    df_dx[220] = 0;
    df_dx[221] = 0;
    df_dx[222] = 0;
    df_dx[223] = 0;
    df_dx[224] = 1;

    df_du[0] = (0.1 * cos(x_2));
    df_du[1] = 0;
    df_du[2] = (0.1 * sin(x_2));
    df_du[3] = 0;
    df_du[4] = (0.1 * u_1);
    df_du[5] = (0.1 * u_0);
    df_du[6] = 0;
    df_du[7] = 0;
    df_du[8] = 0;
    df_du[9] = 0;
    df_du[10] = 0;
    df_du[11] = 0;
    df_du[12] = 0;
    df_du[13] = 0;
    df_du[14] = 0;
    df_du[15] = 0;
    df_du[16] = 0;
    df_du[17] = 0;
    df_du[18] = 0;
    df_du[19] = 0;
    df_du[20] = 0;
    df_du[21] = 0;
    df_du[22] = 0;
    df_du[23] = 0;
    df_du[24] = 0;
    df_du[25] = 0;
    df_du[26] = 0;
    df_du[27] = 0;
    df_du[28] = 0;
    df_du[29] = 0;
}

void VehicleEKF::evaluateDynamicsLinearDerivatives(REAL_TYPE *df_dx, REAL_TYPE *df_du)
{


}

void VehicleEKF::predictMeasurementAndDerivatives(REAL_TYPE *value, REAL_TYPE *H, REAL_TYPE *x_pred)
{
    for(int i = 0; i < this->numEKFMeasurements*this->numEKFStates; i++)
        H[i] = 0.0;

{
    /*Lag = 0 */
    REAL_TYPE x_0 = x_pred[0];
    REAL_TYPE x_1 = x_pred[1];
    REAL_TYPE x_2 = x_pred[2];
    REAL_TYPE x_3 = x_pred[3];
    REAL_TYPE x_4 = x_pred[4];
    REAL_TYPE x_5 = x_pred[5];
    REAL_TYPE x_6 = x_pred[6];
    REAL_TYPE x_7 = x_pred[7];
    REAL_TYPE x_8 = x_pred[8];
    REAL_TYPE x_9 = x_pred[9];
    REAL_TYPE x_10 = x_pred[10];
    REAL_TYPE x_11 = x_pred[11];
    REAL_TYPE x_12 = x_pred[12];
    REAL_TYPE x_13 = x_pred[13];
    REAL_TYPE x_14 = x_pred[14];

    value[0] = x_0;
    value[1] = x_1;
    value[2] = x_2;
    value[3] = ((cos( (-x_2)) * (x_3 - x_0)) - (sin( (-x_2)) * (x_4 - x_1)));
    value[4] = ((sin( (-x_2)) * (x_3 - x_0)) + (cos( (-x_2)) * (x_4 - x_1)));
    value[5] = ((cos( (-x_2)) * (x_5 - x_0)) - (sin( (-x_2)) * (x_6 - x_1)));
    value[6] = ((sin( (-x_2)) * (x_5 - x_0)) + (cos( (-x_2)) * (x_6 - x_1)));
    value[7] = ((cos( (-x_2)) * (x_7 - x_0)) - (sin( (-x_2)) * (x_8 - x_1)));
    value[8] = ((sin( (-x_2)) * (x_7 - x_0)) + (cos( (-x_2)) * (x_8 - x_1)));
    value[9] = ((cos( (-x_2)) * (x_9 - x_0)) - (sin( (-x_2)) * (x_10 - x_1)));
    value[10] = ((sin( (-x_2)) * (x_9 - x_0)) + (cos( (-x_2)) * (x_10 - x_1)));
    value[11] = ((cos( (-x_2)) * (x_11 - x_0)) - (sin( (-x_2)) * (x_12 - x_1)));
    value[12] = ((sin( (-x_2)) * (x_11 - x_0)) + (cos( (-x_2)) * (x_12 - x_1)));
    value[13] = ((cos( (-x_2)) * (x_13 - x_0)) - (sin( (-x_2)) * (x_14 - x_1)));
    value[14] = ((sin( (-x_2)) * (x_13 - x_0)) + (cos( (-x_2)) * (x_14 - x_1)));

    H[ 0*this->numEKFStates + 0] = 1;
    H[ 0*this->numEKFStates + 1] = 0;
    H[ 0*this->numEKFStates + 2] = 0;
    H[ 0*this->numEKFStates + 3] = 0;
    H[ 0*this->numEKFStates + 4] = 0;
    H[ 0*this->numEKFStates + 5] = 0;
    H[ 0*this->numEKFStates + 6] = 0;
    H[ 0*this->numEKFStates + 7] = 0;
    H[ 0*this->numEKFStates + 8] = 0;
    H[ 0*this->numEKFStates + 9] = 0;
    H[ 0*this->numEKFStates + 10] = 0;
    H[ 0*this->numEKFStates + 11] = 0;
    H[ 0*this->numEKFStates + 12] = 0;
    H[ 0*this->numEKFStates + 13] = 0;
    H[ 0*this->numEKFStates + 14] = 0;
    H[ 1*this->numEKFStates + 0] = 0;
    H[ 1*this->numEKFStates + 1] = 1;
    H[ 1*this->numEKFStates + 2] = 0;
    H[ 1*this->numEKFStates + 3] = 0;
    H[ 1*this->numEKFStates + 4] = 0;
    H[ 1*this->numEKFStates + 5] = 0;
    H[ 1*this->numEKFStates + 6] = 0;
    H[ 1*this->numEKFStates + 7] = 0;
    H[ 1*this->numEKFStates + 8] = 0;
    H[ 1*this->numEKFStates + 9] = 0;
    H[ 1*this->numEKFStates + 10] = 0;
    H[ 1*this->numEKFStates + 11] = 0;
    H[ 1*this->numEKFStates + 12] = 0;
    H[ 1*this->numEKFStates + 13] = 0;
    H[ 1*this->numEKFStates + 14] = 0;
    H[ 2*this->numEKFStates + 0] = 0;
    H[ 2*this->numEKFStates + 1] = 0;
    H[ 2*this->numEKFStates + 2] = 1;
    H[ 2*this->numEKFStates + 3] = 0;
    H[ 2*this->numEKFStates + 4] = 0;
    H[ 2*this->numEKFStates + 5] = 0;
    H[ 2*this->numEKFStates + 6] = 0;
    H[ 2*this->numEKFStates + 7] = 0;
    H[ 2*this->numEKFStates + 8] = 0;
    H[ 2*this->numEKFStates + 9] = 0;
    H[ 2*this->numEKFStates + 10] = 0;
    H[ 2*this->numEKFStates + 11] = 0;
    H[ 2*this->numEKFStates + 12] = 0;
    H[ 2*this->numEKFStates + 13] = 0;
    H[ 2*this->numEKFStates + 14] = 0;
    H[ 3*this->numEKFStates + 0] =  (-cos( (-x_2)));
    H[ 3*this->numEKFStates + 1] =  (- (-sin( (-x_2))));
    H[ 3*this->numEKFStates + 2] = ((sin( (-x_2)) * (x_3 - x_0)) - ( (-cos( (-x_2))) * (x_4 - x_1)));
    H[ 3*this->numEKFStates + 3] = cos( (-x_2));
    H[ 3*this->numEKFStates + 4] =  (-sin( (-x_2)));
    H[ 3*this->numEKFStates + 5] = 0;
    H[ 3*this->numEKFStates + 6] = 0;
    H[ 3*this->numEKFStates + 7] = 0;
    H[ 3*this->numEKFStates + 8] = 0;
    H[ 3*this->numEKFStates + 9] = 0;
    H[ 3*this->numEKFStates + 10] = 0;
    H[ 3*this->numEKFStates + 11] = 0;
    H[ 3*this->numEKFStates + 12] = 0;
    H[ 3*this->numEKFStates + 13] = 0;
    H[ 3*this->numEKFStates + 14] = 0;
    H[ 4*this->numEKFStates + 0] =  (-sin( (-x_2)));
    H[ 4*this->numEKFStates + 1] =  (-cos( (-x_2)));
    H[ 4*this->numEKFStates + 2] = (( (-cos( (-x_2))) * (x_3 - x_0)) + (sin( (-x_2)) * (x_4 - x_1)));
    H[ 4*this->numEKFStates + 3] = sin( (-x_2));
    H[ 4*this->numEKFStates + 4] = cos( (-x_2));
    H[ 4*this->numEKFStates + 5] = 0;
    H[ 4*this->numEKFStates + 6] = 0;
    H[ 4*this->numEKFStates + 7] = 0;
    H[ 4*this->numEKFStates + 8] = 0;
    H[ 4*this->numEKFStates + 9] = 0;
    H[ 4*this->numEKFStates + 10] = 0;
    H[ 4*this->numEKFStates + 11] = 0;
    H[ 4*this->numEKFStates + 12] = 0;
    H[ 4*this->numEKFStates + 13] = 0;
    H[ 4*this->numEKFStates + 14] = 0;
    H[ 5*this->numEKFStates + 0] =  (-cos( (-x_2)));
    H[ 5*this->numEKFStates + 1] =  (- (-sin( (-x_2))));
    H[ 5*this->numEKFStates + 2] = ((sin( (-x_2)) * (x_5 - x_0)) - ( (-cos( (-x_2))) * (x_6 - x_1)));
    H[ 5*this->numEKFStates + 3] = 0;
    H[ 5*this->numEKFStates + 4] = 0;
    H[ 5*this->numEKFStates + 5] = cos( (-x_2));
    H[ 5*this->numEKFStates + 6] =  (-sin( (-x_2)));
    H[ 5*this->numEKFStates + 7] = 0;
    H[ 5*this->numEKFStates + 8] = 0;
    H[ 5*this->numEKFStates + 9] = 0;
    H[ 5*this->numEKFStates + 10] = 0;
    H[ 5*this->numEKFStates + 11] = 0;
    H[ 5*this->numEKFStates + 12] = 0;
    H[ 5*this->numEKFStates + 13] = 0;
    H[ 5*this->numEKFStates + 14] = 0;
    H[ 6*this->numEKFStates + 0] =  (-sin( (-x_2)));
    H[ 6*this->numEKFStates + 1] =  (-cos( (-x_2)));
    H[ 6*this->numEKFStates + 2] = (( (-cos( (-x_2))) * (x_5 - x_0)) + (sin( (-x_2)) * (x_6 - x_1)));
    H[ 6*this->numEKFStates + 3] = 0;
    H[ 6*this->numEKFStates + 4] = 0;
    H[ 6*this->numEKFStates + 5] = sin( (-x_2));
    H[ 6*this->numEKFStates + 6] = cos( (-x_2));
    H[ 6*this->numEKFStates + 7] = 0;
    H[ 6*this->numEKFStates + 8] = 0;
    H[ 6*this->numEKFStates + 9] = 0;
    H[ 6*this->numEKFStates + 10] = 0;
    H[ 6*this->numEKFStates + 11] = 0;
    H[ 6*this->numEKFStates + 12] = 0;
    H[ 6*this->numEKFStates + 13] = 0;
    H[ 6*this->numEKFStates + 14] = 0;
    H[ 7*this->numEKFStates + 0] =  (-cos( (-x_2)));
    H[ 7*this->numEKFStates + 1] =  (- (-sin( (-x_2))));
    H[ 7*this->numEKFStates + 2] = ((sin( (-x_2)) * (x_7 - x_0)) - ( (-cos( (-x_2))) * (x_8 - x_1)));
    H[ 7*this->numEKFStates + 3] = 0;
    H[ 7*this->numEKFStates + 4] = 0;
    H[ 7*this->numEKFStates + 5] = 0;
    H[ 7*this->numEKFStates + 6] = 0;
    H[ 7*this->numEKFStates + 7] = cos( (-x_2));
    H[ 7*this->numEKFStates + 8] =  (-sin( (-x_2)));
    H[ 7*this->numEKFStates + 9] = 0;
    H[ 7*this->numEKFStates + 10] = 0;
    H[ 7*this->numEKFStates + 11] = 0;
    H[ 7*this->numEKFStates + 12] = 0;
    H[ 7*this->numEKFStates + 13] = 0;
    H[ 7*this->numEKFStates + 14] = 0;
    H[ 8*this->numEKFStates + 0] =  (-sin( (-x_2)));
    H[ 8*this->numEKFStates + 1] =  (-cos( (-x_2)));
    H[ 8*this->numEKFStates + 2] = (( (-cos( (-x_2))) * (x_7 - x_0)) + (sin( (-x_2)) * (x_8 - x_1)));
    H[ 8*this->numEKFStates + 3] = 0;
    H[ 8*this->numEKFStates + 4] = 0;
    H[ 8*this->numEKFStates + 5] = 0;
    H[ 8*this->numEKFStates + 6] = 0;
    H[ 8*this->numEKFStates + 7] = sin( (-x_2));
    H[ 8*this->numEKFStates + 8] = cos( (-x_2));
    H[ 8*this->numEKFStates + 9] = 0;
    H[ 8*this->numEKFStates + 10] = 0;
    H[ 8*this->numEKFStates + 11] = 0;
    H[ 8*this->numEKFStates + 12] = 0;
    H[ 8*this->numEKFStates + 13] = 0;
    H[ 8*this->numEKFStates + 14] = 0;
    H[ 9*this->numEKFStates + 0] =  (-cos( (-x_2)));
    H[ 9*this->numEKFStates + 1] =  (- (-sin( (-x_2))));
    H[ 9*this->numEKFStates + 2] = ((sin( (-x_2)) * (x_9 - x_0)) - ( (-cos( (-x_2))) * (x_10 - x_1)));
    H[ 9*this->numEKFStates + 3] = 0;
    H[ 9*this->numEKFStates + 4] = 0;
    H[ 9*this->numEKFStates + 5] = 0;
    H[ 9*this->numEKFStates + 6] = 0;
    H[ 9*this->numEKFStates + 7] = 0;
    H[ 9*this->numEKFStates + 8] = 0;
    H[ 9*this->numEKFStates + 9] = cos( (-x_2));
    H[ 9*this->numEKFStates + 10] =  (-sin( (-x_2)));
    H[ 9*this->numEKFStates + 11] = 0;
    H[ 9*this->numEKFStates + 12] = 0;
    H[ 9*this->numEKFStates + 13] = 0;
    H[ 9*this->numEKFStates + 14] = 0;
    H[ 10*this->numEKFStates + 0] =  (-sin( (-x_2)));
    H[ 10*this->numEKFStates + 1] =  (-cos( (-x_2)));
    H[ 10*this->numEKFStates + 2] = (( (-cos( (-x_2))) * (x_9 - x_0)) + (sin( (-x_2)) * (x_10 - x_1)));
    H[ 10*this->numEKFStates + 3] = 0;
    H[ 10*this->numEKFStates + 4] = 0;
    H[ 10*this->numEKFStates + 5] = 0;
    H[ 10*this->numEKFStates + 6] = 0;
    H[ 10*this->numEKFStates + 7] = 0;
    H[ 10*this->numEKFStates + 8] = 0;
    H[ 10*this->numEKFStates + 9] = sin( (-x_2));
    H[ 10*this->numEKFStates + 10] = cos( (-x_2));
    H[ 10*this->numEKFStates + 11] = 0;
    H[ 10*this->numEKFStates + 12] = 0;
    H[ 10*this->numEKFStates + 13] = 0;
    H[ 10*this->numEKFStates + 14] = 0;
    H[ 11*this->numEKFStates + 0] =  (-cos( (-x_2)));
    H[ 11*this->numEKFStates + 1] =  (- (-sin( (-x_2))));
    H[ 11*this->numEKFStates + 2] = ((sin( (-x_2)) * (x_11 - x_0)) - ( (-cos( (-x_2))) * (x_12 - x_1)));
    H[ 11*this->numEKFStates + 3] = 0;
    H[ 11*this->numEKFStates + 4] = 0;
    H[ 11*this->numEKFStates + 5] = 0;
    H[ 11*this->numEKFStates + 6] = 0;
    H[ 11*this->numEKFStates + 7] = 0;
    H[ 11*this->numEKFStates + 8] = 0;
    H[ 11*this->numEKFStates + 9] = 0;
    H[ 11*this->numEKFStates + 10] = 0;
    H[ 11*this->numEKFStates + 11] = cos( (-x_2));
    H[ 11*this->numEKFStates + 12] =  (-sin( (-x_2)));
    H[ 11*this->numEKFStates + 13] = 0;
    H[ 11*this->numEKFStates + 14] = 0;
    H[ 12*this->numEKFStates + 0] =  (-sin( (-x_2)));
    H[ 12*this->numEKFStates + 1] =  (-cos( (-x_2)));
    H[ 12*this->numEKFStates + 2] = (( (-cos( (-x_2))) * (x_11 - x_0)) + (sin( (-x_2)) * (x_12 - x_1)));
    H[ 12*this->numEKFStates + 3] = 0;
    H[ 12*this->numEKFStates + 4] = 0;
    H[ 12*this->numEKFStates + 5] = 0;
    H[ 12*this->numEKFStates + 6] = 0;
    H[ 12*this->numEKFStates + 7] = 0;
    H[ 12*this->numEKFStates + 8] = 0;
    H[ 12*this->numEKFStates + 9] = 0;
    H[ 12*this->numEKFStates + 10] = 0;
    H[ 12*this->numEKFStates + 11] = sin( (-x_2));
    H[ 12*this->numEKFStates + 12] = cos( (-x_2));
    H[ 12*this->numEKFStates + 13] = 0;
    H[ 12*this->numEKFStates + 14] = 0;
    H[ 13*this->numEKFStates + 0] =  (-cos( (-x_2)));
    H[ 13*this->numEKFStates + 1] =  (- (-sin( (-x_2))));
    H[ 13*this->numEKFStates + 2] = ((sin( (-x_2)) * (x_13 - x_0)) - ( (-cos( (-x_2))) * (x_14 - x_1)));
    H[ 13*this->numEKFStates + 3] = 0;
    H[ 13*this->numEKFStates + 4] = 0;
    H[ 13*this->numEKFStates + 5] = 0;
    H[ 13*this->numEKFStates + 6] = 0;
    H[ 13*this->numEKFStates + 7] = 0;
    H[ 13*this->numEKFStates + 8] = 0;
    H[ 13*this->numEKFStates + 9] = 0;
    H[ 13*this->numEKFStates + 10] = 0;
    H[ 13*this->numEKFStates + 11] = 0;
    H[ 13*this->numEKFStates + 12] = 0;
    H[ 13*this->numEKFStates + 13] = cos( (-x_2));
    H[ 13*this->numEKFStates + 14] =  (-sin( (-x_2)));
    H[ 14*this->numEKFStates + 0] =  (-sin( (-x_2)));
    H[ 14*this->numEKFStates + 1] =  (-cos( (-x_2)));
    H[ 14*this->numEKFStates + 2] = (( (-cos( (-x_2))) * (x_13 - x_0)) + (sin( (-x_2)) * (x_14 - x_1)));
    H[ 14*this->numEKFStates + 3] = 0;
    H[ 14*this->numEKFStates + 4] = 0;
    H[ 14*this->numEKFStates + 5] = 0;
    H[ 14*this->numEKFStates + 6] = 0;
    H[ 14*this->numEKFStates + 7] = 0;
    H[ 14*this->numEKFStates + 8] = 0;
    H[ 14*this->numEKFStates + 9] = 0;
    H[ 14*this->numEKFStates + 10] = 0;
    H[ 14*this->numEKFStates + 11] = 0;
    H[ 14*this->numEKFStates + 12] = 0;
    H[ 14*this->numEKFStates + 13] = sin( (-x_2));
    H[ 14*this->numEKFStates + 14] = cos( (-x_2));
}

}

void VehicleEKF::predictStateAndDerivatives(REAL_TYPE *x_pred, REAL_TYPE *A, REAL_TYPE *B)
{
    REAL_TYPE *x_temp = new REAL_TYPE [this->numStates];
    REAL_TYPE *A_temp = new REAL_TYPE [this->numStates*this->numStates];
    REAL_TYPE *B_temp = new REAL_TYPE [this->numStates*this->numControls];

    //transfer states
    //update dynamics states
    integrateSystemSensitivitiesOneStep(x_temp, this->x, this->u, A_temp, B_temp);
    x_pred[0] = x_temp[0];
    x_pred[1] = x_temp[1];
    x_pred[2] = x_temp[2];
    x_pred[3] = x_temp[3];
    x_pred[4] = x_temp[4];
    x_pred[5] = x_temp[5];
    x_pred[6] = x_temp[6];
    x_pred[7] = x_temp[7];
    x_pred[8] = x_temp[8];
    x_pred[9] = x_temp[9];
    x_pred[10] = x_temp[10];
    x_pred[11] = x_temp[11];
    x_pred[12] = x_temp[12];
    x_pred[13] = x_temp[13];
    x_pred[14] = x_temp[14];

    //calculate prediction sensitivities A and B
    for(int i = 0; i < this->numEKFStates*this->numEKFStates; i++)
        A[i] = 0.0;

    for(int i = 0; i < this->numEKFStates*this->numControls; i++)
        B[i] = 0.0;

    for(int i = 0; i < this->numStates; i++)
        for(int j = 0; j < this->numStates; j++)
            A[i*this->numEKFStates + j] = A_temp[i*this->numStates + j];

    for(int i = 0; i < this->numStates; i++)
        for(int j = 0; j < this->numControls; j++)
            B[i*this->numControls + j] = B_temp[i*this->numControls + j];

    delete[] x_temp;
    delete[] A_temp;
    delete[] B_temp;
}

void VehicleEKF::initializeIntegrator()
{
    ALin = new REAL_TYPE[numStates*numStates];
    BLin = new REAL_TYPE[numStates*numControls];

    for(int i = 0; i < numStates*numStates; i++)
        ALin[i] = 0;
    for(int i = 0; i < numStates*numControls; i++)
        BLin[i] = 0;

    /*Initialize constant parts*/
    /*ALin = df(x,u)/dx*/
    /*BLin = df(x,u)/du*/
    evaluateDynamicsLinearDerivatives(ALin, BLin);


}

void VehicleEKF::integrateSystemSensitivitiesOneStep(REAL_TYPE *x_new, REAL_TYPE *x, REAL_TYPE *u, REAL_TYPE *A, REAL_TYPE *B)
{
    /*x_new = f(x,u)*/
    /*A = df(x,u)/dx*/
    /*B = df(x,u)/du*/
    evaluateDynamicsAndDerivatives(x_new, A, B, x, u);
    /*Copy linear parts from ALin and BLin*/
    for(int i = 0; i < numLinearStates; i++)
    {
        for(int j = 0; j < numStates; j++)
            A[i*numStates + j] = ALin[i*numStates + j];
        for(int j = 0; j < numControls; j++)
            B[i*numControls + j] = BLin[i*numControls + j];
    }

}

