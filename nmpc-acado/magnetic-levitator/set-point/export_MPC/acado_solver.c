/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 80; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 3 + 2];

acadoWorkspace.state[15] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 3] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 3 + 3];
acadoWorkspace.d[lRun1 * 3 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 3 + 4];
acadoWorkspace.d[lRun1 * 3 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 3 + 5];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[11];

acadoWorkspace.evGu[lRun1 * 3] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 3 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGu[lRun1 * 3 + 2] = acadoWorkspace.state[14];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[4];
tmpQ1[4] = + tmpQ2[5];
tmpQ1[5] = + tmpQ2[6];
tmpQ1[6] = + tmpQ2[8];
tmpQ1[7] = + tmpQ2[9];
tmpQ1[8] = + tmpQ2[10];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[12];
tmpR2[1] = +tmpObjS[13];
tmpR2[2] = +tmpObjS[14];
tmpR2[3] = +tmpObjS[15];
tmpR1[0] = + tmpR2[3];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 80; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 4] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 4 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 4 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 4 + 3] = acadoWorkspace.objValueOut[3];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 9 ]), &(acadoWorkspace.Q2[ runObj * 12 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj ]), &(acadoWorkspace.R2[ runObj * 4 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[240];
acadoWorkspace.objValueIn[1] = acadoVariables.x[241];
acadoWorkspace.objValueIn[2] = acadoVariables.x[242];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2];
Gu2[1] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[2];
Gu2[2] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[8]*Gu1[2];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 81] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + R11[0];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[3]*Gu1[1] + Gx1[6]*Gu1[2];
Gu2[1] = + Gx1[1]*Gu1[0] + Gx1[4]*Gu1[1] + Gx1[7]*Gu1[2];
Gu2[2] = + Gx1[2]*Gu1[0] + Gx1[5]*Gu1[1] + Gx1[8]*Gu1[2];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[1] + Q11[2]*Gu1[2] + Gu2[0];
Gu3[1] = + Q11[3]*Gu1[0] + Q11[4]*Gu1[1] + Q11[5]*Gu1[2] + Gu2[1];
Gu3[2] = + Q11[6]*Gu1[0] + Q11[7]*Gu1[1] + Q11[8]*Gu1[2] + Gu2[2];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[3]*w11[1] + Gx1[6]*w11[2] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[4]*w11[1] + Gx1[7]*w11[2] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[5]*w11[1] + Gx1[8]*w11[2] + w12[2];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[1]*w11[1] + Gu1[2]*w11[2];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + w12[0];
w13[1] = + Q11[3]*w11[0] + Q11[4]*w11[1] + Q11[5]*w11[2] + w12[1];
w13[2] = + Q11[6]*w11[0] + Q11[7]*w11[1] + Q11[8]*w11[2] + w12[2];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2];
w12[1] += + Gx1[3]*w11[0] + Gx1[4]*w11[1] + Gx1[5]*w11[2];
w12[2] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2];
w12[1] += + Gx1[3]*w11[0] + Gx1[4]*w11[1] + Gx1[5]*w11[2];
w12[2] += + Gx1[6]*w11[0] + Gx1[7]*w11[1] + Gx1[8]*w11[2];
w12[0] += + Gu1[0]*U1[0];
w12[1] += + Gu1[1]*U1[0];
w12[2] += + Gu1[2]*U1[0];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol)] = acadoWorkspace.H[(iCol * 80) + (iRow)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3];
QDy1[1] = + Q2[4]*Dy1[0] + Q2[5]*Dy1[1] + Q2[6]*Dy1[2] + Q2[7]*Dy1[3];
QDy1[2] = + Q2[8]*Dy1[0] + Q2[9]*Dy1[1] + Q2[10]*Dy1[2] + Q2[11]*Dy1[3];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 80 */
static const int xBoundIndices[ 80 ] = 
{ 4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 52, 55, 58, 61, 64, 67, 70, 73, 76, 79, 82, 85, 88, 91, 94, 97, 100, 103, 106, 109, 112, 115, 118, 121, 124, 127, 130, 133, 136, 139, 142, 145, 148, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 184, 187, 190, 193, 196, 199, 202, 205, 208, 211, 214, 217, 220, 223, 226, 229, 232, 235, 238, 241 };
for (lRun2 = 0; lRun2 < 80; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 161)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 3 ]), &(acadoWorkspace.E[ lRun3 * 3 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 80; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (3)) * (3)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (3)) * (1)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (3)) * (1)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (80)) - (1)) * (3)) * (1)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 79; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 3 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 9 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 9 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (3)) * (1)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 ]), &(acadoWorkspace.evGu[ lRun2 * 3 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 80; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 240; ++lRun1)
acadoWorkspace.sbar[lRun1 + 3] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = - acadoVariables.u[0];
acadoWorkspace.lb[1] = - acadoVariables.u[1];
acadoWorkspace.lb[2] = - acadoVariables.u[2];
acadoWorkspace.lb[3] = - acadoVariables.u[3];
acadoWorkspace.lb[4] = - acadoVariables.u[4];
acadoWorkspace.lb[5] = - acadoVariables.u[5];
acadoWorkspace.lb[6] = - acadoVariables.u[6];
acadoWorkspace.lb[7] = - acadoVariables.u[7];
acadoWorkspace.lb[8] = - acadoVariables.u[8];
acadoWorkspace.lb[9] = - acadoVariables.u[9];
acadoWorkspace.lb[10] = - acadoVariables.u[10];
acadoWorkspace.lb[11] = - acadoVariables.u[11];
acadoWorkspace.lb[12] = - acadoVariables.u[12];
acadoWorkspace.lb[13] = - acadoVariables.u[13];
acadoWorkspace.lb[14] = - acadoVariables.u[14];
acadoWorkspace.lb[15] = - acadoVariables.u[15];
acadoWorkspace.lb[16] = - acadoVariables.u[16];
acadoWorkspace.lb[17] = - acadoVariables.u[17];
acadoWorkspace.lb[18] = - acadoVariables.u[18];
acadoWorkspace.lb[19] = - acadoVariables.u[19];
acadoWorkspace.lb[20] = - acadoVariables.u[20];
acadoWorkspace.lb[21] = - acadoVariables.u[21];
acadoWorkspace.lb[22] = - acadoVariables.u[22];
acadoWorkspace.lb[23] = - acadoVariables.u[23];
acadoWorkspace.lb[24] = - acadoVariables.u[24];
acadoWorkspace.lb[25] = - acadoVariables.u[25];
acadoWorkspace.lb[26] = - acadoVariables.u[26];
acadoWorkspace.lb[27] = - acadoVariables.u[27];
acadoWorkspace.lb[28] = - acadoVariables.u[28];
acadoWorkspace.lb[29] = - acadoVariables.u[29];
acadoWorkspace.lb[30] = - acadoVariables.u[30];
acadoWorkspace.lb[31] = - acadoVariables.u[31];
acadoWorkspace.lb[32] = - acadoVariables.u[32];
acadoWorkspace.lb[33] = - acadoVariables.u[33];
acadoWorkspace.lb[34] = - acadoVariables.u[34];
acadoWorkspace.lb[35] = - acadoVariables.u[35];
acadoWorkspace.lb[36] = - acadoVariables.u[36];
acadoWorkspace.lb[37] = - acadoVariables.u[37];
acadoWorkspace.lb[38] = - acadoVariables.u[38];
acadoWorkspace.lb[39] = - acadoVariables.u[39];
acadoWorkspace.lb[40] = - acadoVariables.u[40];
acadoWorkspace.lb[41] = - acadoVariables.u[41];
acadoWorkspace.lb[42] = - acadoVariables.u[42];
acadoWorkspace.lb[43] = - acadoVariables.u[43];
acadoWorkspace.lb[44] = - acadoVariables.u[44];
acadoWorkspace.lb[45] = - acadoVariables.u[45];
acadoWorkspace.lb[46] = - acadoVariables.u[46];
acadoWorkspace.lb[47] = - acadoVariables.u[47];
acadoWorkspace.lb[48] = - acadoVariables.u[48];
acadoWorkspace.lb[49] = - acadoVariables.u[49];
acadoWorkspace.lb[50] = - acadoVariables.u[50];
acadoWorkspace.lb[51] = - acadoVariables.u[51];
acadoWorkspace.lb[52] = - acadoVariables.u[52];
acadoWorkspace.lb[53] = - acadoVariables.u[53];
acadoWorkspace.lb[54] = - acadoVariables.u[54];
acadoWorkspace.lb[55] = - acadoVariables.u[55];
acadoWorkspace.lb[56] = - acadoVariables.u[56];
acadoWorkspace.lb[57] = - acadoVariables.u[57];
acadoWorkspace.lb[58] = - acadoVariables.u[58];
acadoWorkspace.lb[59] = - acadoVariables.u[59];
acadoWorkspace.lb[60] = - acadoVariables.u[60];
acadoWorkspace.lb[61] = - acadoVariables.u[61];
acadoWorkspace.lb[62] = - acadoVariables.u[62];
acadoWorkspace.lb[63] = - acadoVariables.u[63];
acadoWorkspace.lb[64] = - acadoVariables.u[64];
acadoWorkspace.lb[65] = - acadoVariables.u[65];
acadoWorkspace.lb[66] = - acadoVariables.u[66];
acadoWorkspace.lb[67] = - acadoVariables.u[67];
acadoWorkspace.lb[68] = - acadoVariables.u[68];
acadoWorkspace.lb[69] = - acadoVariables.u[69];
acadoWorkspace.lb[70] = - acadoVariables.u[70];
acadoWorkspace.lb[71] = - acadoVariables.u[71];
acadoWorkspace.lb[72] = - acadoVariables.u[72];
acadoWorkspace.lb[73] = - acadoVariables.u[73];
acadoWorkspace.lb[74] = - acadoVariables.u[74];
acadoWorkspace.lb[75] = - acadoVariables.u[75];
acadoWorkspace.lb[76] = - acadoVariables.u[76];
acadoWorkspace.lb[77] = - acadoVariables.u[77];
acadoWorkspace.lb[78] = - acadoVariables.u[78];
acadoWorkspace.lb[79] = - acadoVariables.u[79];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+001 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+001 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+001 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+001 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+001 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+001 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+001 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+001 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+001 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+001 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+001 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+001 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+001 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+001 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+001 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+001 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+001 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+001 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+001 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+001 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+001 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+001 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+001 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+001 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+001 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+001 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+001 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+001 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+001 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+001 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+001 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+001 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)1.0000000000000000e+001 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.0000000000000000e+001 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+001 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+001 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)1.0000000000000000e+001 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e+001 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+001 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.0000000000000000e+001 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)1.0000000000000000e+001 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)1.0000000000000000e+001 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e+001 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.0000000000000000e+001 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)1.0000000000000000e+001 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)1.0000000000000000e+001 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e+001 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)1.0000000000000000e+001 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)1.0000000000000000e+001 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)1.0000000000000000e+001 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)1.0000000000000000e+001 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)1.0000000000000000e+001 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)1.0000000000000000e+001 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)1.0000000000000000e+001 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)1.0000000000000000e+001 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)1.0000000000000000e+001 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)1.0000000000000000e+001 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)1.0000000000000000e+001 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)1.0000000000000000e+001 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)1.0000000000000000e+001 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)1.0000000000000000e+001 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)1.0000000000000000e+001 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)1.0000000000000000e+001 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)1.0000000000000000e+001 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)1.0000000000000000e+001 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)1.0000000000000000e+001 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)1.0000000000000000e+001 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)1.0000000000000000e+001 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)1.0000000000000000e+001 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)1.0000000000000000e+001 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)1.0000000000000000e+001 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)1.0000000000000000e+001 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)1.0000000000000000e+001 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)1.0000000000000000e+001 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)1.0000000000000000e+001 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)1.0000000000000000e+001 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)1.0000000000000000e+001 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)1.0000000000000000e+001 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)1.0000000000000000e+001 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)1.0000000000000000e+001 - acadoVariables.u[79];

for (lRun1 = 0; lRun1 < 80; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 3;
lRun4 = ((lRun3) / (3)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 159)) / (2)) + (lRun4)) - (1)) * (3)) + ((lRun3) % (3));
acadoWorkspace.A[(lRun1 * 80) + (lRun2)] = acadoWorkspace.E[lRun5];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
for (lRun1 = 0; lRun1 < 320; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 4 ]), &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 8 ]), &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 12 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 16 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 20 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 28 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 32 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 44 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 52 ]), &(acadoWorkspace.Dy[ 52 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 56 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 64 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 68 ]), &(acadoWorkspace.Dy[ 68 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 76 ]), &(acadoWorkspace.Dy[ 76 ]), &(acadoWorkspace.g[ 19 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 84 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 88 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 92 ]), &(acadoWorkspace.Dy[ 92 ]), &(acadoWorkspace.g[ 23 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 100 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 25 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 104 ]), &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 108 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 112 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 116 ]), &(acadoWorkspace.Dy[ 116 ]), &(acadoWorkspace.g[ 29 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 124 ]), &(acadoWorkspace.Dy[ 124 ]), &(acadoWorkspace.g[ 31 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 128 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 132 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 136 ]), &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 140 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 35 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 148 ]), &(acadoWorkspace.Dy[ 148 ]), &(acadoWorkspace.g[ 37 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 152 ]), &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 156 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 160 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 164 ]), &(acadoWorkspace.Dy[ 164 ]), &(acadoWorkspace.g[ 41 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 168 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 172 ]), &(acadoWorkspace.Dy[ 172 ]), &(acadoWorkspace.g[ 43 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 176 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 184 ]), &(acadoWorkspace.Dy[ 184 ]), &(acadoWorkspace.g[ 46 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 188 ]), &(acadoWorkspace.Dy[ 188 ]), &(acadoWorkspace.g[ 47 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 196 ]), &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.g[ 49 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 200 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.g[ 50 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 204 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 208 ]), &(acadoWorkspace.Dy[ 208 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 212 ]), &(acadoWorkspace.Dy[ 212 ]), &(acadoWorkspace.g[ 53 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 220 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.g[ 55 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 224 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 228 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 232 ]), &(acadoWorkspace.Dy[ 232 ]), &(acadoWorkspace.g[ 58 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 236 ]), &(acadoWorkspace.Dy[ 236 ]), &(acadoWorkspace.g[ 59 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 244 ]), &(acadoWorkspace.Dy[ 244 ]), &(acadoWorkspace.g[ 61 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 248 ]), &(acadoWorkspace.Dy[ 248 ]), &(acadoWorkspace.g[ 62 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 256 ]), &(acadoWorkspace.Dy[ 256 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 260 ]), &(acadoWorkspace.Dy[ 260 ]), &(acadoWorkspace.g[ 65 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 264 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 268 ]), &(acadoWorkspace.Dy[ 268 ]), &(acadoWorkspace.g[ 67 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 272 ]), &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 276 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.g[ 70 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 284 ]), &(acadoWorkspace.Dy[ 284 ]), &(acadoWorkspace.g[ 71 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 292 ]), &(acadoWorkspace.Dy[ 292 ]), &(acadoWorkspace.g[ 73 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 296 ]), &(acadoWorkspace.Dy[ 296 ]), &(acadoWorkspace.g[ 74 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 300 ]), &(acadoWorkspace.Dy[ 300 ]), &(acadoWorkspace.g[ 75 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 304 ]), &(acadoWorkspace.Dy[ 304 ]), &(acadoWorkspace.g[ 76 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 308 ]), &(acadoWorkspace.Dy[ 308 ]), &(acadoWorkspace.g[ 77 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 312 ]), &(acadoWorkspace.Dy[ 312 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 316 ]), &(acadoWorkspace.Dy[ 316 ]), &(acadoWorkspace.g[ 79 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 12 ]), &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 24 ]), &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 72 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 84 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 96 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 132 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.QDy[ 33 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 144 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 156 ]), &(acadoWorkspace.Dy[ 52 ]), &(acadoWorkspace.QDy[ 39 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 168 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 192 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 204 ]), &(acadoWorkspace.Dy[ 68 ]), &(acadoWorkspace.QDy[ 51 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 216 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 228 ]), &(acadoWorkspace.Dy[ 76 ]), &(acadoWorkspace.QDy[ 57 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 264 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 66 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 276 ]), &(acadoWorkspace.Dy[ 92 ]), &(acadoWorkspace.QDy[ 69 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 288 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 300 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 75 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 312 ]), &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.QDy[ 78 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 324 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 336 ]), &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 348 ]), &(acadoWorkspace.Dy[ 116 ]), &(acadoWorkspace.QDy[ 87 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 372 ]), &(acadoWorkspace.Dy[ 124 ]), &(acadoWorkspace.QDy[ 93 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 384 ]), &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 396 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 408 ]), &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.QDy[ 102 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 105 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 444 ]), &(acadoWorkspace.Dy[ 148 ]), &(acadoWorkspace.QDy[ 111 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 456 ]), &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.QDy[ 114 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 468 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 492 ]), &(acadoWorkspace.Dy[ 164 ]), &(acadoWorkspace.QDy[ 123 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 504 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 516 ]), &(acadoWorkspace.Dy[ 172 ]), &(acadoWorkspace.QDy[ 129 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 528 ]), &(acadoWorkspace.Dy[ 176 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 552 ]), &(acadoWorkspace.Dy[ 184 ]), &(acadoWorkspace.QDy[ 138 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 564 ]), &(acadoWorkspace.Dy[ 188 ]), &(acadoWorkspace.QDy[ 141 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 576 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 588 ]), &(acadoWorkspace.Dy[ 196 ]), &(acadoWorkspace.QDy[ 147 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 600 ]), &(acadoWorkspace.Dy[ 200 ]), &(acadoWorkspace.QDy[ 150 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 612 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 153 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 624 ]), &(acadoWorkspace.Dy[ 208 ]), &(acadoWorkspace.QDy[ 156 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 636 ]), &(acadoWorkspace.Dy[ 212 ]), &(acadoWorkspace.QDy[ 159 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 648 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 162 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 660 ]), &(acadoWorkspace.Dy[ 220 ]), &(acadoWorkspace.QDy[ 165 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 672 ]), &(acadoWorkspace.Dy[ 224 ]), &(acadoWorkspace.QDy[ 168 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 684 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.QDy[ 171 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 696 ]), &(acadoWorkspace.Dy[ 232 ]), &(acadoWorkspace.QDy[ 174 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 708 ]), &(acadoWorkspace.Dy[ 236 ]), &(acadoWorkspace.QDy[ 177 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 240 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 732 ]), &(acadoWorkspace.Dy[ 244 ]), &(acadoWorkspace.QDy[ 183 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 744 ]), &(acadoWorkspace.Dy[ 248 ]), &(acadoWorkspace.QDy[ 186 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 756 ]), &(acadoWorkspace.Dy[ 252 ]), &(acadoWorkspace.QDy[ 189 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 768 ]), &(acadoWorkspace.Dy[ 256 ]), &(acadoWorkspace.QDy[ 192 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 780 ]), &(acadoWorkspace.Dy[ 260 ]), &(acadoWorkspace.QDy[ 195 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 792 ]), &(acadoWorkspace.Dy[ 264 ]), &(acadoWorkspace.QDy[ 198 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 804 ]), &(acadoWorkspace.Dy[ 268 ]), &(acadoWorkspace.QDy[ 201 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 816 ]), &(acadoWorkspace.Dy[ 272 ]), &(acadoWorkspace.QDy[ 204 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 828 ]), &(acadoWorkspace.Dy[ 276 ]), &(acadoWorkspace.QDy[ 207 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 840 ]), &(acadoWorkspace.Dy[ 280 ]), &(acadoWorkspace.QDy[ 210 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 852 ]), &(acadoWorkspace.Dy[ 284 ]), &(acadoWorkspace.QDy[ 213 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 288 ]), &(acadoWorkspace.QDy[ 216 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 876 ]), &(acadoWorkspace.Dy[ 292 ]), &(acadoWorkspace.QDy[ 219 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 888 ]), &(acadoWorkspace.Dy[ 296 ]), &(acadoWorkspace.QDy[ 222 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 900 ]), &(acadoWorkspace.Dy[ 300 ]), &(acadoWorkspace.QDy[ 225 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 912 ]), &(acadoWorkspace.Dy[ 304 ]), &(acadoWorkspace.QDy[ 228 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 924 ]), &(acadoWorkspace.Dy[ 308 ]), &(acadoWorkspace.QDy[ 231 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 936 ]), &(acadoWorkspace.Dy[ 312 ]), &(acadoWorkspace.QDy[ 234 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 948 ]), &(acadoWorkspace.Dy[ 316 ]), &(acadoWorkspace.QDy[ 237 ]) );

acadoWorkspace.QDy[240] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[241] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[242] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 3 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 51 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 57 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 69 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 75 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 87 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 93 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 111 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 123 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 129 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 141 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 159 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 477 ]), &(acadoWorkspace.sbar[ 159 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 165 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 495 ]), &(acadoWorkspace.sbar[ 165 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 174 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 522 ]), &(acadoWorkspace.sbar[ 174 ]), &(acadoWorkspace.sbar[ 177 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 531 ]), &(acadoWorkspace.sbar[ 177 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 183 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 549 ]), &(acadoWorkspace.sbar[ 183 ]), &(acadoWorkspace.sbar[ 186 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 558 ]), &(acadoWorkspace.sbar[ 186 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 195 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 585 ]), &(acadoWorkspace.sbar[ 195 ]), &(acadoWorkspace.sbar[ 198 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 594 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.sbar[ 201 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 603 ]), &(acadoWorkspace.sbar[ 201 ]), &(acadoWorkspace.sbar[ 204 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.sbar[ 204 ]), &(acadoWorkspace.sbar[ 207 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 621 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.sbar[ 210 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 630 ]), &(acadoWorkspace.sbar[ 210 ]), &(acadoWorkspace.sbar[ 213 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 639 ]), &(acadoWorkspace.sbar[ 213 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 219 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 657 ]), &(acadoWorkspace.sbar[ 219 ]), &(acadoWorkspace.sbar[ 222 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 666 ]), &(acadoWorkspace.sbar[ 222 ]), &(acadoWorkspace.sbar[ 225 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 675 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.sbar[ 228 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.sbar[ 228 ]), &(acadoWorkspace.sbar[ 231 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 693 ]), &(acadoWorkspace.sbar[ 231 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 702 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 237 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 711 ]), &(acadoWorkspace.sbar[ 237 ]), &(acadoWorkspace.sbar[ 240 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[240] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[241] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[242] + acadoWorkspace.QDy[240];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[240] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[241] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[242] + acadoWorkspace.QDy[241];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[240] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[241] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[242] + acadoWorkspace.QDy[242];
acado_macBTw1( &(acadoWorkspace.evGu[ 237 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 79 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 711 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 237 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 711 ]), &(acadoWorkspace.sbar[ 237 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 702 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 234 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 702 ]), &(acadoWorkspace.sbar[ 234 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 231 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 77 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 693 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 231 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 693 ]), &(acadoWorkspace.sbar[ 231 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 228 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 684 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 228 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 684 ]), &(acadoWorkspace.sbar[ 228 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 225 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 75 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 675 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 225 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 675 ]), &(acadoWorkspace.sbar[ 225 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 222 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 74 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 666 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 222 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 666 ]), &(acadoWorkspace.sbar[ 222 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 219 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 73 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 657 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 219 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 657 ]), &(acadoWorkspace.sbar[ 219 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 216 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 216 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 213 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 71 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 639 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 213 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 639 ]), &(acadoWorkspace.sbar[ 213 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 210 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 70 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 630 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 210 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 630 ]), &(acadoWorkspace.sbar[ 210 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 207 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 69 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 621 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 207 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 621 ]), &(acadoWorkspace.sbar[ 207 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 204 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 612 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 204 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 612 ]), &(acadoWorkspace.sbar[ 204 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 201 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 67 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 603 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 201 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 603 ]), &(acadoWorkspace.sbar[ 201 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 594 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 198 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 594 ]), &(acadoWorkspace.sbar[ 198 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 195 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 65 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 585 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 195 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 585 ]), &(acadoWorkspace.sbar[ 195 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 192 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 192 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 63 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 189 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.sbar[ 189 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 186 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 62 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 558 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 186 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 558 ]), &(acadoWorkspace.sbar[ 186 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 183 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 61 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 549 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 183 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 549 ]), &(acadoWorkspace.sbar[ 183 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 540 ]), &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 177 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 59 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 531 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 177 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 531 ]), &(acadoWorkspace.sbar[ 177 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 174 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 58 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 522 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 174 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 522 ]), &(acadoWorkspace.sbar[ 174 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 171 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 168 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 504 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 168 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 504 ]), &(acadoWorkspace.sbar[ 168 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 165 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 55 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 495 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 165 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 495 ]), &(acadoWorkspace.sbar[ 165 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 159 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 53 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 477 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 159 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 477 ]), &(acadoWorkspace.sbar[ 159 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 468 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 468 ]), &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 153 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 150 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 50 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 450 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 150 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 450 ]), &(acadoWorkspace.sbar[ 150 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 147 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 49 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 441 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 147 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 441 ]), &(acadoWorkspace.sbar[ 147 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 141 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 47 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 423 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 141 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 423 ]), &(acadoWorkspace.sbar[ 141 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 138 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 46 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 414 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 138 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 414 ]), &(acadoWorkspace.sbar[ 138 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 132 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 396 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 396 ]), &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 129 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 43 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 387 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 129 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 387 ]), &(acadoWorkspace.sbar[ 129 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 123 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 41 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 369 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 123 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 369 ]), &(acadoWorkspace.sbar[ 123 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 360 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 360 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 117 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 114 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 342 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 114 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 342 ]), &(acadoWorkspace.sbar[ 114 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 111 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 37 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 333 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 111 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 333 ]), &(acadoWorkspace.sbar[ 111 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 105 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 35 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 315 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 105 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 315 ]), &(acadoWorkspace.sbar[ 105 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 102 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 306 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 102 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 306 ]), &(acadoWorkspace.sbar[ 102 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 99 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 93 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 31 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 279 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 93 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 279 ]), &(acadoWorkspace.sbar[ 93 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 87 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 29 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 261 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 87 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 261 ]), &(acadoWorkspace.sbar[ 87 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 252 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 252 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 78 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 234 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 78 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 234 ]), &(acadoWorkspace.sbar[ 78 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 75 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 25 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 75 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.sbar[ 75 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 69 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 23 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 207 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 69 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 207 ]), &(acadoWorkspace.sbar[ 69 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 66 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 198 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 66 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 198 ]), &(acadoWorkspace.sbar[ 66 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 63 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 180 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 180 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 57 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 19 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 171 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 57 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.sbar[ 57 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 51 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 17 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 153 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 51 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.sbar[ 51 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 45 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 42 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 126 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.sbar[ 42 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 39 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 13 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 117 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.sbar[ 39 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 33 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 11 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 99 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 33 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.sbar[ 33 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 21 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 7 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.sbar[ 21 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 15 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 5 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.sbar[ 15 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 3 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 9 ]), &(acadoWorkspace.sbar[ 3 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


tmp = acadoWorkspace.sbar[4] + acadoVariables.x[4];
acadoWorkspace.lbA[0] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[0] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[7] + acadoVariables.x[7];
acadoWorkspace.lbA[1] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[1] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[10] + acadoVariables.x[10];
acadoWorkspace.lbA[2] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[2] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[13] + acadoVariables.x[13];
acadoWorkspace.lbA[3] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[3] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[4] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[4] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[19] + acadoVariables.x[19];
acadoWorkspace.lbA[5] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[5] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[22] + acadoVariables.x[22];
acadoWorkspace.lbA[6] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[6] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[25] + acadoVariables.x[25];
acadoWorkspace.lbA[7] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[7] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[28] + acadoVariables.x[28];
acadoWorkspace.lbA[8] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[8] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[31] + acadoVariables.x[31];
acadoWorkspace.lbA[9] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[9] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[10] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[10] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[37] + acadoVariables.x[37];
acadoWorkspace.lbA[11] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[11] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[40] + acadoVariables.x[40];
acadoWorkspace.lbA[12] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[12] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[43] + acadoVariables.x[43];
acadoWorkspace.lbA[13] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[13] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[46] + acadoVariables.x[46];
acadoWorkspace.lbA[14] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[14] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[49] + acadoVariables.x[49];
acadoWorkspace.lbA[15] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[15] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[52] + acadoVariables.x[52];
acadoWorkspace.lbA[16] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[16] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[55] + acadoVariables.x[55];
acadoWorkspace.lbA[17] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[17] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[58] + acadoVariables.x[58];
acadoWorkspace.lbA[18] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[18] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[61] + acadoVariables.x[61];
acadoWorkspace.lbA[19] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[19] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[64] + acadoVariables.x[64];
acadoWorkspace.lbA[20] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[20] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[67] + acadoVariables.x[67];
acadoWorkspace.lbA[21] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[21] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[70] + acadoVariables.x[70];
acadoWorkspace.lbA[22] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[22] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[73] + acadoVariables.x[73];
acadoWorkspace.lbA[23] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[23] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[24] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[24] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[79] + acadoVariables.x[79];
acadoWorkspace.lbA[25] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[25] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[82] + acadoVariables.x[82];
acadoWorkspace.lbA[26] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[26] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[85] + acadoVariables.x[85];
acadoWorkspace.lbA[27] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[27] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[88] + acadoVariables.x[88];
acadoWorkspace.lbA[28] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[28] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[91] + acadoVariables.x[91];
acadoWorkspace.lbA[29] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[29] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[94] + acadoVariables.x[94];
acadoWorkspace.lbA[30] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[30] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[97] + acadoVariables.x[97];
acadoWorkspace.lbA[31] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[31] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[100] + acadoVariables.x[100];
acadoWorkspace.lbA[32] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[32] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[103] + acadoVariables.x[103];
acadoWorkspace.lbA[33] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[33] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[106] + acadoVariables.x[106];
acadoWorkspace.lbA[34] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[34] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[109] + acadoVariables.x[109];
acadoWorkspace.lbA[35] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[35] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[112] + acadoVariables.x[112];
acadoWorkspace.lbA[36] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[36] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[115] + acadoVariables.x[115];
acadoWorkspace.lbA[37] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[37] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[118] + acadoVariables.x[118];
acadoWorkspace.lbA[38] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[38] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[121] + acadoVariables.x[121];
acadoWorkspace.lbA[39] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[39] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[124] + acadoVariables.x[124];
acadoWorkspace.lbA[40] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[40] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[127] + acadoVariables.x[127];
acadoWorkspace.lbA[41] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[41] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[130] + acadoVariables.x[130];
acadoWorkspace.lbA[42] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[42] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[133] + acadoVariables.x[133];
acadoWorkspace.lbA[43] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[43] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[136] + acadoVariables.x[136];
acadoWorkspace.lbA[44] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[44] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[139] + acadoVariables.x[139];
acadoWorkspace.lbA[45] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[45] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[142] + acadoVariables.x[142];
acadoWorkspace.lbA[46] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[46] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[145] + acadoVariables.x[145];
acadoWorkspace.lbA[47] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[47] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[148] + acadoVariables.x[148];
acadoWorkspace.lbA[48] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[48] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[151] + acadoVariables.x[151];
acadoWorkspace.lbA[49] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[49] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[154] + acadoVariables.x[154];
acadoWorkspace.lbA[50] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[50] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[157] + acadoVariables.x[157];
acadoWorkspace.lbA[51] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[51] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[160] + acadoVariables.x[160];
acadoWorkspace.lbA[52] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[52] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[163] + acadoVariables.x[163];
acadoWorkspace.lbA[53] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[53] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[166] + acadoVariables.x[166];
acadoWorkspace.lbA[54] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[54] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[169] + acadoVariables.x[169];
acadoWorkspace.lbA[55] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[55] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[172] + acadoVariables.x[172];
acadoWorkspace.lbA[56] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[56] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[175] + acadoVariables.x[175];
acadoWorkspace.lbA[57] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[57] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[178] + acadoVariables.x[178];
acadoWorkspace.lbA[58] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[58] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[181] + acadoVariables.x[181];
acadoWorkspace.lbA[59] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[59] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[184] + acadoVariables.x[184];
acadoWorkspace.lbA[60] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[60] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[187] + acadoVariables.x[187];
acadoWorkspace.lbA[61] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[61] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[190] + acadoVariables.x[190];
acadoWorkspace.lbA[62] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[62] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[193] + acadoVariables.x[193];
acadoWorkspace.lbA[63] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[63] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[196] + acadoVariables.x[196];
acadoWorkspace.lbA[64] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[64] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[199] + acadoVariables.x[199];
acadoWorkspace.lbA[65] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[65] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[202] + acadoVariables.x[202];
acadoWorkspace.lbA[66] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[66] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[205] + acadoVariables.x[205];
acadoWorkspace.lbA[67] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[67] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[208] + acadoVariables.x[208];
acadoWorkspace.lbA[68] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[68] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[211] + acadoVariables.x[211];
acadoWorkspace.lbA[69] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[69] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[214] + acadoVariables.x[214];
acadoWorkspace.lbA[70] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[70] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[217] + acadoVariables.x[217];
acadoWorkspace.lbA[71] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[71] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[220] + acadoVariables.x[220];
acadoWorkspace.lbA[72] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[72] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[223] + acadoVariables.x[223];
acadoWorkspace.lbA[73] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[73] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[226] + acadoVariables.x[226];
acadoWorkspace.lbA[74] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[74] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[229] + acadoVariables.x[229];
acadoWorkspace.lbA[75] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[75] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[232] + acadoVariables.x[232];
acadoWorkspace.lbA[76] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[76] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[235] + acadoVariables.x[235];
acadoWorkspace.lbA[77] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[77] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[238] + acadoVariables.x[238];
acadoWorkspace.lbA[78] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[78] = (real_t)5.0000000000000000e-001 - tmp;
tmp = acadoWorkspace.sbar[241] + acadoVariables.x[241];
acadoWorkspace.lbA[79] = (real_t)-5.0000000000000000e-001 - tmp;
acadoWorkspace.ubA[79] = (real_t)5.0000000000000000e-001 - tmp;

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
for (lRun1 = 0; lRun1 < 240; ++lRun1)
acadoWorkspace.sbar[lRun1 + 3] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 3 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGu[ 3 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGu[ 9 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.evGu[ 15 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.evGu[ 21 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.evGu[ 33 ]), &(acadoWorkspace.x[ 11 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.evGu[ 39 ]), &(acadoWorkspace.x[ 13 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 42 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 42 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.evGu[ 45 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 51 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.evGu[ 51 ]), &(acadoWorkspace.x[ 17 ]), &(acadoWorkspace.sbar[ 51 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 57 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.evGu[ 57 ]), &(acadoWorkspace.x[ 19 ]), &(acadoWorkspace.sbar[ 57 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 180 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 189 ]), &(acadoWorkspace.evGu[ 63 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 66 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 198 ]), &(acadoWorkspace.evGu[ 66 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 66 ]), &(acadoWorkspace.sbar[ 69 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 207 ]), &(acadoWorkspace.evGu[ 69 ]), &(acadoWorkspace.x[ 23 ]), &(acadoWorkspace.sbar[ 69 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 216 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 75 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.evGu[ 75 ]), &(acadoWorkspace.x[ 25 ]), &(acadoWorkspace.sbar[ 75 ]), &(acadoWorkspace.sbar[ 78 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 234 ]), &(acadoWorkspace.evGu[ 78 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 78 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 81 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 252 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 87 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 261 ]), &(acadoWorkspace.evGu[ 87 ]), &(acadoWorkspace.x[ 29 ]), &(acadoWorkspace.sbar[ 87 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 270 ]), &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 93 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 279 ]), &(acadoWorkspace.evGu[ 93 ]), &(acadoWorkspace.x[ 31 ]), &(acadoWorkspace.sbar[ 93 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 297 ]), &(acadoWorkspace.evGu[ 99 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 102 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 306 ]), &(acadoWorkspace.evGu[ 102 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 102 ]), &(acadoWorkspace.sbar[ 105 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 315 ]), &(acadoWorkspace.evGu[ 105 ]), &(acadoWorkspace.x[ 35 ]), &(acadoWorkspace.sbar[ 105 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 111 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 333 ]), &(acadoWorkspace.evGu[ 111 ]), &(acadoWorkspace.x[ 37 ]), &(acadoWorkspace.sbar[ 111 ]), &(acadoWorkspace.sbar[ 114 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 342 ]), &(acadoWorkspace.evGu[ 114 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 114 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 351 ]), &(acadoWorkspace.evGu[ 117 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 360 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 123 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 369 ]), &(acadoWorkspace.evGu[ 123 ]), &(acadoWorkspace.x[ 41 ]), &(acadoWorkspace.sbar[ 123 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 378 ]), &(acadoWorkspace.evGu[ 126 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 129 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 387 ]), &(acadoWorkspace.evGu[ 129 ]), &(acadoWorkspace.x[ 43 ]), &(acadoWorkspace.sbar[ 129 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 396 ]), &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 135 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 138 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 414 ]), &(acadoWorkspace.evGu[ 138 ]), &(acadoWorkspace.x[ 46 ]), &(acadoWorkspace.sbar[ 138 ]), &(acadoWorkspace.sbar[ 141 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 423 ]), &(acadoWorkspace.evGu[ 141 ]), &(acadoWorkspace.x[ 47 ]), &(acadoWorkspace.sbar[ 141 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 147 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 441 ]), &(acadoWorkspace.evGu[ 147 ]), &(acadoWorkspace.x[ 49 ]), &(acadoWorkspace.sbar[ 147 ]), &(acadoWorkspace.sbar[ 150 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 450 ]), &(acadoWorkspace.evGu[ 150 ]), &(acadoWorkspace.x[ 50 ]), &(acadoWorkspace.sbar[ 150 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 459 ]), &(acadoWorkspace.evGu[ 153 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 468 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 159 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 477 ]), &(acadoWorkspace.evGu[ 159 ]), &(acadoWorkspace.x[ 53 ]), &(acadoWorkspace.sbar[ 159 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 165 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 495 ]), &(acadoWorkspace.evGu[ 165 ]), &(acadoWorkspace.x[ 55 ]), &(acadoWorkspace.sbar[ 165 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 504 ]), &(acadoWorkspace.evGu[ 168 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 513 ]), &(acadoWorkspace.evGu[ 171 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 174 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 522 ]), &(acadoWorkspace.evGu[ 174 ]), &(acadoWorkspace.x[ 58 ]), &(acadoWorkspace.sbar[ 174 ]), &(acadoWorkspace.sbar[ 177 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 531 ]), &(acadoWorkspace.evGu[ 177 ]), &(acadoWorkspace.x[ 59 ]), &(acadoWorkspace.sbar[ 177 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 540 ]), &(acadoWorkspace.evGu[ 180 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 183 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 549 ]), &(acadoWorkspace.evGu[ 183 ]), &(acadoWorkspace.x[ 61 ]), &(acadoWorkspace.sbar[ 183 ]), &(acadoWorkspace.sbar[ 186 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 558 ]), &(acadoWorkspace.evGu[ 186 ]), &(acadoWorkspace.x[ 62 ]), &(acadoWorkspace.sbar[ 186 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 195 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 585 ]), &(acadoWorkspace.evGu[ 195 ]), &(acadoWorkspace.x[ 65 ]), &(acadoWorkspace.sbar[ 195 ]), &(acadoWorkspace.sbar[ 198 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 594 ]), &(acadoWorkspace.evGu[ 198 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.sbar[ 201 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 603 ]), &(acadoWorkspace.evGu[ 201 ]), &(acadoWorkspace.x[ 67 ]), &(acadoWorkspace.sbar[ 201 ]), &(acadoWorkspace.sbar[ 204 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 612 ]), &(acadoWorkspace.evGu[ 204 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 204 ]), &(acadoWorkspace.sbar[ 207 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 621 ]), &(acadoWorkspace.evGu[ 207 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.sbar[ 210 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 630 ]), &(acadoWorkspace.evGu[ 210 ]), &(acadoWorkspace.x[ 70 ]), &(acadoWorkspace.sbar[ 210 ]), &(acadoWorkspace.sbar[ 213 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 639 ]), &(acadoWorkspace.evGu[ 213 ]), &(acadoWorkspace.x[ 71 ]), &(acadoWorkspace.sbar[ 213 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 219 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 657 ]), &(acadoWorkspace.evGu[ 219 ]), &(acadoWorkspace.x[ 73 ]), &(acadoWorkspace.sbar[ 219 ]), &(acadoWorkspace.sbar[ 222 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 666 ]), &(acadoWorkspace.evGu[ 222 ]), &(acadoWorkspace.x[ 74 ]), &(acadoWorkspace.sbar[ 222 ]), &(acadoWorkspace.sbar[ 225 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 675 ]), &(acadoWorkspace.evGu[ 225 ]), &(acadoWorkspace.x[ 75 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.sbar[ 228 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 684 ]), &(acadoWorkspace.evGu[ 228 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 228 ]), &(acadoWorkspace.sbar[ 231 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 693 ]), &(acadoWorkspace.evGu[ 231 ]), &(acadoWorkspace.x[ 77 ]), &(acadoWorkspace.sbar[ 231 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 702 ]), &(acadoWorkspace.evGu[ 234 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 237 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 711 ]), &(acadoWorkspace.evGu[ 237 ]), &(acadoWorkspace.x[ 79 ]), &(acadoWorkspace.sbar[ 237 ]), &(acadoWorkspace.sbar[ 240 ]) );
for (lRun1 = 0; lRun1 < 243; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 80; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[15] = acadoVariables.u[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 80; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[240] = xEnd[0];
acadoVariables.x[241] = xEnd[1];
acadoVariables.x[242] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[240];
acadoWorkspace.state[1] = acadoVariables.x[241];
acadoWorkspace.state[2] = acadoVariables.x[242];
if (uEnd != 0)
{
acadoWorkspace.state[15] = uEnd[0];
}
else
{
acadoWorkspace.state[15] = acadoVariables.u[79];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[240] = acadoWorkspace.state[0];
acadoVariables.x[241] = acadoWorkspace.state[1];
acadoVariables.x[242] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 79; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[79] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79];
kkt = fabs( kkt );
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-012)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-012)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index + 80];
if (prd > 1e-012)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-012)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 4 */
real_t tmpDy[ 4 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 80; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 4] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 4];
acadoWorkspace.Dy[lRun1 * 4 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 4 + 1];
acadoWorkspace.Dy[lRun1 * 4 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 4 + 2];
acadoWorkspace.Dy[lRun1 * 4 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 4 + 3];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[240];
acadoWorkspace.objValueIn[1] = acadoVariables.x[241];
acadoWorkspace.objValueIn[2] = acadoVariables.x[242];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+000;
for (lRun1 = 0; lRun1 < 80; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 4]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 4 + 1]*acadoVariables.W[5];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 4 + 2]*acadoVariables.W[10];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 4 + 3]*acadoVariables.W[15];
objVal += + acadoWorkspace.Dy[lRun1 * 4]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 4 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 4 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 4 + 3]*tmpDy[3];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[4];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[8];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

