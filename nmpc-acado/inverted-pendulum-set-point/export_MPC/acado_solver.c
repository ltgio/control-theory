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
for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 4 + 3];

acadoWorkspace.state[24] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 4] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 4 + 4];
acadoWorkspace.d[lRun1 * 4 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 4 + 5];
acadoWorkspace.d[lRun1 * 4 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 4 + 6];
acadoWorkspace.d[lRun1 * 4 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 4 + 7];

acadoWorkspace.evGx[lRun1 * 16] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 16 + 1] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 16 + 2] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 16 + 3] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 16 + 4] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 16 + 5] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 16 + 6] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 16 + 7] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 16 + 8] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 16 + 9] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 16 + 10] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 16 + 11] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 16 + 12] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 16 + 13] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 16 + 14] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 16 + 15] = acadoWorkspace.state[19];

acadoWorkspace.evGu[lRun1 * 4] = acadoWorkspace.state[20];
acadoWorkspace.evGu[lRun1 * 4 + 1] = acadoWorkspace.state[21];
acadoWorkspace.evGu[lRun1 * 4 + 2] = acadoWorkspace.state[22];
acadoWorkspace.evGu[lRun1 * 4 + 3] = acadoWorkspace.state[23];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = u[0];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
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
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[5];
tmpQ1[5] = + tmpQ2[6];
tmpQ1[6] = + tmpQ2[7];
tmpQ1[7] = + tmpQ2[8];
tmpQ1[8] = + tmpQ2[10];
tmpQ1[9] = + tmpQ2[11];
tmpQ1[10] = + tmpQ2[12];
tmpQ1[11] = + tmpQ2[13];
tmpQ1[12] = + tmpQ2[15];
tmpQ1[13] = + tmpQ2[16];
tmpQ1[14] = + tmpQ2[17];
tmpQ1[15] = + tmpQ2[18];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[20];
tmpR2[1] = +tmpObjS[21];
tmpR2[2] = +tmpObjS[22];
tmpR2[3] = +tmpObjS[23];
tmpR2[4] = +tmpObjS[24];
tmpR1[0] = + tmpR2[4];
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
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 40; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 16 ]), &(acadoWorkspace.Q2[ runObj * 20 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj ]), &(acadoWorkspace.R2[ runObj * 5 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[160];
acadoWorkspace.objValueIn[1] = acadoVariables.x[161];
acadoWorkspace.objValueIn[2] = acadoVariables.x[162];
acadoWorkspace.objValueIn[3] = acadoVariables.x[163];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2] + Gx1[3]*Gu1[3];
Gu2[1] = + Gx1[4]*Gu1[0] + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[3];
Gu2[2] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[2] + Gx1[11]*Gu1[3];
Gu2[3] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[1] + Gx1[14]*Gu1[2] + Gx1[15]*Gu1[3];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 41] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + Gu1[3]*Gu2[3] + R11[0];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[4]*Gu1[1] + Gx1[8]*Gu1[2] + Gx1[12]*Gu1[3];
Gu2[1] = + Gx1[1]*Gu1[0] + Gx1[5]*Gu1[1] + Gx1[9]*Gu1[2] + Gx1[13]*Gu1[3];
Gu2[2] = + Gx1[2]*Gu1[0] + Gx1[6]*Gu1[1] + Gx1[10]*Gu1[2] + Gx1[14]*Gu1[3];
Gu2[3] = + Gx1[3]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[11]*Gu1[2] + Gx1[15]*Gu1[3];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[1] + Q11[2]*Gu1[2] + Q11[3]*Gu1[3] + Gu2[0];
Gu3[1] = + Q11[4]*Gu1[0] + Q11[5]*Gu1[1] + Q11[6]*Gu1[2] + Q11[7]*Gu1[3] + Gu2[1];
Gu3[2] = + Q11[8]*Gu1[0] + Q11[9]*Gu1[1] + Q11[10]*Gu1[2] + Q11[11]*Gu1[3] + Gu2[2];
Gu3[3] = + Q11[12]*Gu1[0] + Q11[13]*Gu1[1] + Q11[14]*Gu1[2] + Q11[15]*Gu1[3] + Gu2[3];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[4]*w11[1] + Gx1[8]*w11[2] + Gx1[12]*w11[3] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[5]*w11[1] + Gx1[9]*w11[2] + Gx1[13]*w11[3] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[6]*w11[1] + Gx1[10]*w11[2] + Gx1[14]*w11[3] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[7]*w11[1] + Gx1[11]*w11[2] + Gx1[15]*w11[3] + w12[3];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[1]*w11[1] + Gu1[2]*w11[2] + Gu1[3]*w11[3];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + w12[0];
w13[1] = + Q11[4]*w11[0] + Q11[5]*w11[1] + Q11[6]*w11[2] + Q11[7]*w11[3] + w12[1];
w13[2] = + Q11[8]*w11[0] + Q11[9]*w11[1] + Q11[10]*w11[2] + Q11[11]*w11[3] + w12[2];
w13[3] = + Q11[12]*w11[0] + Q11[13]*w11[1] + Q11[14]*w11[2] + Q11[15]*w11[3] + w12[3];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3];
w12[1] += + Gx1[4]*w11[0] + Gx1[5]*w11[1] + Gx1[6]*w11[2] + Gx1[7]*w11[3];
w12[2] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3];
w12[3] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3];
w12[1] += + Gx1[4]*w11[0] + Gx1[5]*w11[1] + Gx1[6]*w11[2] + Gx1[7]*w11[3];
w12[2] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3];
w12[3] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3];
w12[0] += + Gu1[0]*U1[0];
w12[1] += + Gu1[1]*U1[0];
w12[2] += + Gu1[2]*U1[0];
w12[3] += + Gu1[3]*U1[0];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol)] = acadoWorkspace.H[(iCol * 40) + (iRow)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4];
QDy1[1] = + Q2[5]*Dy1[0] + Q2[6]*Dy1[1] + Q2[7]*Dy1[2] + Q2[8]*Dy1[3] + Q2[9]*Dy1[4];
QDy1[2] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4];
QDy1[3] = + Q2[15]*Dy1[0] + Q2[16]*Dy1[1] + Q2[17]*Dy1[2] + Q2[18]*Dy1[3] + Q2[19]*Dy1[4];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 40 */
static const int xBoundIndices[ 40 ] = 
{ 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160 };
for (lRun2 = 0; lRun2 < 40; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 81)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 4 ]), &(acadoWorkspace.E[ lRun3 * 4 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 40; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (4)) * (4)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (4)) * (1)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (4)) * (1)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (40)) - (1)) * (4)) * (1)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 39; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 4 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 16 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (4)) * (1)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 ]), &(acadoWorkspace.evGu[ lRun2 * 4 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoWorkspace.sbar[lRun1 + 4] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-2.0000000000000000e+001 - acadoVariables.u[39];
acadoWorkspace.ub[0] = (real_t)2.0000000000000000e+001 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)2.0000000000000000e+001 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)2.0000000000000000e+001 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)2.0000000000000000e+001 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)2.0000000000000000e+001 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)2.0000000000000000e+001 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)2.0000000000000000e+001 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)2.0000000000000000e+001 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)2.0000000000000000e+001 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)2.0000000000000000e+001 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)2.0000000000000000e+001 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)2.0000000000000000e+001 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)2.0000000000000000e+001 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)2.0000000000000000e+001 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)2.0000000000000000e+001 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)2.0000000000000000e+001 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)2.0000000000000000e+001 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)2.0000000000000000e+001 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)2.0000000000000000e+001 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)2.0000000000000000e+001 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)2.0000000000000000e+001 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)2.0000000000000000e+001 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)2.0000000000000000e+001 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)2.0000000000000000e+001 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)2.0000000000000000e+001 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)2.0000000000000000e+001 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)2.0000000000000000e+001 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)2.0000000000000000e+001 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)2.0000000000000000e+001 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)2.0000000000000000e+001 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)2.0000000000000000e+001 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)2.0000000000000000e+001 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)2.0000000000000000e+001 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)2.0000000000000000e+001 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)2.0000000000000000e+001 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)2.0000000000000000e+001 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)2.0000000000000000e+001 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)2.0000000000000000e+001 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)2.0000000000000000e+001 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)2.0000000000000000e+001 - acadoVariables.u[39];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 4;
lRun4 = ((lRun3) / (4)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 79)) / (2)) + (lRun4)) - (1)) * (4)) + ((lRun3) % (4));
acadoWorkspace.A[(lRun1 * 40) + (lRun2)] = acadoWorkspace.E[lRun5];
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
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 5 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 10 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 15 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 20 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 25 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 35 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 45 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 50 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 55 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 65 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 70 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 75 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 85 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 90 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 95 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 19 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 100 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 105 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 110 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 115 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.g[ 23 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 125 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.g[ 25 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 130 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 135 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 140 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 145 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.g[ 29 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 150 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 155 ]), &(acadoWorkspace.Dy[ 155 ]), &(acadoWorkspace.g[ 31 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 160 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 165 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 170 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 175 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.g[ 35 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 185 ]), &(acadoWorkspace.Dy[ 185 ]), &(acadoWorkspace.g[ 37 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 190 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 195 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.g[ 39 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 20 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 40 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 80 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 100 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 140 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 160 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 200 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 220 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 44 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 260 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 52 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 280 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 300 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 320 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 340 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 68 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 360 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 380 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 76 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 400 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 420 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 440 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 88 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 460 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.QDy[ 92 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 500 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.QDy[ 100 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 520 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.QDy[ 104 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 560 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 112 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 580 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.QDy[ 116 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 600 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 620 ]), &(acadoWorkspace.Dy[ 155 ]), &(acadoWorkspace.QDy[ 124 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 640 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 128 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 660 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 680 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.QDy[ 136 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 700 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.QDy[ 140 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 720 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 740 ]), &(acadoWorkspace.Dy[ 185 ]), &(acadoWorkspace.QDy[ 148 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 760 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.QDy[ 152 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 780 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.QDy[ 156 ]) );

acadoWorkspace.QDy[160] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[161] = + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[162] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[3];
acadoWorkspace.QDy[163] = + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[3];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 44 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 68 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 76 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 88 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.sbar[ 92 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 116 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 464 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 124 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 496 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.sbar[ 128 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 528 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 136 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 544 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 148 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 592 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.sbar[ 152 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 608 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 624 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 160 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[163] + acadoWorkspace.QDy[160];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[163] + acadoWorkspace.QDy[161];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[163] + acadoWorkspace.QDy[162];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[163] + acadoWorkspace.QDy[163];
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 624 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 624 ]), &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 152 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 608 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 152 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 608 ]), &(acadoWorkspace.sbar[ 152 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 148 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 37 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 592 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 148 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 592 ]), &(acadoWorkspace.sbar[ 148 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 140 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 35 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 560 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 140 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 560 ]), &(acadoWorkspace.sbar[ 140 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 136 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 544 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 136 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 544 ]), &(acadoWorkspace.sbar[ 136 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 132 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 528 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 528 ]), &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 128 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.sbar[ 128 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 124 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 31 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 496 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 124 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 496 ]), &(acadoWorkspace.sbar[ 124 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 480 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 116 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 29 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 464 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 116 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 464 ]), &(acadoWorkspace.sbar[ 116 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 112 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.sbar[ 112 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 432 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 104 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 416 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 104 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 416 ]), &(acadoWorkspace.sbar[ 104 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 100 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 25 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 400 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 100 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 400 ]), &(acadoWorkspace.sbar[ 100 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 92 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 23 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 368 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 92 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 368 ]), &(acadoWorkspace.sbar[ 92 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 88 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 352 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 88 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 352 ]), &(acadoWorkspace.sbar[ 88 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 336 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 336 ]), &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 80 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 76 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 19 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 304 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 76 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 304 ]), &(acadoWorkspace.sbar[ 76 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 288 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 68 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 17 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 272 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 68 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 272 ]), &(acadoWorkspace.sbar[ 68 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 64 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 240 ]), &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 224 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 224 ]), &(acadoWorkspace.sbar[ 56 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 52 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 13 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 208 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 52 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 208 ]), &(acadoWorkspace.sbar[ 52 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 44 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 11 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 176 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 44 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 176 ]), &(acadoWorkspace.sbar[ 44 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 160 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 7 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 5 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


tmp = acadoWorkspace.sbar[4] + acadoVariables.x[4];
acadoWorkspace.lbA[0] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[0] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[8] + acadoVariables.x[8];
acadoWorkspace.lbA[1] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[1] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[12] + acadoVariables.x[12];
acadoWorkspace.lbA[2] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[2] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[3] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[3] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[20] + acadoVariables.x[20];
acadoWorkspace.lbA[4] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[4] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[5] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[5] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[28] + acadoVariables.x[28];
acadoWorkspace.lbA[6] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[6] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[32] + acadoVariables.x[32];
acadoWorkspace.lbA[7] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[7] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[36] + acadoVariables.x[36];
acadoWorkspace.lbA[8] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[8] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[40] + acadoVariables.x[40];
acadoWorkspace.lbA[9] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[9] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[44] + acadoVariables.x[44];
acadoWorkspace.lbA[10] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[10] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[48] + acadoVariables.x[48];
acadoWorkspace.lbA[11] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[11] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[52] + acadoVariables.x[52];
acadoWorkspace.lbA[12] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[12] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[56] + acadoVariables.x[56];
acadoWorkspace.lbA[13] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[13] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[60] + acadoVariables.x[60];
acadoWorkspace.lbA[14] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[14] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[64] + acadoVariables.x[64];
acadoWorkspace.lbA[15] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[15] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[68] + acadoVariables.x[68];
acadoWorkspace.lbA[16] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[16] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[72] + acadoVariables.x[72];
acadoWorkspace.lbA[17] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[17] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[18] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[18] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[80] + acadoVariables.x[80];
acadoWorkspace.lbA[19] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[19] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[84] + acadoVariables.x[84];
acadoWorkspace.lbA[20] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[20] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[88] + acadoVariables.x[88];
acadoWorkspace.lbA[21] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[21] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[92] + acadoVariables.x[92];
acadoWorkspace.lbA[22] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[22] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[96] + acadoVariables.x[96];
acadoWorkspace.lbA[23] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[23] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[100] + acadoVariables.x[100];
acadoWorkspace.lbA[24] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[24] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[104] + acadoVariables.x[104];
acadoWorkspace.lbA[25] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[25] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[108] + acadoVariables.x[108];
acadoWorkspace.lbA[26] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[26] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[112] + acadoVariables.x[112];
acadoWorkspace.lbA[27] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[27] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[116] + acadoVariables.x[116];
acadoWorkspace.lbA[28] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[28] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[120] + acadoVariables.x[120];
acadoWorkspace.lbA[29] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[29] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[124] + acadoVariables.x[124];
acadoWorkspace.lbA[30] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[30] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[128] + acadoVariables.x[128];
acadoWorkspace.lbA[31] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[31] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[132] + acadoVariables.x[132];
acadoWorkspace.lbA[32] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[32] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[136] + acadoVariables.x[136];
acadoWorkspace.lbA[33] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[33] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[140] + acadoVariables.x[140];
acadoWorkspace.lbA[34] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[34] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[144] + acadoVariables.x[144];
acadoWorkspace.lbA[35] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[35] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[148] + acadoVariables.x[148];
acadoWorkspace.lbA[36] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[36] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[152] + acadoVariables.x[152];
acadoWorkspace.lbA[37] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[37] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[156] + acadoVariables.x[156];
acadoWorkspace.lbA[38] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[38] = (real_t)2.0000000000000000e+000 - tmp;
tmp = acadoWorkspace.sbar[160] + acadoVariables.x[160];
acadoWorkspace.lbA[39] = (real_t)-2.0000000000000000e+000 - tmp;
acadoWorkspace.ubA[39] = (real_t)2.0000000000000000e+000 - tmp;

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
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoWorkspace.sbar[lRun1 + 4] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 48 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 80 ]), &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 96 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 112 ]), &(acadoWorkspace.evGu[ 28 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 160 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 44 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 176 ]), &(acadoWorkspace.evGu[ 44 ]), &(acadoWorkspace.x[ 11 ]), &(acadoWorkspace.sbar[ 44 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 52 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 208 ]), &(acadoWorkspace.evGu[ 52 ]), &(acadoWorkspace.x[ 13 ]), &(acadoWorkspace.sbar[ 52 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 224 ]), &(acadoWorkspace.evGu[ 56 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 240 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 68 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 272 ]), &(acadoWorkspace.evGu[ 68 ]), &(acadoWorkspace.x[ 17 ]), &(acadoWorkspace.sbar[ 68 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 76 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 304 ]), &(acadoWorkspace.evGu[ 76 ]), &(acadoWorkspace.x[ 19 ]), &(acadoWorkspace.sbar[ 76 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 336 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 88 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 352 ]), &(acadoWorkspace.evGu[ 88 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.sbar[ 92 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 368 ]), &(acadoWorkspace.evGu[ 92 ]), &(acadoWorkspace.x[ 23 ]), &(acadoWorkspace.sbar[ 92 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 100 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 400 ]), &(acadoWorkspace.evGu[ 100 ]), &(acadoWorkspace.x[ 25 ]), &(acadoWorkspace.sbar[ 100 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 416 ]), &(acadoWorkspace.evGu[ 104 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 116 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 464 ]), &(acadoWorkspace.evGu[ 116 ]), &(acadoWorkspace.x[ 29 ]), &(acadoWorkspace.sbar[ 116 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 480 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 124 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 496 ]), &(acadoWorkspace.evGu[ 124 ]), &(acadoWorkspace.x[ 31 ]), &(acadoWorkspace.sbar[ 124 ]), &(acadoWorkspace.sbar[ 128 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 528 ]), &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 136 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 544 ]), &(acadoWorkspace.evGu[ 136 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.sbar[ 140 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 560 ]), &(acadoWorkspace.evGu[ 140 ]), &(acadoWorkspace.x[ 35 ]), &(acadoWorkspace.sbar[ 140 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 148 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 592 ]), &(acadoWorkspace.evGu[ 148 ]), &(acadoWorkspace.x[ 37 ]), &(acadoWorkspace.sbar[ 148 ]), &(acadoWorkspace.sbar[ 152 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 608 ]), &(acadoWorkspace.evGu[ 152 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 624 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 160 ]) );
for (lRun1 = 0; lRun1 < 164; ++lRun1)
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
for (index = 0; index < 40; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 4];
acadoWorkspace.state[1] = acadoVariables.x[index * 4 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 4 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 4 + 3];
acadoWorkspace.state[24] = acadoVariables.u[index];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 4 + 4] = acadoWorkspace.state[0];
acadoVariables.x[index * 4 + 5] = acadoWorkspace.state[1];
acadoVariables.x[index * 4 + 6] = acadoWorkspace.state[2];
acadoVariables.x[index * 4 + 7] = acadoWorkspace.state[3];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 40; ++index)
{
acadoVariables.x[index * 4] = acadoVariables.x[index * 4 + 4];
acadoVariables.x[index * 4 + 1] = acadoVariables.x[index * 4 + 5];
acadoVariables.x[index * 4 + 2] = acadoVariables.x[index * 4 + 6];
acadoVariables.x[index * 4 + 3] = acadoVariables.x[index * 4 + 7];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[160] = xEnd[0];
acadoVariables.x[161] = xEnd[1];
acadoVariables.x[162] = xEnd[2];
acadoVariables.x[163] = xEnd[3];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[160];
acadoWorkspace.state[1] = acadoVariables.x[161];
acadoWorkspace.state[2] = acadoVariables.x[162];
acadoWorkspace.state[3] = acadoVariables.x[163];
if (uEnd != 0)
{
acadoWorkspace.state[24] = uEnd[0];
}
else
{
acadoWorkspace.state[24] = acadoVariables.u[39];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[160] = acadoWorkspace.state[0];
acadoVariables.x[161] = acadoWorkspace.state[1];
acadoVariables.x[162] = acadoWorkspace.state[2];
acadoVariables.x[163] = acadoWorkspace.state[3];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 39; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[39] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39];
kkt = fabs( kkt );
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-012)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-012)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index + 40];
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
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 4 */
real_t tmpDyN[ 4 ];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 4];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[160];
acadoWorkspace.objValueIn[1] = acadoVariables.x[161];
acadoWorkspace.objValueIn[2] = acadoVariables.x[162];
acadoWorkspace.objValueIn[3] = acadoVariables.x[163];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
objVal = 0.0000000000000000e+000;
for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5 + 1]*acadoVariables.W[6];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5 + 2]*acadoVariables.W[12];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5 + 3]*acadoVariables.W[18];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5 + 4]*acadoVariables.W[24];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[5];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[10];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[15];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3];

objVal *= 0.5;
return objVal;
}

