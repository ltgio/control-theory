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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 4;
/* Vector of auxiliary variables; number of elements: 11. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (sin(xd[1]));
a[1] = (cos(xd[1]));
a[2] = (sin(xd[1]));
a[3] = (cos(xd[1]));
a[4] = (cos(xd[1]));
a[5] = (cos(xd[1]));
a[6] = (sin(xd[1]));
a[7] = (sin(xd[1]));
a[8] = (cos(xd[1]));
a[9] = (cos(xd[1]));
a[10] = (cos(xd[1]));

/* Compute outputs: */
out[0] = xd[2];
out[1] = xd[3];
out[2] = (((((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[3])-(((real_t)(9.8100000000000009e-001)*a[1])*a[2]))+u[0])/((((real_t)(-1.0000000000000001e-001)*a[3])*a[4])+(real_t)(1.1000000000000001e+000)));
out[3] = (((((((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[3])+((real_t)(1.0791000000000002e+001)*a[7]))+(u[0]*a[8]))/((((real_t)(-1.0000000000000001e-001)*a[9])*a[10])+(real_t)(1.1000000000000001e+000)))/(real_t)(8.0000000000000004e-001));
}

void acado_rhs_ext(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 24;
/* Vector of auxiliary variables; number of elements: 82. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (sin(xd[1]));
a[1] = (cos(xd[1]));
a[2] = (sin(xd[1]));
a[3] = (cos(xd[1]));
a[4] = (cos(xd[1]));
a[5] = (cos(xd[1]));
a[6] = (sin(xd[1]));
a[7] = (sin(xd[1]));
a[8] = (cos(xd[1]));
a[9] = (cos(xd[1]));
a[10] = (cos(xd[1]));
a[11] = (cos(xd[1]));
a[12] = (xd[8]*a[11]);
a[13] = ((real_t)(-1.0000000000000000e+000)*(sin(xd[1])));
a[14] = (xd[8]*a[13]);
a[15] = (cos(xd[1]));
a[16] = (xd[8]*a[15]);
a[17] = ((real_t)(1.0000000000000000e+000)/((((real_t)(-1.0000000000000001e-001)*a[3])*a[4])+(real_t)(1.1000000000000001e+000)));
a[18] = ((real_t)(-1.0000000000000000e+000)*(sin(xd[1])));
a[19] = (xd[8]*a[18]);
a[20] = ((real_t)(-1.0000000000000000e+000)*(sin(xd[1])));
a[21] = (xd[8]*a[20]);
a[22] = (a[17]*a[17]);
a[23] = (xd[9]*a[11]);
a[24] = (xd[9]*a[13]);
a[25] = (xd[9]*a[15]);
a[26] = (xd[9]*a[18]);
a[27] = (xd[9]*a[20]);
a[28] = (xd[10]*a[11]);
a[29] = (xd[10]*a[13]);
a[30] = (xd[10]*a[15]);
a[31] = (xd[10]*a[18]);
a[32] = (xd[10]*a[20]);
a[33] = (xd[11]*a[11]);
a[34] = (xd[11]*a[13]);
a[35] = (xd[11]*a[15]);
a[36] = (xd[11]*a[18]);
a[37] = (xd[11]*a[20]);
a[38] = ((real_t)(-1.0000000000000000e+000)*(sin(xd[1])));
a[39] = (xd[8]*a[38]);
a[40] = (cos(xd[1]));
a[41] = (xd[8]*a[40]);
a[42] = (cos(xd[1]));
a[43] = (xd[8]*a[42]);
a[44] = ((real_t)(-1.0000000000000000e+000)*(sin(xd[1])));
a[45] = (xd[8]*a[44]);
a[46] = ((real_t)(1.0000000000000000e+000)/((((real_t)(-1.0000000000000001e-001)*a[9])*a[10])+(real_t)(1.1000000000000001e+000)));
a[47] = ((real_t)(-1.0000000000000000e+000)*(sin(xd[1])));
a[48] = (xd[8]*a[47]);
a[49] = ((real_t)(-1.0000000000000000e+000)*(sin(xd[1])));
a[50] = (xd[8]*a[49]);
a[51] = (a[46]*a[46]);
a[52] = ((real_t)(1.0000000000000000e+000)/(real_t)(8.0000000000000004e-001));
a[53] = (xd[9]*a[38]);
a[54] = (xd[9]*a[40]);
a[55] = (xd[9]*a[42]);
a[56] = (xd[9]*a[44]);
a[57] = (xd[9]*a[47]);
a[58] = (xd[9]*a[49]);
a[59] = (xd[10]*a[38]);
a[60] = (xd[10]*a[40]);
a[61] = (xd[10]*a[42]);
a[62] = (xd[10]*a[44]);
a[63] = (xd[10]*a[47]);
a[64] = (xd[10]*a[49]);
a[65] = (xd[11]*a[38]);
a[66] = (xd[11]*a[40]);
a[67] = (xd[11]*a[42]);
a[68] = (xd[11]*a[44]);
a[69] = (xd[11]*a[47]);
a[70] = (xd[11]*a[49]);
a[71] = (xd[21]*a[11]);
a[72] = (xd[21]*a[13]);
a[73] = (xd[21]*a[15]);
a[74] = (xd[21]*a[18]);
a[75] = (xd[21]*a[20]);
a[76] = (xd[21]*a[38]);
a[77] = (xd[21]*a[40]);
a[78] = (xd[21]*a[42]);
a[79] = (xd[21]*a[44]);
a[80] = (xd[21]*a[47]);
a[81] = (xd[21]*a[49]);

/* Compute outputs: */
out[0] = xd[2];
out[1] = xd[3];
out[2] = (((((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[3])-(((real_t)(9.8100000000000009e-001)*a[1])*a[2]))+u[0])/((((real_t)(-1.0000000000000001e-001)*a[3])*a[4])+(real_t)(1.1000000000000001e+000)));
out[3] = (((((((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[3])+((real_t)(1.0791000000000002e+001)*a[7]))+(u[0]*a[8]))/((((real_t)(-1.0000000000000001e-001)*a[9])*a[10])+(real_t)(1.1000000000000001e+000)))/(real_t)(8.0000000000000004e-001));
out[4] = xd[12];
out[5] = xd[13];
out[6] = xd[14];
out[7] = xd[15];
out[8] = xd[16];
out[9] = xd[17];
out[10] = xd[18];
out[11] = xd[19];
out[12] = (((((((((real_t)(-8.0000000000000016e-002)*a[12])*xd[3])+(((real_t)(-8.0000000000000016e-002)*a[0])*xd[16]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[16]))-((((real_t)(9.8100000000000009e-001)*a[14])*a[2])+(((real_t)(9.8100000000000009e-001)*a[1])*a[16])))*a[17])-((((((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[3])-(((real_t)(9.8100000000000009e-001)*a[1])*a[2]))+u[0])*((((real_t)(-1.0000000000000001e-001)*a[19])*a[4])+(((real_t)(-1.0000000000000001e-001)*a[3])*a[21])))*a[22]));
out[13] = (((((((((real_t)(-8.0000000000000016e-002)*a[23])*xd[3])+(((real_t)(-8.0000000000000016e-002)*a[0])*xd[17]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[17]))-((((real_t)(9.8100000000000009e-001)*a[24])*a[2])+(((real_t)(9.8100000000000009e-001)*a[1])*a[25])))*a[17])-((((((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[3])-(((real_t)(9.8100000000000009e-001)*a[1])*a[2]))+u[0])*((((real_t)(-1.0000000000000001e-001)*a[26])*a[4])+(((real_t)(-1.0000000000000001e-001)*a[3])*a[27])))*a[22]));
out[14] = (((((((((real_t)(-8.0000000000000016e-002)*a[28])*xd[3])+(((real_t)(-8.0000000000000016e-002)*a[0])*xd[18]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[18]))-((((real_t)(9.8100000000000009e-001)*a[29])*a[2])+(((real_t)(9.8100000000000009e-001)*a[1])*a[30])))*a[17])-((((((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[3])-(((real_t)(9.8100000000000009e-001)*a[1])*a[2]))+u[0])*((((real_t)(-1.0000000000000001e-001)*a[31])*a[4])+(((real_t)(-1.0000000000000001e-001)*a[3])*a[32])))*a[22]));
out[15] = (((((((((real_t)(-8.0000000000000016e-002)*a[33])*xd[3])+(((real_t)(-8.0000000000000016e-002)*a[0])*xd[19]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[19]))-((((real_t)(9.8100000000000009e-001)*a[34])*a[2])+(((real_t)(9.8100000000000009e-001)*a[1])*a[35])))*a[17])-((((((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[3])-(((real_t)(9.8100000000000009e-001)*a[1])*a[2]))+u[0])*((((real_t)(-1.0000000000000001e-001)*a[36])*a[4])+(((real_t)(-1.0000000000000001e-001)*a[3])*a[37])))*a[22]));
out[16] = (((((((((((((real_t)(-8.0000000000000016e-002)*a[39])*a[6])+(((real_t)(-8.0000000000000016e-002)*a[5])*a[41]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[16]))*xd[3])+(((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[16]))+((real_t)(1.0791000000000002e+001)*a[43]))+(u[0]*a[45]))*a[46])-(((((((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[3])+((real_t)(1.0791000000000002e+001)*a[7]))+(u[0]*a[8]))*((((real_t)(-1.0000000000000001e-001)*a[48])*a[10])+(((real_t)(-1.0000000000000001e-001)*a[9])*a[50])))*a[51]))*a[52]);
out[17] = (((((((((((((real_t)(-8.0000000000000016e-002)*a[53])*a[6])+(((real_t)(-8.0000000000000016e-002)*a[5])*a[54]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[17]))*xd[3])+(((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[17]))+((real_t)(1.0791000000000002e+001)*a[55]))+(u[0]*a[56]))*a[46])-(((((((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[3])+((real_t)(1.0791000000000002e+001)*a[7]))+(u[0]*a[8]))*((((real_t)(-1.0000000000000001e-001)*a[57])*a[10])+(((real_t)(-1.0000000000000001e-001)*a[9])*a[58])))*a[51]))*a[52]);
out[18] = (((((((((((((real_t)(-8.0000000000000016e-002)*a[59])*a[6])+(((real_t)(-8.0000000000000016e-002)*a[5])*a[60]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[18]))*xd[3])+(((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[18]))+((real_t)(1.0791000000000002e+001)*a[61]))+(u[0]*a[62]))*a[46])-(((((((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[3])+((real_t)(1.0791000000000002e+001)*a[7]))+(u[0]*a[8]))*((((real_t)(-1.0000000000000001e-001)*a[63])*a[10])+(((real_t)(-1.0000000000000001e-001)*a[9])*a[64])))*a[51]))*a[52]);
out[19] = (((((((((((((real_t)(-8.0000000000000016e-002)*a[65])*a[6])+(((real_t)(-8.0000000000000016e-002)*a[5])*a[66]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[19]))*xd[3])+(((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[19]))+((real_t)(1.0791000000000002e+001)*a[67]))+(u[0]*a[68]))*a[46])-(((((((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[3])+((real_t)(1.0791000000000002e+001)*a[7]))+(u[0]*a[8]))*((((real_t)(-1.0000000000000001e-001)*a[69])*a[10])+(((real_t)(-1.0000000000000001e-001)*a[9])*a[70])))*a[51]))*a[52]);
out[20] = xd[22];
out[21] = xd[23];
out[22] = ((((((((((real_t)(-8.0000000000000016e-002)*a[71])*xd[3])+(((real_t)(-8.0000000000000016e-002)*a[0])*xd[23]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[23]))-((((real_t)(9.8100000000000009e-001)*a[72])*a[2])+(((real_t)(9.8100000000000009e-001)*a[1])*a[73])))*a[17])-((((((((real_t)(-8.0000000000000016e-002)*a[0])*xd[3])*xd[3])-(((real_t)(9.8100000000000009e-001)*a[1])*a[2]))+u[0])*((((real_t)(-1.0000000000000001e-001)*a[74])*a[4])+(((real_t)(-1.0000000000000001e-001)*a[3])*a[75])))*a[22]))+a[17]);
out[23] = ((((((((((((((real_t)(-8.0000000000000016e-002)*a[76])*a[6])+(((real_t)(-8.0000000000000016e-002)*a[5])*a[77]))*xd[3])+((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[23]))*xd[3])+(((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[23]))+((real_t)(1.0791000000000002e+001)*a[78]))+(u[0]*a[79]))*a[46])-(((((((((real_t)(-8.0000000000000016e-002)*a[5])*a[6])*xd[3])*xd[3])+((real_t)(1.0791000000000002e+001)*a[7]))+(u[0]*a[8]))*((((real_t)(-1.0000000000000001e-001)*a[80])*a[10])+(((real_t)(-1.0000000000000001e-001)*a[9])*a[81])))*a[51]))*a[52])+((a[8]*a[46])*a[52]));
}

/* Fixed step size:0.025 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+000;
rk_eta[4] = 1.0000000000000000e+000;
rk_eta[5] = 0.0000000000000000e+000;
rk_eta[6] = 0.0000000000000000e+000;
rk_eta[7] = 0.0000000000000000e+000;
rk_eta[8] = 0.0000000000000000e+000;
rk_eta[9] = 1.0000000000000000e+000;
rk_eta[10] = 0.0000000000000000e+000;
rk_eta[11] = 0.0000000000000000e+000;
rk_eta[12] = 0.0000000000000000e+000;
rk_eta[13] = 0.0000000000000000e+000;
rk_eta[14] = 1.0000000000000000e+000;
rk_eta[15] = 0.0000000000000000e+000;
rk_eta[16] = 0.0000000000000000e+000;
rk_eta[17] = 0.0000000000000000e+000;
rk_eta[18] = 0.0000000000000000e+000;
rk_eta[19] = 1.0000000000000000e+000;
rk_eta[20] = 0.0000000000000000e+000;
rk_eta[21] = 0.0000000000000000e+000;
rk_eta[22] = 0.0000000000000000e+000;
rk_eta[23] = 0.0000000000000000e+000;
acadoWorkspace.rk_xxx[24] = rk_eta[24];

for (run1 = 0; run1 < 2; ++run1)
{
acadoWorkspace.rk_xxx[0] = + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + rk_eta[23];
acado_rhs_ext( acadoWorkspace.rk_xxx, acadoWorkspace.rk_kkk );
acadoWorkspace.rk_xxx[0] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[0] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[1] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[2] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[3] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[4] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[5] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[6] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[7] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[8] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[9] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[10] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[11] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[12] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[13] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[14] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[15] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[16] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[17] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[18] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[19] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[20] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[21] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[22] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[23] + rk_eta[23];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 24 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[24] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[25] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[26] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[27] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[28] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[29] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[30] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[31] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[32] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[33] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[34] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[35] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[36] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[37] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[38] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[39] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[40] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[41] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[42] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[43] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[44] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[45] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[46] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[47] + rk_eta[23];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 48 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[48] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[49] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[50] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[51] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[52] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[53] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[54] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[55] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[56] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[57] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[58] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[59] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[60] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[61] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[62] + rk_eta[14];
acadoWorkspace.rk_xxx[15] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[63] + rk_eta[15];
acadoWorkspace.rk_xxx[16] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[64] + rk_eta[16];
acadoWorkspace.rk_xxx[17] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[65] + rk_eta[17];
acadoWorkspace.rk_xxx[18] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[66] + rk_eta[18];
acadoWorkspace.rk_xxx[19] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[67] + rk_eta[19];
acadoWorkspace.rk_xxx[20] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[68] + rk_eta[20];
acadoWorkspace.rk_xxx[21] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[69] + rk_eta[21];
acadoWorkspace.rk_xxx[22] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[70] + rk_eta[22];
acadoWorkspace.rk_xxx[23] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[71] + rk_eta[23];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 72 ]) );
rk_eta[0] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[0] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[24] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[48] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[72];
rk_eta[1] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[1] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[25] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[49] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[73];
rk_eta[2] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[2] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[26] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[50] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[74];
rk_eta[3] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[3] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[27] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[51] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[75];
rk_eta[4] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[4] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[28] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[52] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[76];
rk_eta[5] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[5] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[29] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[53] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[77];
rk_eta[6] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[6] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[30] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[54] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[78];
rk_eta[7] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[7] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[31] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[55] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[79];
rk_eta[8] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[8] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[32] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[56] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[80];
rk_eta[9] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[9] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[33] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[57] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[81];
rk_eta[10] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[10] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[34] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[58] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[82];
rk_eta[11] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[11] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[35] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[59] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[83];
rk_eta[12] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[12] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[36] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[60] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[84];
rk_eta[13] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[13] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[37] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[61] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[85];
rk_eta[14] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[14] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[38] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[62] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[86];
rk_eta[15] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[15] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[39] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[63] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[87];
rk_eta[16] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[16] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[40] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[64] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[88];
rk_eta[17] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[17] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[41] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[65] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[89];
rk_eta[18] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[18] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[42] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[66] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[90];
rk_eta[19] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[19] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[43] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[67] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[91];
rk_eta[20] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[20] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[44] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[68] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[92];
rk_eta[21] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[21] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[45] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[69] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[93];
rk_eta[22] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[22] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[46] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[70] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[94];
rk_eta[23] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[23] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[47] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[71] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[95];
acadoWorkspace.rk_ttt += 5.0000000000000000e-001;
}
error = 0;
return error;
}

