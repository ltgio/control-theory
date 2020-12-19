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
const real_t* u = in + 3;
/* Vector of auxiliary variables; number of elements: 2. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (pow((((real_t)(1.0000000000000000e+000)/xd[0])*xd[2]),2));
a[1] = ((xd[0])*(xd[0]));

/* Compute outputs: */
out[0] = xd[1];
out[1] = (((real_t)(-9.8100000000000000e+002)*a[0])+(real_t)(9.8100000000000005e+000));
out[2] = (((((real_t)(-5.0000000000000000e+001)*xd[2])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[1]))+u[0])/(((real_t)(1.9620000000000001e+001)/xd[0])+(real_t)(5.0000000000000000e-001)));
}

void acado_rhs_ext(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 15;
/* Vector of auxiliary variables; number of elements: 20. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (pow((((real_t)(1.0000000000000000e+000)/xd[0])*xd[2]),2));
a[1] = ((xd[0])*(xd[0]));
a[2] = ((real_t)(2.0000000000000000e+000)*(((real_t)(1.0000000000000000e+000)/xd[0])*xd[2]));
a[3] = ((real_t)(1.0000000000000000e+000)/xd[0]);
a[4] = (a[3]*a[3]);
a[5] = (a[2]*((((real_t)(0.0000000000000000e+000)-(xd[3]*a[4]))*xd[2])+(((real_t)(1.0000000000000000e+000)/xd[0])*xd[9])));
a[6] = (a[2]*((((real_t)(0.0000000000000000e+000)-(xd[4]*a[4]))*xd[2])+(((real_t)(1.0000000000000000e+000)/xd[0])*xd[10])));
a[7] = (a[2]*((((real_t)(0.0000000000000000e+000)-(xd[5]*a[4]))*xd[2])+(((real_t)(1.0000000000000000e+000)/xd[0])*xd[11])));
a[8] = ((real_t)(2.0000000000000000e+000)*xd[0]);
a[9] = (a[8]*xd[3]);
a[10] = ((real_t)(1.0000000000000000e+000)/a[1]);
a[11] = (a[10]*a[10]);
a[12] = ((real_t)(1.0000000000000000e+000)/(((real_t)(1.9620000000000001e+001)/xd[0])+(real_t)(5.0000000000000000e-001)));
a[13] = ((real_t)(1.0000000000000000e+000)/xd[0]);
a[14] = (a[13]*a[13]);
a[15] = (a[12]*a[12]);
a[16] = (a[8]*xd[4]);
a[17] = (a[8]*xd[5]);
a[18] = (a[2]*((((real_t)(0.0000000000000000e+000)-(xd[12]*a[4]))*xd[2])+(((real_t)(1.0000000000000000e+000)/xd[0])*xd[14])));
a[19] = (a[8]*xd[12]);

/* Compute outputs: */
out[0] = xd[1];
out[1] = (((real_t)(-9.8100000000000000e+002)*a[0])+(real_t)(9.8100000000000005e+000));
out[2] = (((((real_t)(-5.0000000000000000e+001)*xd[2])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[1]))+u[0])/(((real_t)(1.9620000000000001e+001)/xd[0])+(real_t)(5.0000000000000000e-001)));
out[3] = xd[6];
out[4] = xd[7];
out[5] = xd[8];
out[6] = ((real_t)(-9.8100000000000000e+002)*a[5]);
out[7] = ((real_t)(-9.8100000000000000e+002)*a[6]);
out[8] = ((real_t)(-9.8100000000000000e+002)*a[7]);
out[9] = (((((real_t)(-5.0000000000000000e+001)*xd[9])+((((real_t)(0.0000000000000000e+000)-(((real_t)(1.9620000000000001e+001)*a[9])*a[11]))*xd[1])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[6])))*a[12])-((((((real_t)(-5.0000000000000000e+001)*xd[2])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[1]))+u[0])*((real_t)(0.0000000000000000e+000)-(((real_t)(1.9620000000000001e+001)*xd[3])*a[14])))*a[15]));
out[10] = (((((real_t)(-5.0000000000000000e+001)*xd[10])+((((real_t)(0.0000000000000000e+000)-(((real_t)(1.9620000000000001e+001)*a[16])*a[11]))*xd[1])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[7])))*a[12])-((((((real_t)(-5.0000000000000000e+001)*xd[2])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[1]))+u[0])*((real_t)(0.0000000000000000e+000)-(((real_t)(1.9620000000000001e+001)*xd[4])*a[14])))*a[15]));
out[11] = (((((real_t)(-5.0000000000000000e+001)*xd[11])+((((real_t)(0.0000000000000000e+000)-(((real_t)(1.9620000000000001e+001)*a[17])*a[11]))*xd[1])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[8])))*a[12])-((((((real_t)(-5.0000000000000000e+001)*xd[2])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[1]))+u[0])*((real_t)(0.0000000000000000e+000)-(((real_t)(1.9620000000000001e+001)*xd[5])*a[14])))*a[15]));
out[12] = xd[13];
out[13] = ((real_t)(-9.8100000000000000e+002)*a[18]);
out[14] = ((((((real_t)(-5.0000000000000000e+001)*xd[14])+((((real_t)(0.0000000000000000e+000)-(((real_t)(1.9620000000000001e+001)*a[19])*a[11]))*xd[1])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[13])))*a[12])-((((((real_t)(-5.0000000000000000e+001)*xd[2])+(((real_t)(1.9620000000000001e+001)/a[1])*xd[1]))+u[0])*((real_t)(0.0000000000000000e+000)-(((real_t)(1.9620000000000001e+001)*xd[12])*a[14])))*a[15]))+a[12]);
}

/* Fixed step size:0.025 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int run1;
acadoWorkspace.rk_ttt = 0.0000000000000000e+000;
rk_eta[3] = 1.0000000000000000e+000;
rk_eta[4] = 0.0000000000000000e+000;
rk_eta[5] = 0.0000000000000000e+000;
rk_eta[6] = 0.0000000000000000e+000;
rk_eta[7] = 1.0000000000000000e+000;
rk_eta[8] = 0.0000000000000000e+000;
rk_eta[9] = 0.0000000000000000e+000;
rk_eta[10] = 0.0000000000000000e+000;
rk_eta[11] = 1.0000000000000000e+000;
rk_eta[12] = 0.0000000000000000e+000;
rk_eta[13] = 0.0000000000000000e+000;
rk_eta[14] = 0.0000000000000000e+000;
acadoWorkspace.rk_xxx[15] = rk_eta[15];

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
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 15 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[15] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[16] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[17] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[18] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[19] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[20] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[21] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[22] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[23] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[24] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[25] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[26] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[27] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[28] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)1.2500000000000001e-002*acadoWorkspace.rk_kkk[29] + rk_eta[14];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 30 ]) );
acadoWorkspace.rk_xxx[0] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[30] + rk_eta[0];
acadoWorkspace.rk_xxx[1] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[31] + rk_eta[1];
acadoWorkspace.rk_xxx[2] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[32] + rk_eta[2];
acadoWorkspace.rk_xxx[3] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[33] + rk_eta[3];
acadoWorkspace.rk_xxx[4] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[34] + rk_eta[4];
acadoWorkspace.rk_xxx[5] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[35] + rk_eta[5];
acadoWorkspace.rk_xxx[6] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[36] + rk_eta[6];
acadoWorkspace.rk_xxx[7] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[37] + rk_eta[7];
acadoWorkspace.rk_xxx[8] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[38] + rk_eta[8];
acadoWorkspace.rk_xxx[9] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[39] + rk_eta[9];
acadoWorkspace.rk_xxx[10] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[40] + rk_eta[10];
acadoWorkspace.rk_xxx[11] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[41] + rk_eta[11];
acadoWorkspace.rk_xxx[12] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[42] + rk_eta[12];
acadoWorkspace.rk_xxx[13] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[43] + rk_eta[13];
acadoWorkspace.rk_xxx[14] = + (real_t)2.5000000000000001e-002*acadoWorkspace.rk_kkk[44] + rk_eta[14];
acado_rhs_ext( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_kkk[ 45 ]) );
rk_eta[0] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[0] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[15] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[30] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[45];
rk_eta[1] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[1] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[16] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[31] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[46];
rk_eta[2] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[2] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[17] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[32] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[47];
rk_eta[3] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[3] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[18] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[33] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[48];
rk_eta[4] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[4] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[19] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[34] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[49];
rk_eta[5] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[5] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[20] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[35] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[50];
rk_eta[6] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[6] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[21] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[36] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[51];
rk_eta[7] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[7] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[22] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[37] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[52];
rk_eta[8] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[8] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[23] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[38] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[53];
rk_eta[9] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[9] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[24] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[39] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[54];
rk_eta[10] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[10] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[25] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[40] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[55];
rk_eta[11] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[11] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[26] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[41] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[56];
rk_eta[12] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[12] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[27] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[42] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[57];
rk_eta[13] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[13] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[28] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[43] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[58];
rk_eta[14] += + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[14] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[29] + (real_t)8.3333333333333332e-003*acadoWorkspace.rk_kkk[44] + (real_t)4.1666666666666666e-003*acadoWorkspace.rk_kkk[59];
acadoWorkspace.rk_ttt += 5.0000000000000000e-001;
}
error = 0;
return error;
}

