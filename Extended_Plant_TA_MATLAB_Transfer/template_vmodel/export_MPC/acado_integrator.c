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
const real_t* u = in + 2;
const real_t* od = in + 6;

/* Compute outputs: */
out[0] = ((((real_t)(-2.2463768115942028e+02)/od[0])*xd[0])+((((real_t)(7.3231884057971016e+01)/od[0])-od[0])*xd[1]));
out[1] = ((((((real_t)(0.0000000000000000e+00)-(((((((((real_t)(-1.5061290322580643e+02)*od[1])+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.4111374193548390e+03))/(((((((((((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))+(real_t)(3.3577625806451615e+03))+(real_t)(3.4111374193548390e+03))+(real_t)(3.4111374193548390e+03)))*u[0])-((((real_t)(1.0000000000000000e+00)/(((((((((((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))+(real_t)(3.3577625806451615e+03))+(real_t)(3.4111374193548390e+03))+(real_t)(3.4111374193548390e+03)))*((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.4111374193548390e+03)))*u[1]))+((((real_t)(1.0000000000000000e+00)/(((((((((((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))+(real_t)(3.3577625806451615e+03))+(real_t)(3.4111374193548390e+03))+(real_t)(3.4111374193548390e+03)))*((((real_t)(1.5061290322580643e+02)*od[1])+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03)))*u[2]))-((((real_t)(1.0000000000000000e+00)/(((((((((((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))+(real_t)(3.3577625806451615e+03))+(real_t)(3.4111374193548390e+03))+(real_t)(3.4111374193548390e+03)))*((((real_t)(1.5061290322580643e+02)*od[1])-((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03)))*u[3])))*(real_t)(7.8800000000000003e-01))/(real_t)(7.9957074999999998e+02))+(((real_t)(3.8360220155627253e+01)/od[0])*xd[0]))-(((real_t)(2.2981725564623270e+02)*xd[1])/od[0]));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 2. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((real_t)(1.0000000000000000e+00)/od[0]);
a[1] = ((real_t)(1.0000000000000000e+00)/(real_t)(7.9957074999999998e+02));

/* Compute outputs: */
out[0] = ((real_t)(-2.2463768115942028e+02)/od[0]);
out[1] = (((real_t)(7.3231884057971016e+01)/od[0])-od[0]);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = ((real_t)(3.8360220155627253e+01)/od[0]);
out[7] = ((real_t)(0.0000000000000000e+00)-((real_t)(2.2981725564623270e+02)*a[0]));
out[8] = ((((real_t)(0.0000000000000000e+00)-(((((real_t)(-1.5061290322580643e+02)*od[1])+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.4111374193548390e+03))/(((((((((((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))+(real_t)(3.3577625806451615e+03))+(real_t)(3.4111374193548390e+03))+(real_t)(3.4111374193548390e+03))))*(real_t)(7.8800000000000003e-01))*a[1]);
out[9] = ((((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(((real_t)(1.0000000000000000e+00)/(((((((((((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))+(real_t)(3.3577625806451615e+03))+(real_t)(3.4111374193548390e+03))+(real_t)(3.4111374193548390e+03)))*((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.4111374193548390e+03)))))*(real_t)(7.8800000000000003e-01))*a[1]);
out[10] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(1.0000000000000000e+00)/(((((((((((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))+(real_t)(3.3577625806451615e+03))+(real_t)(3.4111374193548390e+03))+(real_t)(3.4111374193548390e+03)))*((((real_t)(1.5061290322580643e+02)*od[1])+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))))*(real_t)(7.8800000000000003e-01))*a[1]);
out[11] = ((((real_t)(0.0000000000000000e+00)-((real_t)(0.0000000000000000e+00)-(((real_t)(1.0000000000000000e+00)/(((((((((((((real_t)(-1.5061290322580643e+02)*od[1])-((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))+((real_t)(1.5061290322580643e+02)*od[1]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))-((real_t)(2.6663071065989845e+02)*od[2]))+((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03))+(real_t)(3.3577625806451615e+03))+(real_t)(3.4111374193548390e+03))+(real_t)(3.4111374193548390e+03)))*((((real_t)(1.5061290322580643e+02)*od[1])-((real_t)(2.6663071065989845e+02)*od[2]))+(real_t)(3.3577625806451615e+03)))))*(real_t)(7.8800000000000003e-01))*a[1]);
}



void acado_solve_dim2_triangular( real_t* const A, real_t* const b )
{

b[1] = b[1]/A[3];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim2_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 2; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
if(fabs(A[2]) > fabs(A[0])) {
acadoWorkspace.rk_dim2_swap = A[0];
A[0] = A[2];
A[2] = acadoWorkspace.rk_dim2_swap;
acadoWorkspace.rk_dim2_swap = A[1];
A[1] = A[3];
A[3] = acadoWorkspace.rk_dim2_swap;
acadoWorkspace.rk_dim2_swap = b[0];
b[0] = b[1];
b[1] = acadoWorkspace.rk_dim2_swap;
intSwap = rk_perm[0];
rk_perm[0] = rk_perm[1];
rk_perm[1] = intSwap;
}

A[2] = -A[2]/A[0];
A[3] += + A[2]*A[1];
b[1] += + A[2]*b[0];

det = + det*A[0];

det = + det*A[3];

det = fabs(det);
acado_solve_dim2_triangular( A, b );
return det;
}

void acado_solve_dim2_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim2_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim2_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim2_bPerm[1] += A[2]*acadoWorkspace.rk_dim2_bPerm[0];


acado_solve_dim2_triangular( A, acadoWorkspace.rk_dim2_bPerm );
b[0] = acadoWorkspace.rk_dim2_bPerm[0];
b[1] = acadoWorkspace.rk_dim2_bPerm[1];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 1.6666666666666668e-03 };


/* Fixed step size:0.00333333 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[2] = rk_eta[14];
acadoWorkspace.rk_xxx[3] = rk_eta[15];
acadoWorkspace.rk_xxx[4] = rk_eta[16];
acadoWorkspace.rk_xxx[5] = rk_eta[17];
acadoWorkspace.rk_xxx[6] = rk_eta[18];
acadoWorkspace.rk_xxx[7] = rk_eta[19];
acadoWorkspace.rk_xxx[8] = rk_eta[20];

for (run = 0; run < 3; ++run)
{
if( run > 0 ) {
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 6] = rk_eta[i * 2 + 2];
acadoWorkspace.rk_diffsPrev2[i * 6 + 1] = rk_eta[i * 2 + 3];
acadoWorkspace.rk_diffsPrev2[i * 6 + 2] = rk_eta[i * 4 + 6];
acadoWorkspace.rk_diffsPrev2[i * 6 + 3] = rk_eta[i * 4 + 7];
acadoWorkspace.rk_diffsPrev2[i * 6 + 4] = rk_eta[i * 4 + 8];
acadoWorkspace.rk_diffsPrev2[i * 6 + 5] = rk_eta[i * 4 + 9];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 12 ]) );
for (j = 0; j < 2; ++j)
{
tmp_index1 = (run1 * 2) + (j);
acadoWorkspace.rk_A[tmp_index1 * 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 2 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 6 + 1)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 2) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 2] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 2 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
}
det = acado_solve_dim2_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim2_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 2];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 2 + 1];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 2] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 2 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
}
acado_solve_dim2_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim2_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 2];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 2 + 1];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 2; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 12 ]) );
for (j = 0; j < 2; ++j)
{
tmp_index1 = (run1 * 2) + (j);
acadoWorkspace.rk_A[tmp_index1 * 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 6)];
acadoWorkspace.rk_A[tmp_index1 * 2 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 12) + (j * 6 + 1)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 2) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_b[i * 2] = - acadoWorkspace.rk_diffsTemp2[(i * 12) + (run1)];
acadoWorkspace.rk_b[i * 2 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 12) + (run1 + 6)];
}
if( 0 == run1 ) {
det = acado_solve_dim2_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim2_perm );
}
 else {
acado_solve_dim2_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim2_perm );
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 2];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 2 + 1];
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1)] += + acadoWorkspace.rk_diffK[i]*(real_t)3.3333333333333335e-03;
}
}
for (run1 = 0; run1 < 4; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 2; ++j)
{
tmp_index1 = (i * 2) + (j);
tmp_index2 = (run1) + (j * 6);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 12) + (tmp_index2 + 2)];
}
}
acado_solve_dim2_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim2_perm );
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 2];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 2 + 1];
}
for (i = 0; i < 2; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 6) + (run1 + 2)] = + acadoWorkspace.rk_diffK[i]*(real_t)3.3333333333333335e-03;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)3.3333333333333335e-03;
rk_eta[1] += + acadoWorkspace.rk_kkk[1]*(real_t)3.3333333333333335e-03;
if( run == 0 ) {
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 2] = acadoWorkspace.rk_diffsNew2[(i * 6) + (j)];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 6] = acadoWorkspace.rk_diffsNew2[(i * 6) + (j + 2)];
}
}
}
else {
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 2] = + acadoWorkspace.rk_diffsNew2[i * 6]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 2] += + acadoWorkspace.rk_diffsNew2[i * 6 + 1]*acadoWorkspace.rk_diffsPrev2[j + 6];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 6] = acadoWorkspace.rk_diffsNew2[(i * 6) + (j + 2)];
rk_eta[tmp_index2 + 6] += + acadoWorkspace.rk_diffsNew2[i * 6]*acadoWorkspace.rk_diffsPrev2[j + 2];
rk_eta[tmp_index2 + 6] += + acadoWorkspace.rk_diffsNew2[i * 6 + 1]*acadoWorkspace.rk_diffsPrev2[j + 8];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 3.3333333333333331e-01;
}
for (i = 0; i < 2; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



