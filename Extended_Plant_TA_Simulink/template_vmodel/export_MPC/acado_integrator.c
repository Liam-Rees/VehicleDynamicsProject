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
const real_t* u = in + 1;
const real_t* od = in + 2;

/* Compute outputs: */
out[0] = ((((real_t)(-2.2981725564623270e+02)*xd[0])/od[0])+((real_t)(3.7957866767887642e-04)*u[0]));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 2;
/* Vector of auxiliary variables; number of elements: 1. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = ((real_t)(1.0000000000000000e+00)/od[0]);

/* Compute outputs: */
out[0] = ((real_t)(-2.2981725564623270e+02)*a[0]);
out[1] = (real_t)(3.7957866767887642e-04);
}



void acado_solve_dim1_triangular( real_t* const A, real_t* const b )
{

b[0] = b[0]/A[0];
}

real_t acado_solve_dim1_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 1; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
det = + det*A[0];

det = fabs(det);
acado_solve_dim1_triangular( A, b );
return det;
}

void acado_solve_dim1_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim1_bPerm[0] = b[rk_perm[0]];

acado_solve_dim1_triangular( A, acadoWorkspace.rk_dim1_bPerm );
b[0] = acadoWorkspace.rk_dim1_bPerm[0];
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
acadoWorkspace.rk_xxx[1] = rk_eta[3];
acadoWorkspace.rk_xxx[2] = rk_eta[4];

for (run = 0; run < 3; ++run)
{
if( run > 0 ) {
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 2] = rk_eta[i + 1];
acadoWorkspace.rk_diffsPrev2[i * 2 + 1] = rk_eta[i + 2];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 2 ]) );
for (j = 0; j < 1; ++j)
{
tmp_index1 = (run1) + (j);
acadoWorkspace.rk_A[tmp_index1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 2) + (j * 2)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
}
det = acado_solve_dim1_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim1_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
}
acado_solve_dim1_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim1_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 2 ]) );
for (j = 0; j < 1; ++j)
{
tmp_index1 = (run1) + (j);
acadoWorkspace.rk_A[tmp_index1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 2) + (j * 2)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_b[i] = - acadoWorkspace.rk_diffsTemp2[(i * 2) + (run1)];
}
if( 0 == run1 ) {
det = acado_solve_dim1_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim1_perm );
}
 else {
acado_solve_dim1_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim1_perm );
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i];
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 2) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 2) + (run1)] += + acadoWorkspace.rk_diffK[i]*(real_t)3.3333333333333335e-03;
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 1; ++j)
{
tmp_index1 = (i) + (j);
tmp_index2 = (run1) + (j * 2);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 2) + (tmp_index2 + 1)];
}
}
acado_solve_dim1_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim1_perm );
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i];
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 2) + (run1 + 1)] = + acadoWorkspace.rk_diffK[i]*(real_t)3.3333333333333335e-03;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)3.3333333333333335e-03;
if( run == 0 ) {
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 1] = acadoWorkspace.rk_diffsNew2[(i * 2) + (j)];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 2] = acadoWorkspace.rk_diffsNew2[(i * 2) + (j + 1)];
}
}
}
else {
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 1] = + acadoWorkspace.rk_diffsNew2[i * 2]*acadoWorkspace.rk_diffsPrev2[j];
}
for (j = 0; j < 1; ++j)
{
tmp_index2 = (j) + (i);
rk_eta[tmp_index2 + 2] = acadoWorkspace.rk_diffsNew2[(i * 2) + (j + 1)];
rk_eta[tmp_index2 + 2] += + acadoWorkspace.rk_diffsNew2[i * 2]*acadoWorkspace.rk_diffsPrev2[j + 1];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 3.3333333333333331e-01;
}
for (i = 0; i < 1; ++i)
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



