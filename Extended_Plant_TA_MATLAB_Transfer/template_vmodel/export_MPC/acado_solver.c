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
for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 2 + 1];

acadoWorkspace.state[14] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.state[15] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.state[16] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.state[17] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.state[18] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.state[19] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.state[20] = acadoVariables.od[lRun1 * 3 + 2];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 2] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 2 + 2];
acadoWorkspace.d[lRun1 * 2 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 2 + 3];

acadoWorkspace.evGx[lRun1 * 4] = acadoWorkspace.state[2];
acadoWorkspace.evGx[lRun1 * 4 + 1] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 4 + 2] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 4 + 3] = acadoWorkspace.state[5];

acadoWorkspace.evGu[lRun1 * 8] = acadoWorkspace.state[6];
acadoWorkspace.evGu[lRun1 * 8 + 1] = acadoWorkspace.state[7];
acadoWorkspace.evGu[lRun1 * 8 + 2] = acadoWorkspace.state[8];
acadoWorkspace.evGu[lRun1 * 8 + 3] = acadoWorkspace.state[9];
acadoWorkspace.evGu[lRun1 * 8 + 4] = acadoWorkspace.state[10];
acadoWorkspace.evGu[lRun1 * 8 + 5] = acadoWorkspace.state[11];
acadoWorkspace.evGu[lRun1 * 8 + 6] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 8 + 7] = acadoWorkspace.state[13];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 2;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = u[0];
out[3] = u[1];
out[4] = u[2];
out[5] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 6;
/* Vector of auxiliary variables; number of elements: 6. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = od[0];
out[1] = a[0];
out[2] = a[1];
out[3] = a[2];
out[4] = a[3];
out[5] = a[4];
out[6] = a[5];
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
tmpQ1[2] = + tmpQ2[6];
tmpQ1[3] = + tmpQ2[7];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[12];
tmpR2[1] = +tmpObjS[13];
tmpR2[2] = +tmpObjS[14];
tmpR2[3] = +tmpObjS[15];
tmpR2[4] = +tmpObjS[16];
tmpR2[5] = +tmpObjS[17];
tmpR2[6] = +tmpObjS[18];
tmpR2[7] = +tmpObjS[19];
tmpR2[8] = +tmpObjS[20];
tmpR2[9] = +tmpObjS[21];
tmpR2[10] = +tmpObjS[22];
tmpR2[11] = +tmpObjS[23];
tmpR2[12] = +tmpObjS[24];
tmpR2[13] = +tmpObjS[25];
tmpR2[14] = +tmpObjS[26];
tmpR2[15] = +tmpObjS[27];
tmpR2[16] = +tmpObjS[28];
tmpR2[17] = +tmpObjS[29];
tmpR2[18] = +tmpObjS[30];
tmpR2[19] = +tmpObjS[31];
tmpR2[20] = +tmpObjS[32];
tmpR2[21] = +tmpObjS[33];
tmpR2[22] = +tmpObjS[34];
tmpR2[23] = +tmpObjS[35];
tmpR1[0] = + tmpR2[2];
tmpR1[1] = + tmpR2[3];
tmpR1[2] = + tmpR2[4];
tmpR1[3] = + tmpR2[5];
tmpR1[4] = + tmpR2[8];
tmpR1[5] = + tmpR2[9];
tmpR1[6] = + tmpR2[10];
tmpR1[7] = + tmpR2[11];
tmpR1[8] = + tmpR2[14];
tmpR1[9] = + tmpR2[15];
tmpR1[10] = + tmpR2[16];
tmpR1[11] = + tmpR2[17];
tmpR1[12] = + tmpR2[20];
tmpR1[13] = + tmpR2[21];
tmpR1[14] = + tmpR2[22];
tmpR1[15] = + tmpR2[23];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 5; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 4 + 3];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 3];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 3 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 6] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 6 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 6 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 6 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 6 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 6 + 5] = acadoWorkspace.objValueOut[5];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 4 ]), &(acadoWorkspace.Q2[ runObj * 12 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 16 ]), &(acadoWorkspace.R2[ runObj * 24 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[11];
acadoWorkspace.objValueIn[2] = acadoVariables.od[15];
acadoWorkspace.objValueIn[3] = acadoVariables.od[16];
acadoWorkspace.objValueIn[4] = acadoVariables.od[17];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[2];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[3];
Gx3[2] = + Gx1[2]*Gx2[0] + Gx1[3]*Gx2[2];
Gx3[3] = + Gx1[2]*Gx2[1] + Gx1[3]*Gx2[3];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[3]*Gu1[4];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[3]*Gu1[5];
Gu2[6] = + Gx1[2]*Gu1[2] + Gx1[3]*Gu1[6];
Gu2[7] = + Gx1[2]*Gu1[3] + Gx1[3]*Gu1[7];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 4)] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4];
acadoWorkspace.H[(iRow * 80) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5];
acadoWorkspace.H[(iRow * 80) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6];
acadoWorkspace.H[(iRow * 80) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7];
acadoWorkspace.H[(iRow * 80 + 20) + (iCol * 4)] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4];
acadoWorkspace.H[(iRow * 80 + 20) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5];
acadoWorkspace.H[(iRow * 80 + 20) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6];
acadoWorkspace.H[(iRow * 80 + 20) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 4)] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7];
acadoWorkspace.H[(iRow * 80 + 60) + (iCol * 4)] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4];
acadoWorkspace.H[(iRow * 80 + 60) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5];
acadoWorkspace.H[(iRow * 80 + 60) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6];
acadoWorkspace.H[(iRow * 80 + 60) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 84] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + R11[0];
acadoWorkspace.H[iRow * 84 + 1] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + R11[1];
acadoWorkspace.H[iRow * 84 + 2] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + R11[2];
acadoWorkspace.H[iRow * 84 + 3] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + R11[3];
acadoWorkspace.H[iRow * 84 + 20] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + R11[4];
acadoWorkspace.H[iRow * 84 + 21] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + R11[5];
acadoWorkspace.H[iRow * 84 + 22] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + R11[6];
acadoWorkspace.H[iRow * 84 + 23] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + R11[7];
acadoWorkspace.H[iRow * 84 + 40] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + R11[8];
acadoWorkspace.H[iRow * 84 + 41] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + R11[9];
acadoWorkspace.H[iRow * 84 + 42] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + R11[10];
acadoWorkspace.H[iRow * 84 + 43] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + R11[11];
acadoWorkspace.H[iRow * 84 + 60] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + R11[12];
acadoWorkspace.H[iRow * 84 + 61] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + R11[13];
acadoWorkspace.H[iRow * 84 + 62] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + R11[14];
acadoWorkspace.H[iRow * 84 + 63] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + R11[15];
acadoWorkspace.H[iRow * 84] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 84 + 21] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 84 + 42] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 84 + 63] += 1.0000000000000000e-04;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[2]*Gu1[4];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[2]*Gu1[5];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[2]*Gu1[6];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[2]*Gu1[7];
Gu2[4] = + Gx1[1]*Gu1[0] + Gx1[3]*Gu1[4];
Gu2[5] = + Gx1[1]*Gu1[1] + Gx1[3]*Gu1[5];
Gu2[6] = + Gx1[1]*Gu1[2] + Gx1[3]*Gu1[6];
Gu2[7] = + Gx1[1]*Gu1[3] + Gx1[3]*Gu1[7];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[4] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[5] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[6] + Gu2[2];
Gu3[3] = + Q11[0]*Gu1[3] + Q11[1]*Gu1[7] + Gu2[3];
Gu3[4] = + Q11[2]*Gu1[0] + Q11[3]*Gu1[4] + Gu2[4];
Gu3[5] = + Q11[2]*Gu1[1] + Q11[3]*Gu1[5] + Gu2[5];
Gu3[6] = + Q11[2]*Gu1[2] + Q11[3]*Gu1[6] + Gu2[6];
Gu3[7] = + Q11[2]*Gu1[3] + Q11[3]*Gu1[7] + Gu2[7];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[2]*w11[1] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[3]*w11[1] + w12[1];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + w12[0];
w13[1] = + Q11[2]*w11[0] + Q11[3]*w11[1] + w12[1];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1];
w12[1] += + Gx1[2]*w11[0] + Gx1[3]*w11[1];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1];
w12[1] += + Gx1[2]*w11[0] + Gx1[3]*w11[1];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
w12[1] += + Gu1[4]*U1[0] + Gu1[5]*U1[1] + Gu1[6]*U1[2] + Gu1[7]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 80) + (iCol * 4)] = acadoWorkspace.H[(iCol * 80) + (iRow * 4)];
acadoWorkspace.H[(iRow * 80) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 80 + 20) + (iRow * 4)];
acadoWorkspace.H[(iRow * 80) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 4)];
acadoWorkspace.H[(iRow * 80) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 80 + 60) + (iRow * 4)];
acadoWorkspace.H[(iRow * 80 + 20) + (iCol * 4)] = acadoWorkspace.H[(iCol * 80) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 80 + 20) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 80 + 20) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 80 + 20) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 80 + 20) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 80 + 60) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 4)] = acadoWorkspace.H[(iCol * 80) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 80 + 20) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 80 + 40) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 80 + 60) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 80 + 60) + (iCol * 4)] = acadoWorkspace.H[(iCol * 80) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 80 + 60) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 80 + 20) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 80 + 60) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 80 + 40) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 80 + 60) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 80 + 60) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5];
RDy1[1] = + R2[6]*Dy1[0] + R2[7]*Dy1[1] + R2[8]*Dy1[2] + R2[9]*Dy1[3] + R2[10]*Dy1[4] + R2[11]*Dy1[5];
RDy1[2] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5];
RDy1[3] = + R2[18]*Dy1[0] + R2[19]*Dy1[1] + R2[20]*Dy1[2] + R2[21]*Dy1[3] + R2[22]*Dy1[4] + R2[23]*Dy1[5];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5];
QDy1[1] = + Q2[6]*Dy1[0] + Q2[7]*Dy1[1] + Q2[8]*Dy1[2] + Q2[9]*Dy1[3] + Q2[10]*Dy1[4] + Q2[11]*Dy1[5];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 20) + (col * 4)] = + Hx[0]*E[0] + Hx[1]*E[4];
acadoWorkspace.A[(row * 20) + (col * 4 + 1)] = + Hx[0]*E[1] + Hx[1]*E[5];
acadoWorkspace.A[(row * 20) + (col * 4 + 2)] = + Hx[0]*E[2] + Hx[1]*E[6];
acadoWorkspace.A[(row * 20) + (col * 4 + 3)] = + Hx[0]*E[3] + Hx[1]*E[7];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_condensePrep(  )
{
int lRun1;
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 4 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.C[ 4 ]), &(acadoWorkspace.C[ 8 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.C[ 8 ]), &(acadoWorkspace.C[ 12 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.C[ 12 ]), &(acadoWorkspace.C[ 16 ]) );
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 8 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.E[ 8 ]), &(acadoWorkspace.E[ 16 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 16 ]), &(acadoWorkspace.E[ 24 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 32 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 32 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.E[ 24 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.W1, 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.E[ 16 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 8 ]), &(acadoWorkspace.E[ 8 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 4 ]), acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.R1, acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.E[ 40 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 56 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 56 ]), &(acadoWorkspace.E[ 64 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 64 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.E[ 56 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.W1, 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.E[ 48 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 8 ]), &(acadoWorkspace.E[ 40 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 16 ]), &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 80 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 88 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 88 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.E[ 80 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.W1, 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.E[ 72 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 32 ]), &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 104 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 104 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.E[ 96 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 48 ]), &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.E[ 112 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 112 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 64 ]), &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.W1, 4 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );

acadoWorkspace.sbar[2] = acadoWorkspace.d[0];
acadoWorkspace.sbar[3] = acadoWorkspace.d[1];
acadoWorkspace.sbar[4] = acadoWorkspace.d[2];
acadoWorkspace.sbar[5] = acadoWorkspace.d[3];
acadoWorkspace.sbar[6] = acadoWorkspace.d[4];
acadoWorkspace.sbar[7] = acadoWorkspace.d[5];
acadoWorkspace.sbar[8] = acadoWorkspace.d[6];
acadoWorkspace.sbar[9] = acadoWorkspace.d[7];
acadoWorkspace.sbar[10] = acadoWorkspace.d[8];
acadoWorkspace.sbar[11] = acadoWorkspace.d[9];
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - acadoVariables.u[19];

for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 2 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.conValueIn[3] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[8] = acadoVariables.od[lRun1 * 3 + 2];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 2] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 2 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[6];
}



acado_multHxE( &(acadoWorkspace.evHx[ 2 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 4 ]), &(acadoWorkspace.E[ 8 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 4 ]), &(acadoWorkspace.E[ 40 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 6 ]), &(acadoWorkspace.E[ 16 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 6 ]), &(acadoWorkspace.E[ 48 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 6 ]), &(acadoWorkspace.E[ 72 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.E[ 24 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.E[ 56 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.E[ 80 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.E[ 96 ]), 4, 3 );

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[3] = acadoWorkspace.evHu[3];
acadoWorkspace.A[24] = acadoWorkspace.evHu[4];
acadoWorkspace.A[25] = acadoWorkspace.evHu[5];
acadoWorkspace.A[26] = acadoWorkspace.evHu[6];
acadoWorkspace.A[27] = acadoWorkspace.evHu[7];
acadoWorkspace.A[48] = acadoWorkspace.evHu[8];
acadoWorkspace.A[49] = acadoWorkspace.evHu[9];
acadoWorkspace.A[50] = acadoWorkspace.evHu[10];
acadoWorkspace.A[51] = acadoWorkspace.evHu[11];
acadoWorkspace.A[72] = acadoWorkspace.evHu[12];
acadoWorkspace.A[73] = acadoWorkspace.evHu[13];
acadoWorkspace.A[74] = acadoWorkspace.evHu[14];
acadoWorkspace.A[75] = acadoWorkspace.evHu[15];
acadoWorkspace.A[96] = acadoWorkspace.evHu[16];
acadoWorkspace.A[97] = acadoWorkspace.evHu[17];
acadoWorkspace.A[98] = acadoWorkspace.evHu[18];
acadoWorkspace.A[99] = acadoWorkspace.evHu[19];
acadoWorkspace.lbA[0] = - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = - acadoWorkspace.evH[4];

acadoWorkspace.ubA[0] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[4];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 16 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 12 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 2 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 24 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 8 ]) );

acadoWorkspace.QDy[10] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[11] = + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 2 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 10 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[10] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[11] + acadoWorkspace.QDy[10];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[10] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[11] + acadoWorkspace.QDy[11];
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 2 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4 ]), &(acadoWorkspace.sbar[ 2 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );



acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 2 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 4 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 6 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );

}

void acado_expand(  )
{
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
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.d[0];
acadoWorkspace.sbar[3] = acadoWorkspace.d[1];
acadoWorkspace.sbar[4] = acadoWorkspace.d[2];
acadoWorkspace.sbar[5] = acadoWorkspace.d[3];
acadoWorkspace.sbar[6] = acadoWorkspace.d[4];
acadoWorkspace.sbar[7] = acadoWorkspace.d[5];
acadoWorkspace.sbar[8] = acadoWorkspace.d[6];
acadoWorkspace.sbar[9] = acadoWorkspace.d[7];
acadoWorkspace.sbar[10] = acadoWorkspace.d[8];
acadoWorkspace.sbar[11] = acadoWorkspace.d[9];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 2 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 10 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
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
for (index = 0; index < 5; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 2];
acadoWorkspace.state[1] = acadoVariables.x[index * 2 + 1];
acadoWorkspace.state[14] = acadoVariables.u[index * 4];
acadoWorkspace.state[15] = acadoVariables.u[index * 4 + 1];
acadoWorkspace.state[16] = acadoVariables.u[index * 4 + 2];
acadoWorkspace.state[17] = acadoVariables.u[index * 4 + 3];
acadoWorkspace.state[18] = acadoVariables.od[index * 3];
acadoWorkspace.state[19] = acadoVariables.od[index * 3 + 1];
acadoWorkspace.state[20] = acadoVariables.od[index * 3 + 2];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 2 + 2] = acadoWorkspace.state[0];
acadoVariables.x[index * 2 + 3] = acadoWorkspace.state[1];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 5; ++index)
{
acadoVariables.x[index * 2] = acadoVariables.x[index * 2 + 2];
acadoVariables.x[index * 2 + 1] = acadoVariables.x[index * 2 + 3];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[10] = xEnd[0];
acadoVariables.x[11] = xEnd[1];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[10];
acadoWorkspace.state[1] = acadoVariables.x[11];
if (uEnd != 0)
{
acadoWorkspace.state[14] = uEnd[0];
acadoWorkspace.state[15] = uEnd[1];
acadoWorkspace.state[16] = uEnd[2];
acadoWorkspace.state[17] = uEnd[3];
}
else
{
acadoWorkspace.state[14] = acadoVariables.u[16];
acadoWorkspace.state[15] = acadoVariables.u[17];
acadoWorkspace.state[16] = acadoVariables.u[18];
acadoWorkspace.state[17] = acadoVariables.u[19];
}
acadoWorkspace.state[18] = acadoVariables.od[15];
acadoWorkspace.state[19] = acadoVariables.od[16];
acadoWorkspace.state[20] = acadoVariables.od[17];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[10] = acadoWorkspace.state[0];
acadoVariables.x[11] = acadoWorkspace.state[1];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 4; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[16] = uEnd[0];
acadoVariables.u[17] = uEnd[1];
acadoVariables.u[18] = uEnd[2];
acadoVariables.u[19] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 5; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 6 */
real_t tmpDy[ 6 ];

/** Row vector of size: 2 */
real_t tmpDyN[ 2 ];

for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 2];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 2 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[8] = acadoVariables.od[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 6] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 6];
acadoWorkspace.Dy[lRun1 * 6 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 6 + 1];
acadoWorkspace.Dy[lRun1 * 6 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 6 + 2];
acadoWorkspace.Dy[lRun1 * 6 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 6 + 3];
acadoWorkspace.Dy[lRun1 * 6 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 6 + 4];
acadoWorkspace.Dy[lRun1 * 6 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 6 + 5];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[10];
acadoWorkspace.objValueIn[1] = acadoVariables.x[11];
acadoWorkspace.objValueIn[2] = acadoVariables.od[15];
acadoWorkspace.objValueIn[3] = acadoVariables.od[16];
acadoWorkspace.objValueIn[4] = acadoVariables.od[17];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 5; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 6]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 6 + 1]*acadoVariables.W[7];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 6 + 2]*acadoVariables.W[14];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 6 + 3]*acadoVariables.W[21];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 6 + 4]*acadoVariables.W[28];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 6 + 5]*acadoVariables.W[35];
objVal += + acadoWorkspace.Dy[lRun1 * 6]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 6 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 6 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 6 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 6 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 6 + 5]*tmpDy[5];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[3];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1];

objVal *= 0.5;
return objVal;
}

