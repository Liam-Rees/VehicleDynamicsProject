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
acadoWorkspace.state[0] = acadoVariables.x[lRun1];

acadoWorkspace.state[6] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.state[7] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.state[8] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.state[9] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.state[10] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.state[11] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.state[12] = acadoVariables.od[lRun1 * 3 + 2];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 + 1];

acadoWorkspace.evGx[lRun1] = acadoWorkspace.state[1];

acadoWorkspace.evGu[lRun1 * 4] = acadoWorkspace.state[2];
acadoWorkspace.evGu[lRun1 * 4 + 1] = acadoWorkspace.state[3];
acadoWorkspace.evGu[lRun1 * 4 + 2] = acadoWorkspace.state[4];
acadoWorkspace.evGu[lRun1 * 4 + 3] = acadoWorkspace.state[5];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 1;

/* Compute outputs: */
out[0] = xd[0];
out[1] = u[0];
out[2] = u[1];
out[3] = u[2];
out[4] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 5;
/* Vector of auxiliary variables; number of elements: 5. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (real_t)(0.0000000000000000e+00);
a[1] = (real_t)(0.0000000000000000e+00);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = od[0];
out[1] = a[0];
out[2] = a[1];
out[3] = a[2];
out[4] = a[3];
out[5] = a[4];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ1[0] = + tmpQ2[0];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[5];
tmpR2[1] = +tmpObjS[6];
tmpR2[2] = +tmpObjS[7];
tmpR2[3] = +tmpObjS[8];
tmpR2[4] = +tmpObjS[9];
tmpR2[5] = +tmpObjS[10];
tmpR2[6] = +tmpObjS[11];
tmpR2[7] = +tmpObjS[12];
tmpR2[8] = +tmpObjS[13];
tmpR2[9] = +tmpObjS[14];
tmpR2[10] = +tmpObjS[15];
tmpR2[11] = +tmpObjS[16];
tmpR2[12] = +tmpObjS[17];
tmpR2[13] = +tmpObjS[18];
tmpR2[14] = +tmpObjS[19];
tmpR2[15] = +tmpObjS[20];
tmpR2[16] = +tmpObjS[21];
tmpR2[17] = +tmpObjS[22];
tmpR2[18] = +tmpObjS[23];
tmpR2[19] = +tmpObjS[24];
tmpR1[0] = + tmpR2[1];
tmpR1[1] = + tmpR2[2];
tmpR1[2] = + tmpR2[3];
tmpR1[3] = + tmpR2[4];
tmpR1[4] = + tmpR2[6];
tmpR1[5] = + tmpR2[7];
tmpR1[6] = + tmpR2[8];
tmpR1[7] = + tmpR2[9];
tmpR1[8] = + tmpR2[11];
tmpR1[9] = + tmpR2[12];
tmpR1[10] = + tmpR2[13];
tmpR1[11] = + tmpR2[14];
tmpR1[12] = + tmpR2[16];
tmpR1[13] = + tmpR2[17];
tmpR1[14] = + tmpR2[18];
tmpR1[15] = + tmpR2[19];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN1[0] = + tmpQN2[0];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 40; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj];
acadoWorkspace.objValueIn[1] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[2] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[runObj * 4 + 3];
acadoWorkspace.objValueIn[5] = acadoVariables.od[runObj * 3];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 3 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj ]), &(acadoWorkspace.Q2[ runObj * 5 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 16 ]), &(acadoWorkspace.R2[ runObj * 20 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[40];
acadoWorkspace.objValueIn[1] = acadoVariables.od[120];
acadoWorkspace.objValueIn[2] = acadoVariables.od[121];
acadoWorkspace.objValueIn[3] = acadoVariables.od[122];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0];
Gu2[1] = + Gx1[0]*Gu1[1];
Gu2[2] = + Gx1[0]*Gu1[2];
Gu2[3] = + Gx1[0]*Gu1[3];
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
acadoWorkspace.H[(iRow * 640) + (iCol * 4)] = + Gu1[0]*Gu2[0];
acadoWorkspace.H[(iRow * 640) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1];
acadoWorkspace.H[(iRow * 640) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2];
acadoWorkspace.H[(iRow * 640) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3];
acadoWorkspace.H[(iRow * 640 + 160) + (iCol * 4)] = + Gu1[1]*Gu2[0];
acadoWorkspace.H[(iRow * 640 + 160) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1];
acadoWorkspace.H[(iRow * 640 + 160) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2];
acadoWorkspace.H[(iRow * 640 + 160) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3];
acadoWorkspace.H[(iRow * 640 + 320) + (iCol * 4)] = + Gu1[2]*Gu2[0];
acadoWorkspace.H[(iRow * 640 + 320) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1];
acadoWorkspace.H[(iRow * 640 + 320) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2];
acadoWorkspace.H[(iRow * 640 + 320) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3];
acadoWorkspace.H[(iRow * 640 + 480) + (iCol * 4)] = + Gu1[3]*Gu2[0];
acadoWorkspace.H[(iRow * 640 + 480) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1];
acadoWorkspace.H[(iRow * 640 + 480) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2];
acadoWorkspace.H[(iRow * 640 + 480) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 644] = + Gu1[0]*Gu2[0] + R11[0];
acadoWorkspace.H[iRow * 644 + 1] = + Gu1[0]*Gu2[1] + R11[1];
acadoWorkspace.H[iRow * 644 + 2] = + Gu1[0]*Gu2[2] + R11[2];
acadoWorkspace.H[iRow * 644 + 3] = + Gu1[0]*Gu2[3] + R11[3];
acadoWorkspace.H[iRow * 644 + 160] = + Gu1[1]*Gu2[0] + R11[4];
acadoWorkspace.H[iRow * 644 + 161] = + Gu1[1]*Gu2[1] + R11[5];
acadoWorkspace.H[iRow * 644 + 162] = + Gu1[1]*Gu2[2] + R11[6];
acadoWorkspace.H[iRow * 644 + 163] = + Gu1[1]*Gu2[3] + R11[7];
acadoWorkspace.H[iRow * 644 + 320] = + Gu1[2]*Gu2[0] + R11[8];
acadoWorkspace.H[iRow * 644 + 321] = + Gu1[2]*Gu2[1] + R11[9];
acadoWorkspace.H[iRow * 644 + 322] = + Gu1[2]*Gu2[2] + R11[10];
acadoWorkspace.H[iRow * 644 + 323] = + Gu1[2]*Gu2[3] + R11[11];
acadoWorkspace.H[iRow * 644 + 480] = + Gu1[3]*Gu2[0] + R11[12];
acadoWorkspace.H[iRow * 644 + 481] = + Gu1[3]*Gu2[1] + R11[13];
acadoWorkspace.H[iRow * 644 + 482] = + Gu1[3]*Gu2[2] + R11[14];
acadoWorkspace.H[iRow * 644 + 483] = + Gu1[3]*Gu2[3] + R11[15];
acadoWorkspace.H[iRow * 644] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 644 + 161] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 644 + 322] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 644 + 483] += 1.0000000000000000e-04;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0];
Gu2[1] = + Gx1[0]*Gu1[1];
Gu2[2] = + Gx1[0]*Gu1[2];
Gu2[3] = + Gx1[0]*Gu1[3];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Gu2[2];
Gu3[3] = + Q11[0]*Gu1[3] + Gu2[3];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + w12[0];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0];
U1[1] += + Gu1[1]*w11[0];
U1[2] += + Gu1[2]*w11[0];
U1[3] += + Gu1[3]*w11[0];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + w12[0];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 640) + (iCol * 4)] = acadoWorkspace.H[(iCol * 640) + (iRow * 4)];
acadoWorkspace.H[(iRow * 640) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 640 + 160) + (iRow * 4)];
acadoWorkspace.H[(iRow * 640) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 640 + 320) + (iRow * 4)];
acadoWorkspace.H[(iRow * 640) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 640 + 480) + (iRow * 4)];
acadoWorkspace.H[(iRow * 640 + 160) + (iCol * 4)] = acadoWorkspace.H[(iCol * 640) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 640 + 160) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 640 + 160) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 640 + 160) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 640 + 320) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 640 + 160) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 640 + 480) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 640 + 320) + (iCol * 4)] = acadoWorkspace.H[(iCol * 640) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 640 + 320) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 640 + 160) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 640 + 320) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 640 + 320) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 640 + 320) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 640 + 480) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 640 + 480) + (iCol * 4)] = acadoWorkspace.H[(iCol * 640) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 640 + 480) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 640 + 160) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 640 + 480) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 640 + 320) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 640 + 480) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 640 + 480) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4];
RDy1[1] = + R2[5]*Dy1[0] + R2[6]*Dy1[1] + R2[7]*Dy1[2] + R2[8]*Dy1[3] + R2[9]*Dy1[4];
RDy1[2] = + R2[10]*Dy1[0] + R2[11]*Dy1[1] + R2[12]*Dy1[2] + R2[13]*Dy1[3] + R2[14]*Dy1[4];
RDy1[3] = + R2[15]*Dy1[0] + R2[16]*Dy1[1] + R2[17]*Dy1[2] + R2[18]*Dy1[3] + R2[19]*Dy1[4];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 160) + (col * 4)] = + Hx[0]*E[0];
acadoWorkspace.A[(row * 160) + (col * 4 + 1)] = + Hx[0]*E[1];
acadoWorkspace.A[(row * 160) + (col * 4 + 2)] = + Hx[0]*E[2];
acadoWorkspace.A[(row * 160) + (col * 4 + 3)] = + Hx[0]*E[3];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 1 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 1 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 2 ]), &(acadoWorkspace.C[ 1 ]), &(acadoWorkspace.C[ 2 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 3 ]), &(acadoWorkspace.C[ 2 ]), &(acadoWorkspace.C[ 3 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.C[ 3 ]), &(acadoWorkspace.C[ 4 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 5 ]), &(acadoWorkspace.C[ 4 ]), &(acadoWorkspace.C[ 5 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 6 ]), &(acadoWorkspace.C[ 5 ]), &(acadoWorkspace.C[ 6 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 7 ]), &(acadoWorkspace.C[ 6 ]), &(acadoWorkspace.C[ 7 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.C[ 7 ]), &(acadoWorkspace.C[ 8 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.C[ 8 ]), &(acadoWorkspace.C[ 9 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 10 ]), &(acadoWorkspace.C[ 9 ]), &(acadoWorkspace.C[ 10 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 11 ]), &(acadoWorkspace.C[ 10 ]), &(acadoWorkspace.C[ 11 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.C[ 11 ]), &(acadoWorkspace.C[ 12 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 13 ]), &(acadoWorkspace.C[ 12 ]), &(acadoWorkspace.C[ 13 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 14 ]), &(acadoWorkspace.C[ 13 ]), &(acadoWorkspace.C[ 14 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 15 ]), &(acadoWorkspace.C[ 14 ]), &(acadoWorkspace.C[ 15 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.C[ 15 ]), &(acadoWorkspace.C[ 16 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 17 ]), &(acadoWorkspace.C[ 16 ]), &(acadoWorkspace.C[ 17 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.C[ 17 ]), &(acadoWorkspace.C[ 18 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 19 ]), &(acadoWorkspace.C[ 18 ]), &(acadoWorkspace.C[ 19 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.C[ 19 ]), &(acadoWorkspace.C[ 20 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 21 ]), &(acadoWorkspace.C[ 20 ]), &(acadoWorkspace.C[ 21 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 22 ]), &(acadoWorkspace.C[ 21 ]), &(acadoWorkspace.C[ 22 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 23 ]), &(acadoWorkspace.C[ 22 ]), &(acadoWorkspace.C[ 23 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.C[ 23 ]), &(acadoWorkspace.C[ 24 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.C[ 24 ]), &(acadoWorkspace.C[ 25 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 26 ]), &(acadoWorkspace.C[ 25 ]), &(acadoWorkspace.C[ 26 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.C[ 26 ]), &(acadoWorkspace.C[ 27 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.C[ 27 ]), &(acadoWorkspace.C[ 28 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 29 ]), &(acadoWorkspace.C[ 28 ]), &(acadoWorkspace.C[ 29 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 30 ]), &(acadoWorkspace.C[ 29 ]), &(acadoWorkspace.C[ 30 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 31 ]), &(acadoWorkspace.C[ 30 ]), &(acadoWorkspace.C[ 31 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.C[ 31 ]), &(acadoWorkspace.C[ 32 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 33 ]), &(acadoWorkspace.C[ 32 ]), &(acadoWorkspace.C[ 33 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 34 ]), &(acadoWorkspace.C[ 33 ]), &(acadoWorkspace.C[ 34 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 35 ]), &(acadoWorkspace.C[ 34 ]), &(acadoWorkspace.C[ 35 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.C[ 35 ]), &(acadoWorkspace.C[ 36 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 37 ]), &(acadoWorkspace.C[ 36 ]), &(acadoWorkspace.C[ 37 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 38 ]), &(acadoWorkspace.C[ 37 ]), &(acadoWorkspace.C[ 38 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 39 ]), &(acadoWorkspace.C[ 38 ]), &(acadoWorkspace.C[ 39 ]) );
for (lRun2 = 0; lRun2 < 40; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 81)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 4 ]), &(acadoWorkspace.E[ lRun3 * 4 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 40; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (1)) * (1)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (1)) * (4)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (1)) * (4)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (40)) - (1)) * (1)) * (4)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 39; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 4 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (1)) * (4)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 16 ]), &(acadoWorkspace.evGu[ lRun2 * 4 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

acadoWorkspace.sbar[1] = acadoWorkspace.d[0];
acadoWorkspace.sbar[2] = acadoWorkspace.d[1];
acadoWorkspace.sbar[3] = acadoWorkspace.d[2];
acadoWorkspace.sbar[4] = acadoWorkspace.d[3];
acadoWorkspace.sbar[5] = acadoWorkspace.d[4];
acadoWorkspace.sbar[6] = acadoWorkspace.d[5];
acadoWorkspace.sbar[7] = acadoWorkspace.d[6];
acadoWorkspace.sbar[8] = acadoWorkspace.d[7];
acadoWorkspace.sbar[9] = acadoWorkspace.d[8];
acadoWorkspace.sbar[10] = acadoWorkspace.d[9];
acadoWorkspace.sbar[11] = acadoWorkspace.d[10];
acadoWorkspace.sbar[12] = acadoWorkspace.d[11];
acadoWorkspace.sbar[13] = acadoWorkspace.d[12];
acadoWorkspace.sbar[14] = acadoWorkspace.d[13];
acadoWorkspace.sbar[15] = acadoWorkspace.d[14];
acadoWorkspace.sbar[16] = acadoWorkspace.d[15];
acadoWorkspace.sbar[17] = acadoWorkspace.d[16];
acadoWorkspace.sbar[18] = acadoWorkspace.d[17];
acadoWorkspace.sbar[19] = acadoWorkspace.d[18];
acadoWorkspace.sbar[20] = acadoWorkspace.d[19];
acadoWorkspace.sbar[21] = acadoWorkspace.d[20];
acadoWorkspace.sbar[22] = acadoWorkspace.d[21];
acadoWorkspace.sbar[23] = acadoWorkspace.d[22];
acadoWorkspace.sbar[24] = acadoWorkspace.d[23];
acadoWorkspace.sbar[25] = acadoWorkspace.d[24];
acadoWorkspace.sbar[26] = acadoWorkspace.d[25];
acadoWorkspace.sbar[27] = acadoWorkspace.d[26];
acadoWorkspace.sbar[28] = acadoWorkspace.d[27];
acadoWorkspace.sbar[29] = acadoWorkspace.d[28];
acadoWorkspace.sbar[30] = acadoWorkspace.d[29];
acadoWorkspace.sbar[31] = acadoWorkspace.d[30];
acadoWorkspace.sbar[32] = acadoWorkspace.d[31];
acadoWorkspace.sbar[33] = acadoWorkspace.d[32];
acadoWorkspace.sbar[34] = acadoWorkspace.d[33];
acadoWorkspace.sbar[35] = acadoWorkspace.d[34];
acadoWorkspace.sbar[36] = acadoWorkspace.d[35];
acadoWorkspace.sbar[37] = acadoWorkspace.d[36];
acadoWorkspace.sbar[38] = acadoWorkspace.d[37];
acadoWorkspace.sbar[39] = acadoWorkspace.d[38];
acadoWorkspace.sbar[40] = acadoWorkspace.d[39];
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
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[75];
acadoWorkspace.lb[76] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[76];
acadoWorkspace.lb[77] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[77];
acadoWorkspace.lb[78] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[78];
acadoWorkspace.lb[79] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[79];
acadoWorkspace.lb[80] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[81];
acadoWorkspace.lb[82] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[82];
acadoWorkspace.lb[83] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[83];
acadoWorkspace.lb[84] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[86];
acadoWorkspace.lb[87] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[87];
acadoWorkspace.lb[88] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[88];
acadoWorkspace.lb[89] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[89];
acadoWorkspace.lb[90] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[90];
acadoWorkspace.lb[91] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[91];
acadoWorkspace.lb[92] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[92];
acadoWorkspace.lb[93] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[93];
acadoWorkspace.lb[94] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[94];
acadoWorkspace.lb[95] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[95];
acadoWorkspace.lb[96] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[96];
acadoWorkspace.lb[97] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[97];
acadoWorkspace.lb[98] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[98];
acadoWorkspace.lb[99] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[99];
acadoWorkspace.lb[100] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[100];
acadoWorkspace.lb[101] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[101];
acadoWorkspace.lb[102] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[102];
acadoWorkspace.lb[103] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[103];
acadoWorkspace.lb[104] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[104];
acadoWorkspace.lb[105] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[105];
acadoWorkspace.lb[106] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[106];
acadoWorkspace.lb[107] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[107];
acadoWorkspace.lb[108] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[108];
acadoWorkspace.lb[109] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[109];
acadoWorkspace.lb[110] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[110];
acadoWorkspace.lb[111] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[111];
acadoWorkspace.lb[112] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[112];
acadoWorkspace.lb[113] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[113];
acadoWorkspace.lb[114] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[114];
acadoWorkspace.lb[115] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[115];
acadoWorkspace.lb[116] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[116];
acadoWorkspace.lb[117] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[117];
acadoWorkspace.lb[118] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[118];
acadoWorkspace.lb[119] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[119];
acadoWorkspace.lb[120] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[120];
acadoWorkspace.lb[121] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[121];
acadoWorkspace.lb[122] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[122];
acadoWorkspace.lb[123] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[123];
acadoWorkspace.lb[124] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[124];
acadoWorkspace.lb[125] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[125];
acadoWorkspace.lb[126] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[126];
acadoWorkspace.lb[127] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[127];
acadoWorkspace.lb[128] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[128];
acadoWorkspace.lb[129] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[129];
acadoWorkspace.lb[130] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[130];
acadoWorkspace.lb[131] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[131];
acadoWorkspace.lb[132] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[132];
acadoWorkspace.lb[133] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[133];
acadoWorkspace.lb[134] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[134];
acadoWorkspace.lb[135] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[135];
acadoWorkspace.lb[136] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[136];
acadoWorkspace.lb[137] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[137];
acadoWorkspace.lb[138] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[138];
acadoWorkspace.lb[139] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[139];
acadoWorkspace.lb[140] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[140];
acadoWorkspace.lb[141] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[141];
acadoWorkspace.lb[142] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[142];
acadoWorkspace.lb[143] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[143];
acadoWorkspace.lb[144] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[144];
acadoWorkspace.lb[145] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[145];
acadoWorkspace.lb[146] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[146];
acadoWorkspace.lb[147] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[147];
acadoWorkspace.lb[148] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[148];
acadoWorkspace.lb[149] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[149];
acadoWorkspace.lb[150] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[150];
acadoWorkspace.lb[151] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[151];
acadoWorkspace.lb[152] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[152];
acadoWorkspace.lb[153] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[153];
acadoWorkspace.lb[154] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[154];
acadoWorkspace.lb[155] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[155];
acadoWorkspace.lb[156] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[156];
acadoWorkspace.lb[157] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[157];
acadoWorkspace.lb[158] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[158];
acadoWorkspace.lb[159] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[159];
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
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+12 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+12 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+12 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+12 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+12 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+12 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+12 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)1.0000000000000000e+12 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.0000000000000000e+12 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+12 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)1.0000000000000000e+12 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e+12 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.0000000000000000e+12 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)1.0000000000000000e+12 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)1.0000000000000000e+12 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e+12 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.0000000000000000e+12 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)1.0000000000000000e+12 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)1.0000000000000000e+12 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e+12 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)1.0000000000000000e+12 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)1.0000000000000000e+12 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)1.0000000000000000e+12 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)1.0000000000000000e+12 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)1.0000000000000000e+12 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)1.0000000000000000e+12 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)1.0000000000000000e+12 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)1.0000000000000000e+12 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)1.0000000000000000e+12 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)1.0000000000000000e+12 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)1.0000000000000000e+12 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)1.0000000000000000e+12 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)1.0000000000000000e+12 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)1.0000000000000000e+12 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)1.0000000000000000e+12 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)1.0000000000000000e+12 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)1.0000000000000000e+12 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)1.0000000000000000e+12 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)1.0000000000000000e+12 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)1.0000000000000000e+12 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)1.0000000000000000e+12 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)1.0000000000000000e+12 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)1.0000000000000000e+12 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)1.0000000000000000e+12 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)1.0000000000000000e+12 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)1.0000000000000000e+12 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)1.0000000000000000e+12 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)1.0000000000000000e+12 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)1.0000000000000000e+12 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)1.0000000000000000e+12 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)1.0000000000000000e+12 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)1.0000000000000000e+12 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)1.0000000000000000e+12 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)1.0000000000000000e+12 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)1.0000000000000000e+12 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)1.0000000000000000e+12 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)1.0000000000000000e+12 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)1.0000000000000000e+12 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)1.0000000000000000e+12 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)1.0000000000000000e+12 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)1.0000000000000000e+12 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)1.0000000000000000e+12 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)1.0000000000000000e+12 - acadoVariables.u[89];
acadoWorkspace.ub[90] = (real_t)1.0000000000000000e+12 - acadoVariables.u[90];
acadoWorkspace.ub[91] = (real_t)1.0000000000000000e+12 - acadoVariables.u[91];
acadoWorkspace.ub[92] = (real_t)1.0000000000000000e+12 - acadoVariables.u[92];
acadoWorkspace.ub[93] = (real_t)1.0000000000000000e+12 - acadoVariables.u[93];
acadoWorkspace.ub[94] = (real_t)1.0000000000000000e+12 - acadoVariables.u[94];
acadoWorkspace.ub[95] = (real_t)1.0000000000000000e+12 - acadoVariables.u[95];
acadoWorkspace.ub[96] = (real_t)1.0000000000000000e+12 - acadoVariables.u[96];
acadoWorkspace.ub[97] = (real_t)1.0000000000000000e+12 - acadoVariables.u[97];
acadoWorkspace.ub[98] = (real_t)1.0000000000000000e+12 - acadoVariables.u[98];
acadoWorkspace.ub[99] = (real_t)1.0000000000000000e+12 - acadoVariables.u[99];
acadoWorkspace.ub[100] = (real_t)1.0000000000000000e+12 - acadoVariables.u[100];
acadoWorkspace.ub[101] = (real_t)1.0000000000000000e+12 - acadoVariables.u[101];
acadoWorkspace.ub[102] = (real_t)1.0000000000000000e+12 - acadoVariables.u[102];
acadoWorkspace.ub[103] = (real_t)1.0000000000000000e+12 - acadoVariables.u[103];
acadoWorkspace.ub[104] = (real_t)1.0000000000000000e+12 - acadoVariables.u[104];
acadoWorkspace.ub[105] = (real_t)1.0000000000000000e+12 - acadoVariables.u[105];
acadoWorkspace.ub[106] = (real_t)1.0000000000000000e+12 - acadoVariables.u[106];
acadoWorkspace.ub[107] = (real_t)1.0000000000000000e+12 - acadoVariables.u[107];
acadoWorkspace.ub[108] = (real_t)1.0000000000000000e+12 - acadoVariables.u[108];
acadoWorkspace.ub[109] = (real_t)1.0000000000000000e+12 - acadoVariables.u[109];
acadoWorkspace.ub[110] = (real_t)1.0000000000000000e+12 - acadoVariables.u[110];
acadoWorkspace.ub[111] = (real_t)1.0000000000000000e+12 - acadoVariables.u[111];
acadoWorkspace.ub[112] = (real_t)1.0000000000000000e+12 - acadoVariables.u[112];
acadoWorkspace.ub[113] = (real_t)1.0000000000000000e+12 - acadoVariables.u[113];
acadoWorkspace.ub[114] = (real_t)1.0000000000000000e+12 - acadoVariables.u[114];
acadoWorkspace.ub[115] = (real_t)1.0000000000000000e+12 - acadoVariables.u[115];
acadoWorkspace.ub[116] = (real_t)1.0000000000000000e+12 - acadoVariables.u[116];
acadoWorkspace.ub[117] = (real_t)1.0000000000000000e+12 - acadoVariables.u[117];
acadoWorkspace.ub[118] = (real_t)1.0000000000000000e+12 - acadoVariables.u[118];
acadoWorkspace.ub[119] = (real_t)1.0000000000000000e+12 - acadoVariables.u[119];
acadoWorkspace.ub[120] = (real_t)1.0000000000000000e+12 - acadoVariables.u[120];
acadoWorkspace.ub[121] = (real_t)1.0000000000000000e+12 - acadoVariables.u[121];
acadoWorkspace.ub[122] = (real_t)1.0000000000000000e+12 - acadoVariables.u[122];
acadoWorkspace.ub[123] = (real_t)1.0000000000000000e+12 - acadoVariables.u[123];
acadoWorkspace.ub[124] = (real_t)1.0000000000000000e+12 - acadoVariables.u[124];
acadoWorkspace.ub[125] = (real_t)1.0000000000000000e+12 - acadoVariables.u[125];
acadoWorkspace.ub[126] = (real_t)1.0000000000000000e+12 - acadoVariables.u[126];
acadoWorkspace.ub[127] = (real_t)1.0000000000000000e+12 - acadoVariables.u[127];
acadoWorkspace.ub[128] = (real_t)1.0000000000000000e+12 - acadoVariables.u[128];
acadoWorkspace.ub[129] = (real_t)1.0000000000000000e+12 - acadoVariables.u[129];
acadoWorkspace.ub[130] = (real_t)1.0000000000000000e+12 - acadoVariables.u[130];
acadoWorkspace.ub[131] = (real_t)1.0000000000000000e+12 - acadoVariables.u[131];
acadoWorkspace.ub[132] = (real_t)1.0000000000000000e+12 - acadoVariables.u[132];
acadoWorkspace.ub[133] = (real_t)1.0000000000000000e+12 - acadoVariables.u[133];
acadoWorkspace.ub[134] = (real_t)1.0000000000000000e+12 - acadoVariables.u[134];
acadoWorkspace.ub[135] = (real_t)1.0000000000000000e+12 - acadoVariables.u[135];
acadoWorkspace.ub[136] = (real_t)1.0000000000000000e+12 - acadoVariables.u[136];
acadoWorkspace.ub[137] = (real_t)1.0000000000000000e+12 - acadoVariables.u[137];
acadoWorkspace.ub[138] = (real_t)1.0000000000000000e+12 - acadoVariables.u[138];
acadoWorkspace.ub[139] = (real_t)1.0000000000000000e+12 - acadoVariables.u[139];
acadoWorkspace.ub[140] = (real_t)1.0000000000000000e+12 - acadoVariables.u[140];
acadoWorkspace.ub[141] = (real_t)1.0000000000000000e+12 - acadoVariables.u[141];
acadoWorkspace.ub[142] = (real_t)1.0000000000000000e+12 - acadoVariables.u[142];
acadoWorkspace.ub[143] = (real_t)1.0000000000000000e+12 - acadoVariables.u[143];
acadoWorkspace.ub[144] = (real_t)1.0000000000000000e+12 - acadoVariables.u[144];
acadoWorkspace.ub[145] = (real_t)1.0000000000000000e+12 - acadoVariables.u[145];
acadoWorkspace.ub[146] = (real_t)1.0000000000000000e+12 - acadoVariables.u[146];
acadoWorkspace.ub[147] = (real_t)1.0000000000000000e+12 - acadoVariables.u[147];
acadoWorkspace.ub[148] = (real_t)1.0000000000000000e+12 - acadoVariables.u[148];
acadoWorkspace.ub[149] = (real_t)1.0000000000000000e+12 - acadoVariables.u[149];
acadoWorkspace.ub[150] = (real_t)1.0000000000000000e+12 - acadoVariables.u[150];
acadoWorkspace.ub[151] = (real_t)1.0000000000000000e+12 - acadoVariables.u[151];
acadoWorkspace.ub[152] = (real_t)1.0000000000000000e+12 - acadoVariables.u[152];
acadoWorkspace.ub[153] = (real_t)1.0000000000000000e+12 - acadoVariables.u[153];
acadoWorkspace.ub[154] = (real_t)1.0000000000000000e+12 - acadoVariables.u[154];
acadoWorkspace.ub[155] = (real_t)1.0000000000000000e+12 - acadoVariables.u[155];
acadoWorkspace.ub[156] = (real_t)1.0000000000000000e+12 - acadoVariables.u[156];
acadoWorkspace.ub[157] = (real_t)1.0000000000000000e+12 - acadoVariables.u[157];
acadoWorkspace.ub[158] = (real_t)1.0000000000000000e+12 - acadoVariables.u[158];
acadoWorkspace.ub[159] = (real_t)1.0000000000000000e+12 - acadoVariables.u[159];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1];
acadoWorkspace.conValueIn[1] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.conValueIn[2] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[3] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[4] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.conValueIn[5] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 3 + 2];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[5];
}



for (lRun2 = 0; lRun2 < 39; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun3) * (lRun3 * -1 + 79)) / (2)) + (lRun2);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 + 1 ]), &(acadoWorkspace.E[ lRun4 * 4 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[3] = acadoWorkspace.evHu[3];
acadoWorkspace.A[164] = acadoWorkspace.evHu[4];
acadoWorkspace.A[165] = acadoWorkspace.evHu[5];
acadoWorkspace.A[166] = acadoWorkspace.evHu[6];
acadoWorkspace.A[167] = acadoWorkspace.evHu[7];
acadoWorkspace.A[328] = acadoWorkspace.evHu[8];
acadoWorkspace.A[329] = acadoWorkspace.evHu[9];
acadoWorkspace.A[330] = acadoWorkspace.evHu[10];
acadoWorkspace.A[331] = acadoWorkspace.evHu[11];
acadoWorkspace.A[492] = acadoWorkspace.evHu[12];
acadoWorkspace.A[493] = acadoWorkspace.evHu[13];
acadoWorkspace.A[494] = acadoWorkspace.evHu[14];
acadoWorkspace.A[495] = acadoWorkspace.evHu[15];
acadoWorkspace.A[656] = acadoWorkspace.evHu[16];
acadoWorkspace.A[657] = acadoWorkspace.evHu[17];
acadoWorkspace.A[658] = acadoWorkspace.evHu[18];
acadoWorkspace.A[659] = acadoWorkspace.evHu[19];
acadoWorkspace.A[820] = acadoWorkspace.evHu[20];
acadoWorkspace.A[821] = acadoWorkspace.evHu[21];
acadoWorkspace.A[822] = acadoWorkspace.evHu[22];
acadoWorkspace.A[823] = acadoWorkspace.evHu[23];
acadoWorkspace.A[984] = acadoWorkspace.evHu[24];
acadoWorkspace.A[985] = acadoWorkspace.evHu[25];
acadoWorkspace.A[986] = acadoWorkspace.evHu[26];
acadoWorkspace.A[987] = acadoWorkspace.evHu[27];
acadoWorkspace.A[1148] = acadoWorkspace.evHu[28];
acadoWorkspace.A[1149] = acadoWorkspace.evHu[29];
acadoWorkspace.A[1150] = acadoWorkspace.evHu[30];
acadoWorkspace.A[1151] = acadoWorkspace.evHu[31];
acadoWorkspace.A[1312] = acadoWorkspace.evHu[32];
acadoWorkspace.A[1313] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1314] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1315] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1476] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1477] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1478] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1479] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1640] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1641] = acadoWorkspace.evHu[41];
acadoWorkspace.A[1642] = acadoWorkspace.evHu[42];
acadoWorkspace.A[1643] = acadoWorkspace.evHu[43];
acadoWorkspace.A[1804] = acadoWorkspace.evHu[44];
acadoWorkspace.A[1805] = acadoWorkspace.evHu[45];
acadoWorkspace.A[1806] = acadoWorkspace.evHu[46];
acadoWorkspace.A[1807] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1968] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1969] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1970] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1971] = acadoWorkspace.evHu[51];
acadoWorkspace.A[2132] = acadoWorkspace.evHu[52];
acadoWorkspace.A[2133] = acadoWorkspace.evHu[53];
acadoWorkspace.A[2134] = acadoWorkspace.evHu[54];
acadoWorkspace.A[2135] = acadoWorkspace.evHu[55];
acadoWorkspace.A[2296] = acadoWorkspace.evHu[56];
acadoWorkspace.A[2297] = acadoWorkspace.evHu[57];
acadoWorkspace.A[2298] = acadoWorkspace.evHu[58];
acadoWorkspace.A[2299] = acadoWorkspace.evHu[59];
acadoWorkspace.A[2460] = acadoWorkspace.evHu[60];
acadoWorkspace.A[2461] = acadoWorkspace.evHu[61];
acadoWorkspace.A[2462] = acadoWorkspace.evHu[62];
acadoWorkspace.A[2463] = acadoWorkspace.evHu[63];
acadoWorkspace.A[2624] = acadoWorkspace.evHu[64];
acadoWorkspace.A[2625] = acadoWorkspace.evHu[65];
acadoWorkspace.A[2626] = acadoWorkspace.evHu[66];
acadoWorkspace.A[2627] = acadoWorkspace.evHu[67];
acadoWorkspace.A[2788] = acadoWorkspace.evHu[68];
acadoWorkspace.A[2789] = acadoWorkspace.evHu[69];
acadoWorkspace.A[2790] = acadoWorkspace.evHu[70];
acadoWorkspace.A[2791] = acadoWorkspace.evHu[71];
acadoWorkspace.A[2952] = acadoWorkspace.evHu[72];
acadoWorkspace.A[2953] = acadoWorkspace.evHu[73];
acadoWorkspace.A[2954] = acadoWorkspace.evHu[74];
acadoWorkspace.A[2955] = acadoWorkspace.evHu[75];
acadoWorkspace.A[3116] = acadoWorkspace.evHu[76];
acadoWorkspace.A[3117] = acadoWorkspace.evHu[77];
acadoWorkspace.A[3118] = acadoWorkspace.evHu[78];
acadoWorkspace.A[3119] = acadoWorkspace.evHu[79];
acadoWorkspace.A[3280] = acadoWorkspace.evHu[80];
acadoWorkspace.A[3281] = acadoWorkspace.evHu[81];
acadoWorkspace.A[3282] = acadoWorkspace.evHu[82];
acadoWorkspace.A[3283] = acadoWorkspace.evHu[83];
acadoWorkspace.A[3444] = acadoWorkspace.evHu[84];
acadoWorkspace.A[3445] = acadoWorkspace.evHu[85];
acadoWorkspace.A[3446] = acadoWorkspace.evHu[86];
acadoWorkspace.A[3447] = acadoWorkspace.evHu[87];
acadoWorkspace.A[3608] = acadoWorkspace.evHu[88];
acadoWorkspace.A[3609] = acadoWorkspace.evHu[89];
acadoWorkspace.A[3610] = acadoWorkspace.evHu[90];
acadoWorkspace.A[3611] = acadoWorkspace.evHu[91];
acadoWorkspace.A[3772] = acadoWorkspace.evHu[92];
acadoWorkspace.A[3773] = acadoWorkspace.evHu[93];
acadoWorkspace.A[3774] = acadoWorkspace.evHu[94];
acadoWorkspace.A[3775] = acadoWorkspace.evHu[95];
acadoWorkspace.A[3936] = acadoWorkspace.evHu[96];
acadoWorkspace.A[3937] = acadoWorkspace.evHu[97];
acadoWorkspace.A[3938] = acadoWorkspace.evHu[98];
acadoWorkspace.A[3939] = acadoWorkspace.evHu[99];
acadoWorkspace.A[4100] = acadoWorkspace.evHu[100];
acadoWorkspace.A[4101] = acadoWorkspace.evHu[101];
acadoWorkspace.A[4102] = acadoWorkspace.evHu[102];
acadoWorkspace.A[4103] = acadoWorkspace.evHu[103];
acadoWorkspace.A[4264] = acadoWorkspace.evHu[104];
acadoWorkspace.A[4265] = acadoWorkspace.evHu[105];
acadoWorkspace.A[4266] = acadoWorkspace.evHu[106];
acadoWorkspace.A[4267] = acadoWorkspace.evHu[107];
acadoWorkspace.A[4428] = acadoWorkspace.evHu[108];
acadoWorkspace.A[4429] = acadoWorkspace.evHu[109];
acadoWorkspace.A[4430] = acadoWorkspace.evHu[110];
acadoWorkspace.A[4431] = acadoWorkspace.evHu[111];
acadoWorkspace.A[4592] = acadoWorkspace.evHu[112];
acadoWorkspace.A[4593] = acadoWorkspace.evHu[113];
acadoWorkspace.A[4594] = acadoWorkspace.evHu[114];
acadoWorkspace.A[4595] = acadoWorkspace.evHu[115];
acadoWorkspace.A[4756] = acadoWorkspace.evHu[116];
acadoWorkspace.A[4757] = acadoWorkspace.evHu[117];
acadoWorkspace.A[4758] = acadoWorkspace.evHu[118];
acadoWorkspace.A[4759] = acadoWorkspace.evHu[119];
acadoWorkspace.A[4920] = acadoWorkspace.evHu[120];
acadoWorkspace.A[4921] = acadoWorkspace.evHu[121];
acadoWorkspace.A[4922] = acadoWorkspace.evHu[122];
acadoWorkspace.A[4923] = acadoWorkspace.evHu[123];
acadoWorkspace.A[5084] = acadoWorkspace.evHu[124];
acadoWorkspace.A[5085] = acadoWorkspace.evHu[125];
acadoWorkspace.A[5086] = acadoWorkspace.evHu[126];
acadoWorkspace.A[5087] = acadoWorkspace.evHu[127];
acadoWorkspace.A[5248] = acadoWorkspace.evHu[128];
acadoWorkspace.A[5249] = acadoWorkspace.evHu[129];
acadoWorkspace.A[5250] = acadoWorkspace.evHu[130];
acadoWorkspace.A[5251] = acadoWorkspace.evHu[131];
acadoWorkspace.A[5412] = acadoWorkspace.evHu[132];
acadoWorkspace.A[5413] = acadoWorkspace.evHu[133];
acadoWorkspace.A[5414] = acadoWorkspace.evHu[134];
acadoWorkspace.A[5415] = acadoWorkspace.evHu[135];
acadoWorkspace.A[5576] = acadoWorkspace.evHu[136];
acadoWorkspace.A[5577] = acadoWorkspace.evHu[137];
acadoWorkspace.A[5578] = acadoWorkspace.evHu[138];
acadoWorkspace.A[5579] = acadoWorkspace.evHu[139];
acadoWorkspace.A[5740] = acadoWorkspace.evHu[140];
acadoWorkspace.A[5741] = acadoWorkspace.evHu[141];
acadoWorkspace.A[5742] = acadoWorkspace.evHu[142];
acadoWorkspace.A[5743] = acadoWorkspace.evHu[143];
acadoWorkspace.A[5904] = acadoWorkspace.evHu[144];
acadoWorkspace.A[5905] = acadoWorkspace.evHu[145];
acadoWorkspace.A[5906] = acadoWorkspace.evHu[146];
acadoWorkspace.A[5907] = acadoWorkspace.evHu[147];
acadoWorkspace.A[6068] = acadoWorkspace.evHu[148];
acadoWorkspace.A[6069] = acadoWorkspace.evHu[149];
acadoWorkspace.A[6070] = acadoWorkspace.evHu[150];
acadoWorkspace.A[6071] = acadoWorkspace.evHu[151];
acadoWorkspace.A[6232] = acadoWorkspace.evHu[152];
acadoWorkspace.A[6233] = acadoWorkspace.evHu[153];
acadoWorkspace.A[6234] = acadoWorkspace.evHu[154];
acadoWorkspace.A[6235] = acadoWorkspace.evHu[155];
acadoWorkspace.A[6396] = acadoWorkspace.evHu[156];
acadoWorkspace.A[6397] = acadoWorkspace.evHu[157];
acadoWorkspace.A[6398] = acadoWorkspace.evHu[158];
acadoWorkspace.A[6399] = acadoWorkspace.evHu[159];
acadoWorkspace.lbA[0] = - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = - acadoWorkspace.evH[39];

acadoWorkspace.ubA[0] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = (real_t)4.7222222222222221e+01 - acadoWorkspace.evH[39];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
for (lRun1 = 0; lRun1 < 200; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 20 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 100 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 120 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 140 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 160 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 200 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 220 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 260 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 280 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 300 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 320 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 340 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 380 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 76 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 400 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 80 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 420 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 440 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.g[ 88 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 460 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.g[ 92 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 500 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.g[ 100 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 520 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.g[ 104 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 540 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 560 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 112 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 580 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.g[ 116 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 600 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.g[ 120 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 620 ]), &(acadoWorkspace.Dy[ 155 ]), &(acadoWorkspace.g[ 124 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 640 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.g[ 128 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 660 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.g[ 132 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 680 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.g[ 136 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 700 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.g[ 140 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 720 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 144 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 740 ]), &(acadoWorkspace.Dy[ 185 ]), &(acadoWorkspace.g[ 148 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 760 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.g[ 152 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 780 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.g[ 156 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 5 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 1 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 10 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 2 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 15 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 20 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 25 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 30 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 35 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 7 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 40 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 45 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 50 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 55 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 11 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 65 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 13 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 70 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 14 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 75 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 80 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 85 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 17 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 90 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 95 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 19 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 100 ]), &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 105 ]), &(acadoWorkspace.Dy[ 105 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 110 ]), &(acadoWorkspace.Dy[ 110 ]), &(acadoWorkspace.QDy[ 22 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 115 ]), &(acadoWorkspace.Dy[ 115 ]), &(acadoWorkspace.QDy[ 23 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 125 ]), &(acadoWorkspace.Dy[ 125 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 130 ]), &(acadoWorkspace.Dy[ 130 ]), &(acadoWorkspace.QDy[ 26 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 135 ]), &(acadoWorkspace.Dy[ 135 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 140 ]), &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 145 ]), &(acadoWorkspace.Dy[ 145 ]), &(acadoWorkspace.QDy[ 29 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 150 ]), &(acadoWorkspace.Dy[ 150 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 155 ]), &(acadoWorkspace.Dy[ 155 ]), &(acadoWorkspace.QDy[ 31 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 160 ]), &(acadoWorkspace.Dy[ 160 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 165 ]), &(acadoWorkspace.Dy[ 165 ]), &(acadoWorkspace.QDy[ 33 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 170 ]), &(acadoWorkspace.Dy[ 170 ]), &(acadoWorkspace.QDy[ 34 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 175 ]), &(acadoWorkspace.Dy[ 175 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 185 ]), &(acadoWorkspace.Dy[ 185 ]), &(acadoWorkspace.QDy[ 37 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 190 ]), &(acadoWorkspace.Dy[ 190 ]), &(acadoWorkspace.QDy[ 38 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 195 ]), &(acadoWorkspace.Dy[ 195 ]), &(acadoWorkspace.QDy[ 39 ]) );

acadoWorkspace.QDy[40] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 1 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1 ]), &(acadoWorkspace.sbar[ 1 ]), &(acadoWorkspace.sbar[ 2 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 3 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 5 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 6 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 7 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 7 ]), &(acadoWorkspace.sbar[ 7 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 10 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 11 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 11 ]), &(acadoWorkspace.sbar[ 11 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 13 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 13 ]), &(acadoWorkspace.sbar[ 13 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 14 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 15 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 17 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 17 ]), &(acadoWorkspace.sbar[ 17 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 19 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 19 ]), &(acadoWorkspace.sbar[ 19 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 21 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 22 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 22 ]), &(acadoWorkspace.sbar[ 22 ]), &(acadoWorkspace.sbar[ 23 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 23 ]), &(acadoWorkspace.sbar[ 23 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 25 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 26 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 29 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 29 ]), &(acadoWorkspace.sbar[ 29 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 31 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 31 ]), &(acadoWorkspace.sbar[ 31 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 33 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 34 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 34 ]), &(acadoWorkspace.sbar[ 34 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 35 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 37 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 37 ]), &(acadoWorkspace.sbar[ 37 ]), &(acadoWorkspace.sbar[ 38 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 38 ]), &(acadoWorkspace.sbar[ 38 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 39 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 40 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[40] + acadoWorkspace.QDy[40];
acado_macBTw1( &(acadoWorkspace.evGu[ 156 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 156 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 39 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 39 ]), &(acadoWorkspace.sbar[ 39 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 152 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 152 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 38 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 38 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 38 ]), &(acadoWorkspace.sbar[ 38 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 148 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 148 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 37 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 37 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 37 ]), &(acadoWorkspace.sbar[ 37 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 144 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 140 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 140 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 35 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 35 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 35 ]), &(acadoWorkspace.sbar[ 35 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 136 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 136 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 34 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 34 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 34 ]), &(acadoWorkspace.sbar[ 34 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 132 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 132 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 33 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 33 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 33 ]), &(acadoWorkspace.sbar[ 33 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 128 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 124 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 124 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 31 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 31 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 31 ]), &(acadoWorkspace.sbar[ 31 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 120 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 120 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 116 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 116 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 29 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 29 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 29 ]), &(acadoWorkspace.sbar[ 29 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 112 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 112 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.sbar[ 28 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 108 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 104 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 104 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 26 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 26 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 26 ]), &(acadoWorkspace.sbar[ 26 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 100 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 100 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 25 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 25 ]), &(acadoWorkspace.sbar[ 25 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 92 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 92 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 23 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 23 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 23 ]), &(acadoWorkspace.sbar[ 23 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 88 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 88 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 22 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 22 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 22 ]), &(acadoWorkspace.sbar[ 22 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 84 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 21 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 21 ]), &(acadoWorkspace.sbar[ 21 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 80 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 20 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 76 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 19 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 19 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 19 ]), &(acadoWorkspace.sbar[ 19 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 72 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 68 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 17 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 17 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 17 ]), &(acadoWorkspace.sbar[ 17 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 15 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 15 ]), &(acadoWorkspace.sbar[ 15 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 56 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 14 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 14 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 14 ]), &(acadoWorkspace.sbar[ 14 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 52 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 13 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 13 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 13 ]), &(acadoWorkspace.sbar[ 13 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 44 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 11 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 11 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 11 ]), &(acadoWorkspace.sbar[ 11 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 10 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 10 ]), &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 9 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 8 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 7 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 7 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 7 ]), &(acadoWorkspace.sbar[ 7 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 6 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 5 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 5 ]), &(acadoWorkspace.sbar[ 5 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4 ]), &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3 ]), &(acadoWorkspace.sbar[ 3 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 2 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2 ]), &(acadoWorkspace.sbar[ 2 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 1 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1 ]), &(acadoWorkspace.sbar[ 1 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );



acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 1 ]), &(acadoWorkspace.sbar[ 1 ]), &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 2 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 3 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 4 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 5 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.lbA[ 5 ]), &(acadoWorkspace.ubA[ 5 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 6 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 7 ]), &(acadoWorkspace.sbar[ 7 ]), &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 8 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 9 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 10 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 11 ]), &(acadoWorkspace.sbar[ 11 ]), &(acadoWorkspace.lbA[ 11 ]), &(acadoWorkspace.ubA[ 11 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 13 ]), &(acadoWorkspace.sbar[ 13 ]), &(acadoWorkspace.lbA[ 13 ]), &(acadoWorkspace.ubA[ 13 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 14 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 15 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 16 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 17 ]), &(acadoWorkspace.sbar[ 17 ]), &(acadoWorkspace.lbA[ 17 ]), &(acadoWorkspace.ubA[ 17 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 18 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 19 ]), &(acadoWorkspace.sbar[ 19 ]), &(acadoWorkspace.lbA[ 19 ]), &(acadoWorkspace.ubA[ 19 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 20 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 21 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 22 ]), &(acadoWorkspace.sbar[ 22 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 23 ]), &(acadoWorkspace.sbar[ 23 ]), &(acadoWorkspace.lbA[ 23 ]), &(acadoWorkspace.ubA[ 23 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 25 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.lbA[ 25 ]), &(acadoWorkspace.ubA[ 25 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 26 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 27 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.lbA[ 27 ]), &(acadoWorkspace.ubA[ 27 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 28 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 29 ]), &(acadoWorkspace.sbar[ 29 ]), &(acadoWorkspace.lbA[ 29 ]), &(acadoWorkspace.ubA[ 29 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 31 ]), &(acadoWorkspace.sbar[ 31 ]), &(acadoWorkspace.lbA[ 31 ]), &(acadoWorkspace.ubA[ 31 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 32 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 33 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.lbA[ 33 ]), &(acadoWorkspace.ubA[ 33 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 34 ]), &(acadoWorkspace.sbar[ 34 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 35 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.lbA[ 35 ]), &(acadoWorkspace.ubA[ 35 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 37 ]), &(acadoWorkspace.sbar[ 37 ]), &(acadoWorkspace.lbA[ 37 ]), &(acadoWorkspace.ubA[ 37 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 38 ]), &(acadoWorkspace.sbar[ 38 ]), &(acadoWorkspace.lbA[ 38 ]), &(acadoWorkspace.ubA[ 38 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 39 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.lbA[ 39 ]), &(acadoWorkspace.ubA[ 39 ]) );

}

void acado_expand(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoVariables.u[lRun1] += acadoWorkspace.x[lRun1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.d[0];
acadoWorkspace.sbar[2] = acadoWorkspace.d[1];
acadoWorkspace.sbar[3] = acadoWorkspace.d[2];
acadoWorkspace.sbar[4] = acadoWorkspace.d[3];
acadoWorkspace.sbar[5] = acadoWorkspace.d[4];
acadoWorkspace.sbar[6] = acadoWorkspace.d[5];
acadoWorkspace.sbar[7] = acadoWorkspace.d[6];
acadoWorkspace.sbar[8] = acadoWorkspace.d[7];
acadoWorkspace.sbar[9] = acadoWorkspace.d[8];
acadoWorkspace.sbar[10] = acadoWorkspace.d[9];
acadoWorkspace.sbar[11] = acadoWorkspace.d[10];
acadoWorkspace.sbar[12] = acadoWorkspace.d[11];
acadoWorkspace.sbar[13] = acadoWorkspace.d[12];
acadoWorkspace.sbar[14] = acadoWorkspace.d[13];
acadoWorkspace.sbar[15] = acadoWorkspace.d[14];
acadoWorkspace.sbar[16] = acadoWorkspace.d[15];
acadoWorkspace.sbar[17] = acadoWorkspace.d[16];
acadoWorkspace.sbar[18] = acadoWorkspace.d[17];
acadoWorkspace.sbar[19] = acadoWorkspace.d[18];
acadoWorkspace.sbar[20] = acadoWorkspace.d[19];
acadoWorkspace.sbar[21] = acadoWorkspace.d[20];
acadoWorkspace.sbar[22] = acadoWorkspace.d[21];
acadoWorkspace.sbar[23] = acadoWorkspace.d[22];
acadoWorkspace.sbar[24] = acadoWorkspace.d[23];
acadoWorkspace.sbar[25] = acadoWorkspace.d[24];
acadoWorkspace.sbar[26] = acadoWorkspace.d[25];
acadoWorkspace.sbar[27] = acadoWorkspace.d[26];
acadoWorkspace.sbar[28] = acadoWorkspace.d[27];
acadoWorkspace.sbar[29] = acadoWorkspace.d[28];
acadoWorkspace.sbar[30] = acadoWorkspace.d[29];
acadoWorkspace.sbar[31] = acadoWorkspace.d[30];
acadoWorkspace.sbar[32] = acadoWorkspace.d[31];
acadoWorkspace.sbar[33] = acadoWorkspace.d[32];
acadoWorkspace.sbar[34] = acadoWorkspace.d[33];
acadoWorkspace.sbar[35] = acadoWorkspace.d[34];
acadoWorkspace.sbar[36] = acadoWorkspace.d[35];
acadoWorkspace.sbar[37] = acadoWorkspace.d[36];
acadoWorkspace.sbar[38] = acadoWorkspace.d[37];
acadoWorkspace.sbar[39] = acadoWorkspace.d[38];
acadoWorkspace.sbar[40] = acadoWorkspace.d[39];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 1 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1 ]), &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 1 ]), &(acadoWorkspace.sbar[ 2 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 3 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 5 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5 ]), &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 6 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 7 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 7 ]), &(acadoWorkspace.evGu[ 28 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 7 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 10 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 11 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 11 ]), &(acadoWorkspace.evGu[ 44 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 11 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 13 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 13 ]), &(acadoWorkspace.evGu[ 52 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 13 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 14 ]), &(acadoWorkspace.evGu[ 56 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 15 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 17 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 17 ]), &(acadoWorkspace.evGu[ 68 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 17 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGu[ 72 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 19 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 19 ]), &(acadoWorkspace.evGu[ 76 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 19 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 80 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 21 ]), &(acadoWorkspace.evGu[ 84 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 22 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 22 ]), &(acadoWorkspace.evGu[ 88 ]), &(acadoWorkspace.x[ 88 ]), &(acadoWorkspace.sbar[ 22 ]), &(acadoWorkspace.sbar[ 23 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 23 ]), &(acadoWorkspace.evGu[ 92 ]), &(acadoWorkspace.x[ 92 ]), &(acadoWorkspace.sbar[ 23 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 25 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.evGu[ 100 ]), &(acadoWorkspace.x[ 100 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 26 ]), &(acadoWorkspace.evGu[ 104 ]), &(acadoWorkspace.x[ 104 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.evGu[ 112 ]), &(acadoWorkspace.x[ 112 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 29 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 29 ]), &(acadoWorkspace.evGu[ 116 ]), &(acadoWorkspace.x[ 116 ]), &(acadoWorkspace.sbar[ 29 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 30 ]), &(acadoWorkspace.evGu[ 120 ]), &(acadoWorkspace.x[ 120 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 31 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 31 ]), &(acadoWorkspace.evGu[ 124 ]), &(acadoWorkspace.x[ 124 ]), &(acadoWorkspace.sbar[ 31 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.x[ 128 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 33 ]), &(acadoWorkspace.evGu[ 132 ]), &(acadoWorkspace.x[ 132 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 34 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 34 ]), &(acadoWorkspace.evGu[ 136 ]), &(acadoWorkspace.x[ 136 ]), &(acadoWorkspace.sbar[ 34 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 35 ]), &(acadoWorkspace.evGu[ 140 ]), &(acadoWorkspace.x[ 140 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 144 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 37 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 37 ]), &(acadoWorkspace.evGu[ 148 ]), &(acadoWorkspace.x[ 148 ]), &(acadoWorkspace.sbar[ 37 ]), &(acadoWorkspace.sbar[ 38 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 38 ]), &(acadoWorkspace.evGu[ 152 ]), &(acadoWorkspace.x[ 152 ]), &(acadoWorkspace.sbar[ 38 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 39 ]), &(acadoWorkspace.evGu[ 156 ]), &(acadoWorkspace.x[ 156 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 40 ]) );
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
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
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
acadoWorkspace.state[0] = acadoVariables.x[index];
acadoWorkspace.state[6] = acadoVariables.u[index * 4];
acadoWorkspace.state[7] = acadoVariables.u[index * 4 + 1];
acadoWorkspace.state[8] = acadoVariables.u[index * 4 + 2];
acadoWorkspace.state[9] = acadoVariables.u[index * 4 + 3];
acadoWorkspace.state[10] = acadoVariables.od[index * 3];
acadoWorkspace.state[11] = acadoVariables.od[index * 3 + 1];
acadoWorkspace.state[12] = acadoVariables.od[index * 3 + 2];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index + 1] = acadoWorkspace.state[0];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 40; ++index)
{
acadoVariables.x[index] = acadoVariables.x[index + 1];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[40] = xEnd[0];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[40];
if (uEnd != 0)
{
acadoWorkspace.state[6] = uEnd[0];
acadoWorkspace.state[7] = uEnd[1];
acadoWorkspace.state[8] = uEnd[2];
acadoWorkspace.state[9] = uEnd[3];
}
else
{
acadoWorkspace.state[6] = acadoVariables.u[156];
acadoWorkspace.state[7] = acadoVariables.u[157];
acadoWorkspace.state[8] = acadoVariables.u[158];
acadoWorkspace.state[9] = acadoVariables.u[159];
}
acadoWorkspace.state[10] = acadoVariables.od[120];
acadoWorkspace.state[11] = acadoVariables.od[121];
acadoWorkspace.state[12] = acadoVariables.od[122];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[40] = acadoWorkspace.state[0];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 39; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[156] = uEnd[0];
acadoVariables.u[157] = uEnd[1];
acadoVariables.u[158] = uEnd[2];
acadoVariables.u[159] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119] + acadoWorkspace.g[120]*acadoWorkspace.x[120] + acadoWorkspace.g[121]*acadoWorkspace.x[121] + acadoWorkspace.g[122]*acadoWorkspace.x[122] + acadoWorkspace.g[123]*acadoWorkspace.x[123] + acadoWorkspace.g[124]*acadoWorkspace.x[124] + acadoWorkspace.g[125]*acadoWorkspace.x[125] + acadoWorkspace.g[126]*acadoWorkspace.x[126] + acadoWorkspace.g[127]*acadoWorkspace.x[127] + acadoWorkspace.g[128]*acadoWorkspace.x[128] + acadoWorkspace.g[129]*acadoWorkspace.x[129] + acadoWorkspace.g[130]*acadoWorkspace.x[130] + acadoWorkspace.g[131]*acadoWorkspace.x[131] + acadoWorkspace.g[132]*acadoWorkspace.x[132] + acadoWorkspace.g[133]*acadoWorkspace.x[133] + acadoWorkspace.g[134]*acadoWorkspace.x[134] + acadoWorkspace.g[135]*acadoWorkspace.x[135] + acadoWorkspace.g[136]*acadoWorkspace.x[136] + acadoWorkspace.g[137]*acadoWorkspace.x[137] + acadoWorkspace.g[138]*acadoWorkspace.x[138] + acadoWorkspace.g[139]*acadoWorkspace.x[139] + acadoWorkspace.g[140]*acadoWorkspace.x[140] + acadoWorkspace.g[141]*acadoWorkspace.x[141] + acadoWorkspace.g[142]*acadoWorkspace.x[142] + acadoWorkspace.g[143]*acadoWorkspace.x[143] + acadoWorkspace.g[144]*acadoWorkspace.x[144] + acadoWorkspace.g[145]*acadoWorkspace.x[145] + acadoWorkspace.g[146]*acadoWorkspace.x[146] + acadoWorkspace.g[147]*acadoWorkspace.x[147] + acadoWorkspace.g[148]*acadoWorkspace.x[148] + acadoWorkspace.g[149]*acadoWorkspace.x[149] + acadoWorkspace.g[150]*acadoWorkspace.x[150] + acadoWorkspace.g[151]*acadoWorkspace.x[151] + acadoWorkspace.g[152]*acadoWorkspace.x[152] + acadoWorkspace.g[153]*acadoWorkspace.x[153] + acadoWorkspace.g[154]*acadoWorkspace.x[154] + acadoWorkspace.g[155]*acadoWorkspace.x[155] + acadoWorkspace.g[156]*acadoWorkspace.x[156] + acadoWorkspace.g[157]*acadoWorkspace.x[157] + acadoWorkspace.g[158]*acadoWorkspace.x[158] + acadoWorkspace.g[159]*acadoWorkspace.x[159];
kkt = fabs( kkt );
for (index = 0; index < 160; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index + 160];
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
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Column vector of size: 1 */
real_t tmpDyN[ 1 ];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1];
acadoWorkspace.objValueIn[1] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[2] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[4] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[5] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[40];
acadoWorkspace.objValueIn[1] = acadoVariables.od[120];
acadoWorkspace.objValueIn[2] = acadoVariables.od[121];
acadoWorkspace.objValueIn[3] = acadoVariables.od[122];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
objVal = 0.0000000000000000e+00;
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
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0];

objVal *= 0.5;
return objVal;
}

