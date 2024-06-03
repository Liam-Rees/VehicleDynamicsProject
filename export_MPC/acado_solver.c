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

acadoWorkspace.state[3] = acadoVariables.u[lRun1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 + 1];

acadoWorkspace.evGx[lRun1] = acadoWorkspace.state[1];

acadoWorkspace.evGu[lRun1] = acadoWorkspace.state[2];
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
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ1[0] = + tmpQ2[0];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[2];
tmpR2[1] = +tmpObjS[3];
tmpR1[0] = + tmpR2[1];
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
acadoWorkspace.objValueIn[1] = acadoVariables.u[runObj];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 2] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 2 + 1] = acadoWorkspace.objValueOut[1];

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj ]), &(acadoWorkspace.Q2[ runObj * 2 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj ]), &(acadoWorkspace.R2[ runObj * 2 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[40];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol)] = + Gu1[0]*Gu2[0];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 41] = + Gu1[0]*Gu2[0] + R11[0];
acadoWorkspace.H[iRow * 41] += 1.0000000000000000e-04;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Gu2[0];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + w12[0];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0];
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
w12[0] += + Gu1[0]*U1[0];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol)] = acadoWorkspace.H[(iCol * 40) + (iRow)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 40; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 81)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 ]), &(acadoWorkspace.E[ lRun3 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 40; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (1)) * (1)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (1)) * (1)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (1)) * (1)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (40)) - (1)) * (1)) * (1)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 39; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (1)) * (1)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 ]), &(acadoWorkspace.evGu[ lRun2 ]), acadoWorkspace.W1, lRun2 );
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

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
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
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 2 ]), &(acadoWorkspace.Dy[ 2 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 4 ]), &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 6 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 8 ]), &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 10 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 12 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 14 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 16 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 18 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 20 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 22 ]), &(acadoWorkspace.Dy[ 22 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 24 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 26 ]), &(acadoWorkspace.Dy[ 26 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 28 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 32 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 34 ]), &(acadoWorkspace.Dy[ 34 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 38 ]), &(acadoWorkspace.Dy[ 38 ]), &(acadoWorkspace.g[ 19 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 42 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 44 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.g[ 22 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 46 ]), &(acadoWorkspace.Dy[ 46 ]), &(acadoWorkspace.g[ 23 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 50 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 25 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 52 ]), &(acadoWorkspace.Dy[ 52 ]), &(acadoWorkspace.g[ 26 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 54 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 56 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 58 ]), &(acadoWorkspace.Dy[ 58 ]), &(acadoWorkspace.g[ 29 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 62 ]), &(acadoWorkspace.Dy[ 62 ]), &(acadoWorkspace.g[ 31 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 64 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 66 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 68 ]), &(acadoWorkspace.Dy[ 68 ]), &(acadoWorkspace.g[ 34 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 70 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 35 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 74 ]), &(acadoWorkspace.Dy[ 74 ]), &(acadoWorkspace.g[ 37 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 76 ]), &(acadoWorkspace.Dy[ 76 ]), &(acadoWorkspace.g[ 38 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 78 ]), &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.g[ 39 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 2 ]), &(acadoWorkspace.Dy[ 2 ]), &(acadoWorkspace.QDy[ 1 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 4 ]), &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.QDy[ 2 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 6 ]), &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 8 ]), &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 4 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 10 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 12 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 14 ]), &(acadoWorkspace.Dy[ 14 ]), &(acadoWorkspace.QDy[ 7 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 16 ]), &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 18 ]), &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 20 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 22 ]), &(acadoWorkspace.Dy[ 22 ]), &(acadoWorkspace.QDy[ 11 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 24 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 26 ]), &(acadoWorkspace.Dy[ 26 ]), &(acadoWorkspace.QDy[ 13 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 28 ]), &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 14 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 30 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 32 ]), &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 34 ]), &(acadoWorkspace.Dy[ 34 ]), &(acadoWorkspace.QDy[ 17 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 36 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 38 ]), &(acadoWorkspace.Dy[ 38 ]), &(acadoWorkspace.QDy[ 19 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 40 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 42 ]), &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 44 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.QDy[ 22 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 46 ]), &(acadoWorkspace.Dy[ 46 ]), &(acadoWorkspace.QDy[ 23 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 48 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 50 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 52 ]), &(acadoWorkspace.Dy[ 52 ]), &(acadoWorkspace.QDy[ 26 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 54 ]), &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 56 ]), &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 28 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 58 ]), &(acadoWorkspace.Dy[ 58 ]), &(acadoWorkspace.QDy[ 29 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 62 ]), &(acadoWorkspace.Dy[ 62 ]), &(acadoWorkspace.QDy[ 31 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 64 ]), &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 66 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 33 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 68 ]), &(acadoWorkspace.Dy[ 68 ]), &(acadoWorkspace.QDy[ 34 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 70 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 72 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 74 ]), &(acadoWorkspace.Dy[ 74 ]), &(acadoWorkspace.QDy[ 37 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 76 ]), &(acadoWorkspace.Dy[ 76 ]), &(acadoWorkspace.QDy[ 38 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 78 ]), &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.QDy[ 39 ]) );

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
acado_macBTw1( &(acadoWorkspace.evGu[ 39 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 39 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 39 ]), &(acadoWorkspace.sbar[ 39 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 38 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 38 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 38 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 38 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 38 ]), &(acadoWorkspace.sbar[ 38 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 37 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 37 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 37 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 37 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 37 ]), &(acadoWorkspace.sbar[ 37 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 35 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 35 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 35 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 35 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 35 ]), &(acadoWorkspace.sbar[ 35 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 34 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 34 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 34 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 34 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 34 ]), &(acadoWorkspace.sbar[ 34 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 33 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 33 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 33 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 33 ]), &(acadoWorkspace.sbar[ 33 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 32 ]), &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 31 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 31 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 31 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 31 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 31 ]), &(acadoWorkspace.sbar[ 31 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 29 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 29 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 29 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 29 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 29 ]), &(acadoWorkspace.sbar[ 29 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 28 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 28 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 28 ]), &(acadoWorkspace.sbar[ 28 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 26 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 26 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 26 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 26 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 26 ]), &(acadoWorkspace.sbar[ 26 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 25 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 25 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 25 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 25 ]), &(acadoWorkspace.sbar[ 25 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 24 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 23 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 23 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 23 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 23 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 23 ]), &(acadoWorkspace.sbar[ 23 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 22 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 22 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 22 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 22 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 22 ]), &(acadoWorkspace.sbar[ 22 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 21 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 21 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 21 ]), &(acadoWorkspace.sbar[ 21 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 20 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 19 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 19 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 19 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 19 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 19 ]), &(acadoWorkspace.sbar[ 19 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 17 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 17 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 17 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 17 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 17 ]), &(acadoWorkspace.sbar[ 17 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 16 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 16 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 15 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 15 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 15 ]), &(acadoWorkspace.sbar[ 15 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 14 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 14 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 14 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 14 ]), &(acadoWorkspace.sbar[ 14 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 13 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 13 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 13 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 13 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 13 ]), &(acadoWorkspace.sbar[ 13 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 12 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 11 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 11 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 11 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 11 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 11 ]), &(acadoWorkspace.sbar[ 11 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 10 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 10 ]), &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 9 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 8 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 8 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 7 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 7 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 7 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 7 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 7 ]), &(acadoWorkspace.sbar[ 7 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 6 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 6 ]), &(acadoWorkspace.sbar[ 6 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 5 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 5 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 5 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 5 ]), &(acadoWorkspace.sbar[ 5 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 4 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 4 ]), &(acadoWorkspace.sbar[ 4 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 3 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 3 ]), &(acadoWorkspace.sbar[ 3 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 2 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 2 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2 ]), &(acadoWorkspace.sbar[ 2 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 1 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 1 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1 ]), &(acadoWorkspace.sbar[ 1 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


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
acado_expansionStep( &(acadoWorkspace.evGx[ 1 ]), &(acadoWorkspace.evGu[ 1 ]), &(acadoWorkspace.x[ 1 ]), &(acadoWorkspace.sbar[ 1 ]), &(acadoWorkspace.sbar[ 2 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2 ]), &(acadoWorkspace.evGu[ 2 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 2 ]), &(acadoWorkspace.sbar[ 3 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3 ]), &(acadoWorkspace.evGu[ 3 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 3 ]), &(acadoWorkspace.sbar[ 4 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4 ]), &(acadoWorkspace.evGu[ 4 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 4 ]), &(acadoWorkspace.sbar[ 5 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5 ]), &(acadoWorkspace.evGu[ 5 ]), &(acadoWorkspace.x[ 5 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.sbar[ 6 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 6 ]), &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 6 ]), &(acadoWorkspace.sbar[ 7 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 7 ]), &(acadoWorkspace.evGu[ 7 ]), &(acadoWorkspace.x[ 7 ]), &(acadoWorkspace.sbar[ 7 ]), &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 8 ]), &(acadoWorkspace.evGu[ 8 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGu[ 9 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 10 ]), &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 11 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 11 ]), &(acadoWorkspace.evGu[ 11 ]), &(acadoWorkspace.x[ 11 ]), &(acadoWorkspace.sbar[ 11 ]), &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 12 ]), &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 13 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 13 ]), &(acadoWorkspace.evGu[ 13 ]), &(acadoWorkspace.x[ 13 ]), &(acadoWorkspace.sbar[ 13 ]), &(acadoWorkspace.sbar[ 14 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 14 ]), &(acadoWorkspace.evGu[ 14 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 14 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 15 ]), &(acadoWorkspace.evGu[ 15 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 16 ]), &(acadoWorkspace.evGu[ 16 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 17 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 17 ]), &(acadoWorkspace.evGu[ 17 ]), &(acadoWorkspace.x[ 17 ]), &(acadoWorkspace.sbar[ 17 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 19 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 19 ]), &(acadoWorkspace.evGu[ 19 ]), &(acadoWorkspace.x[ 19 ]), &(acadoWorkspace.sbar[ 19 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 20 ]), &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 21 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 21 ]), &(acadoWorkspace.evGu[ 21 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 21 ]), &(acadoWorkspace.sbar[ 22 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 22 ]), &(acadoWorkspace.evGu[ 22 ]), &(acadoWorkspace.x[ 22 ]), &(acadoWorkspace.sbar[ 22 ]), &(acadoWorkspace.sbar[ 23 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 23 ]), &(acadoWorkspace.evGu[ 23 ]), &(acadoWorkspace.x[ 23 ]), &(acadoWorkspace.sbar[ 23 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 24 ]), &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 25 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.evGu[ 25 ]), &(acadoWorkspace.x[ 25 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.sbar[ 26 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 26 ]), &(acadoWorkspace.evGu[ 26 ]), &(acadoWorkspace.x[ 26 ]), &(acadoWorkspace.sbar[ 26 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 28 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 28 ]), &(acadoWorkspace.evGu[ 28 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 28 ]), &(acadoWorkspace.sbar[ 29 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 29 ]), &(acadoWorkspace.evGu[ 29 ]), &(acadoWorkspace.x[ 29 ]), &(acadoWorkspace.sbar[ 29 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 30 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 31 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 31 ]), &(acadoWorkspace.evGu[ 31 ]), &(acadoWorkspace.x[ 31 ]), &(acadoWorkspace.sbar[ 31 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 32 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 33 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 33 ]), &(acadoWorkspace.evGu[ 33 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 33 ]), &(acadoWorkspace.sbar[ 34 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 34 ]), &(acadoWorkspace.evGu[ 34 ]), &(acadoWorkspace.x[ 34 ]), &(acadoWorkspace.sbar[ 34 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 35 ]), &(acadoWorkspace.evGu[ 35 ]), &(acadoWorkspace.x[ 35 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 37 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 37 ]), &(acadoWorkspace.evGu[ 37 ]), &(acadoWorkspace.x[ 37 ]), &(acadoWorkspace.sbar[ 37 ]), &(acadoWorkspace.sbar[ 38 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 38 ]), &(acadoWorkspace.evGu[ 38 ]), &(acadoWorkspace.x[ 38 ]), &(acadoWorkspace.sbar[ 38 ]), &(acadoWorkspace.sbar[ 39 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 39 ]), &(acadoWorkspace.evGu[ 39 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 39 ]), &(acadoWorkspace.sbar[ 40 ]) );
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
acadoWorkspace.state[3] = acadoVariables.u[index];

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
acadoWorkspace.state[3] = uEnd[0];
}
else
{
acadoWorkspace.state[3] = acadoVariables.u[39];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[40] = acadoWorkspace.state[0];
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
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 2 */
real_t tmpDy[ 2 ];

/** Column vector of size: 1 */
real_t tmpDyN[ 1 ];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1];
acadoWorkspace.objValueIn[1] = acadoVariables.u[lRun1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 2] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 2];
acadoWorkspace.Dy[lRun1 * 2 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 2 + 1];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[40];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 2]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 2 + 1]*acadoVariables.W[3];
objVal += + acadoWorkspace.Dy[lRun1 * 2]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 2 + 1]*tmpDy[1];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0];

objVal *= 0.5;
return objVal;
}

