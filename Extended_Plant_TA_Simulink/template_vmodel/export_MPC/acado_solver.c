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
for (runObj = 0; runObj < 40; ++runObj)
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
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
acadoWorkspace.H[(iRow * 40) + (iCol)] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 41] = + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2] + R11[0];
acadoWorkspace.H[iRow * 41] += 1.0000000000000000e-04;
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
acadoWorkspace.H[(iRow * 40) + (iCol)] = acadoWorkspace.H[(iCol * 40) + (iRow)];
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
/** Row vector of size: 40 */
static const int xBoundIndices[ 40 ] = 
{ 3, 6, 9, 12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45, 48, 51, 54, 57, 60, 63, 66, 69, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 114, 117, 120 };
for (lRun2 = 0; lRun2 < 40; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 81)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 3 ]), &(acadoWorkspace.E[ lRun3 * 3 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 40; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (3)) * (3)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (3)) * (1)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (3)) * (1)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (40)) - (1)) * (3)) * (1)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 39; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 3 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 9 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 9 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (3)) * (1)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 ]), &(acadoWorkspace.evGu[ lRun2 * 3 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

acadoWorkspace.sbar[3] = acadoWorkspace.d[0];
acadoWorkspace.sbar[4] = acadoWorkspace.d[1];
acadoWorkspace.sbar[5] = acadoWorkspace.d[2];
acadoWorkspace.sbar[6] = acadoWorkspace.d[3];
acadoWorkspace.sbar[7] = acadoWorkspace.d[4];
acadoWorkspace.sbar[8] = acadoWorkspace.d[5];
acadoWorkspace.sbar[9] = acadoWorkspace.d[6];
acadoWorkspace.sbar[10] = acadoWorkspace.d[7];
acadoWorkspace.sbar[11] = acadoWorkspace.d[8];
acadoWorkspace.sbar[12] = acadoWorkspace.d[9];
acadoWorkspace.sbar[13] = acadoWorkspace.d[10];
acadoWorkspace.sbar[14] = acadoWorkspace.d[11];
acadoWorkspace.sbar[15] = acadoWorkspace.d[12];
acadoWorkspace.sbar[16] = acadoWorkspace.d[13];
acadoWorkspace.sbar[17] = acadoWorkspace.d[14];
acadoWorkspace.sbar[18] = acadoWorkspace.d[15];
acadoWorkspace.sbar[19] = acadoWorkspace.d[16];
acadoWorkspace.sbar[20] = acadoWorkspace.d[17];
acadoWorkspace.sbar[21] = acadoWorkspace.d[18];
acadoWorkspace.sbar[22] = acadoWorkspace.d[19];
acadoWorkspace.sbar[23] = acadoWorkspace.d[20];
acadoWorkspace.sbar[24] = acadoWorkspace.d[21];
acadoWorkspace.sbar[25] = acadoWorkspace.d[22];
acadoWorkspace.sbar[26] = acadoWorkspace.d[23];
acadoWorkspace.sbar[27] = acadoWorkspace.d[24];
acadoWorkspace.sbar[28] = acadoWorkspace.d[25];
acadoWorkspace.sbar[29] = acadoWorkspace.d[26];
acadoWorkspace.sbar[30] = acadoWorkspace.d[27];
acadoWorkspace.sbar[31] = acadoWorkspace.d[28];
acadoWorkspace.sbar[32] = acadoWorkspace.d[29];
acadoWorkspace.sbar[33] = acadoWorkspace.d[30];
acadoWorkspace.sbar[34] = acadoWorkspace.d[31];
acadoWorkspace.sbar[35] = acadoWorkspace.d[32];
acadoWorkspace.sbar[36] = acadoWorkspace.d[33];
acadoWorkspace.sbar[37] = acadoWorkspace.d[34];
acadoWorkspace.sbar[38] = acadoWorkspace.d[35];
acadoWorkspace.sbar[39] = acadoWorkspace.d[36];
acadoWorkspace.sbar[40] = acadoWorkspace.d[37];
acadoWorkspace.sbar[41] = acadoWorkspace.d[38];
acadoWorkspace.sbar[42] = acadoWorkspace.d[39];
acadoWorkspace.sbar[43] = acadoWorkspace.d[40];
acadoWorkspace.sbar[44] = acadoWorkspace.d[41];
acadoWorkspace.sbar[45] = acadoWorkspace.d[42];
acadoWorkspace.sbar[46] = acadoWorkspace.d[43];
acadoWorkspace.sbar[47] = acadoWorkspace.d[44];
acadoWorkspace.sbar[48] = acadoWorkspace.d[45];
acadoWorkspace.sbar[49] = acadoWorkspace.d[46];
acadoWorkspace.sbar[50] = acadoWorkspace.d[47];
acadoWorkspace.sbar[51] = acadoWorkspace.d[48];
acadoWorkspace.sbar[52] = acadoWorkspace.d[49];
acadoWorkspace.sbar[53] = acadoWorkspace.d[50];
acadoWorkspace.sbar[54] = acadoWorkspace.d[51];
acadoWorkspace.sbar[55] = acadoWorkspace.d[52];
acadoWorkspace.sbar[56] = acadoWorkspace.d[53];
acadoWorkspace.sbar[57] = acadoWorkspace.d[54];
acadoWorkspace.sbar[58] = acadoWorkspace.d[55];
acadoWorkspace.sbar[59] = acadoWorkspace.d[56];
acadoWorkspace.sbar[60] = acadoWorkspace.d[57];
acadoWorkspace.sbar[61] = acadoWorkspace.d[58];
acadoWorkspace.sbar[62] = acadoWorkspace.d[59];
acadoWorkspace.sbar[63] = acadoWorkspace.d[60];
acadoWorkspace.sbar[64] = acadoWorkspace.d[61];
acadoWorkspace.sbar[65] = acadoWorkspace.d[62];
acadoWorkspace.sbar[66] = acadoWorkspace.d[63];
acadoWorkspace.sbar[67] = acadoWorkspace.d[64];
acadoWorkspace.sbar[68] = acadoWorkspace.d[65];
acadoWorkspace.sbar[69] = acadoWorkspace.d[66];
acadoWorkspace.sbar[70] = acadoWorkspace.d[67];
acadoWorkspace.sbar[71] = acadoWorkspace.d[68];
acadoWorkspace.sbar[72] = acadoWorkspace.d[69];
acadoWorkspace.sbar[73] = acadoWorkspace.d[70];
acadoWorkspace.sbar[74] = acadoWorkspace.d[71];
acadoWorkspace.sbar[75] = acadoWorkspace.d[72];
acadoWorkspace.sbar[76] = acadoWorkspace.d[73];
acadoWorkspace.sbar[77] = acadoWorkspace.d[74];
acadoWorkspace.sbar[78] = acadoWorkspace.d[75];
acadoWorkspace.sbar[79] = acadoWorkspace.d[76];
acadoWorkspace.sbar[80] = acadoWorkspace.d[77];
acadoWorkspace.sbar[81] = acadoWorkspace.d[78];
acadoWorkspace.sbar[82] = acadoWorkspace.d[79];
acadoWorkspace.sbar[83] = acadoWorkspace.d[80];
acadoWorkspace.sbar[84] = acadoWorkspace.d[81];
acadoWorkspace.sbar[85] = acadoWorkspace.d[82];
acadoWorkspace.sbar[86] = acadoWorkspace.d[83];
acadoWorkspace.sbar[87] = acadoWorkspace.d[84];
acadoWorkspace.sbar[88] = acadoWorkspace.d[85];
acadoWorkspace.sbar[89] = acadoWorkspace.d[86];
acadoWorkspace.sbar[90] = acadoWorkspace.d[87];
acadoWorkspace.sbar[91] = acadoWorkspace.d[88];
acadoWorkspace.sbar[92] = acadoWorkspace.d[89];
acadoWorkspace.sbar[93] = acadoWorkspace.d[90];
acadoWorkspace.sbar[94] = acadoWorkspace.d[91];
acadoWorkspace.sbar[95] = acadoWorkspace.d[92];
acadoWorkspace.sbar[96] = acadoWorkspace.d[93];
acadoWorkspace.sbar[97] = acadoWorkspace.d[94];
acadoWorkspace.sbar[98] = acadoWorkspace.d[95];
acadoWorkspace.sbar[99] = acadoWorkspace.d[96];
acadoWorkspace.sbar[100] = acadoWorkspace.d[97];
acadoWorkspace.sbar[101] = acadoWorkspace.d[98];
acadoWorkspace.sbar[102] = acadoWorkspace.d[99];
acadoWorkspace.sbar[103] = acadoWorkspace.d[100];
acadoWorkspace.sbar[104] = acadoWorkspace.d[101];
acadoWorkspace.sbar[105] = acadoWorkspace.d[102];
acadoWorkspace.sbar[106] = acadoWorkspace.d[103];
acadoWorkspace.sbar[107] = acadoWorkspace.d[104];
acadoWorkspace.sbar[108] = acadoWorkspace.d[105];
acadoWorkspace.sbar[109] = acadoWorkspace.d[106];
acadoWorkspace.sbar[110] = acadoWorkspace.d[107];
acadoWorkspace.sbar[111] = acadoWorkspace.d[108];
acadoWorkspace.sbar[112] = acadoWorkspace.d[109];
acadoWorkspace.sbar[113] = acadoWorkspace.d[110];
acadoWorkspace.sbar[114] = acadoWorkspace.d[111];
acadoWorkspace.sbar[115] = acadoWorkspace.d[112];
acadoWorkspace.sbar[116] = acadoWorkspace.d[113];
acadoWorkspace.sbar[117] = acadoWorkspace.d[114];
acadoWorkspace.sbar[118] = acadoWorkspace.d[115];
acadoWorkspace.sbar[119] = acadoWorkspace.d[116];
acadoWorkspace.sbar[120] = acadoWorkspace.d[117];
acadoWorkspace.sbar[121] = acadoWorkspace.d[118];
acadoWorkspace.sbar[122] = acadoWorkspace.d[119];
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-1.0000000000000000e+04 - acadoVariables.u[39];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+04 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+04 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+04 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+04 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+04 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+04 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+04 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+04 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+04 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+04 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+04 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+04 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+04 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+04 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+04 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+04 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+04 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+04 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+04 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+04 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+04 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+04 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+04 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+04 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+04 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+04 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+04 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+04 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+04 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+04 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+04 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.0000000000000000e+04 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)1.0000000000000000e+04 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)1.0000000000000000e+04 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+04 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.0000000000000000e+04 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)1.0000000000000000e+04 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)1.0000000000000000e+04 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+04 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.0000000000000000e+04 - acadoVariables.u[39];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 3;
lRun4 = ((lRun3) / (3)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 79)) / (2)) + (lRun4)) - (1)) * (3)) + ((lRun3) % (3));
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
for (lRun1 = 0; lRun1 < 160; ++lRun1)
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

acadoWorkspace.QDy[120] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[121] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[122] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];

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

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[122] + acadoWorkspace.QDy[120];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[122] + acadoWorkspace.QDy[121];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[120] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[121] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[122] + acadoWorkspace.QDy[122];
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


tmp = acadoWorkspace.sbar[3] + acadoVariables.x[3];
acadoWorkspace.lbA[0] = - tmp;
acadoWorkspace.ubA[0] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[6] + acadoVariables.x[6];
acadoWorkspace.lbA[1] = - tmp;
acadoWorkspace.ubA[1] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[9] + acadoVariables.x[9];
acadoWorkspace.lbA[2] = - tmp;
acadoWorkspace.ubA[2] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[12] + acadoVariables.x[12];
acadoWorkspace.lbA[3] = - tmp;
acadoWorkspace.ubA[3] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[15] + acadoVariables.x[15];
acadoWorkspace.lbA[4] = - tmp;
acadoWorkspace.ubA[4] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[18] + acadoVariables.x[18];
acadoWorkspace.lbA[5] = - tmp;
acadoWorkspace.ubA[5] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[21] + acadoVariables.x[21];
acadoWorkspace.lbA[6] = - tmp;
acadoWorkspace.ubA[6] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[7] = - tmp;
acadoWorkspace.ubA[7] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[27] + acadoVariables.x[27];
acadoWorkspace.lbA[8] = - tmp;
acadoWorkspace.ubA[8] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[30] + acadoVariables.x[30];
acadoWorkspace.lbA[9] = - tmp;
acadoWorkspace.ubA[9] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[33] + acadoVariables.x[33];
acadoWorkspace.lbA[10] = - tmp;
acadoWorkspace.ubA[10] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[36] + acadoVariables.x[36];
acadoWorkspace.lbA[11] = - tmp;
acadoWorkspace.ubA[11] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[39] + acadoVariables.x[39];
acadoWorkspace.lbA[12] = - tmp;
acadoWorkspace.ubA[12] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[42] + acadoVariables.x[42];
acadoWorkspace.lbA[13] = - tmp;
acadoWorkspace.ubA[13] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[45] + acadoVariables.x[45];
acadoWorkspace.lbA[14] = - tmp;
acadoWorkspace.ubA[14] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[48] + acadoVariables.x[48];
acadoWorkspace.lbA[15] = - tmp;
acadoWorkspace.ubA[15] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[51] + acadoVariables.x[51];
acadoWorkspace.lbA[16] = - tmp;
acadoWorkspace.ubA[16] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[54] + acadoVariables.x[54];
acadoWorkspace.lbA[17] = - tmp;
acadoWorkspace.ubA[17] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[57] + acadoVariables.x[57];
acadoWorkspace.lbA[18] = - tmp;
acadoWorkspace.ubA[18] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[60] + acadoVariables.x[60];
acadoWorkspace.lbA[19] = - tmp;
acadoWorkspace.ubA[19] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[63] + acadoVariables.x[63];
acadoWorkspace.lbA[20] = - tmp;
acadoWorkspace.ubA[20] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[66] + acadoVariables.x[66];
acadoWorkspace.lbA[21] = - tmp;
acadoWorkspace.ubA[21] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[69] + acadoVariables.x[69];
acadoWorkspace.lbA[22] = - tmp;
acadoWorkspace.ubA[22] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[72] + acadoVariables.x[72];
acadoWorkspace.lbA[23] = - tmp;
acadoWorkspace.ubA[23] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[75] + acadoVariables.x[75];
acadoWorkspace.lbA[24] = - tmp;
acadoWorkspace.ubA[24] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[78] + acadoVariables.x[78];
acadoWorkspace.lbA[25] = - tmp;
acadoWorkspace.ubA[25] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[81] + acadoVariables.x[81];
acadoWorkspace.lbA[26] = - tmp;
acadoWorkspace.ubA[26] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[84] + acadoVariables.x[84];
acadoWorkspace.lbA[27] = - tmp;
acadoWorkspace.ubA[27] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[87] + acadoVariables.x[87];
acadoWorkspace.lbA[28] = - tmp;
acadoWorkspace.ubA[28] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[90] + acadoVariables.x[90];
acadoWorkspace.lbA[29] = - tmp;
acadoWorkspace.ubA[29] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[93] + acadoVariables.x[93];
acadoWorkspace.lbA[30] = - tmp;
acadoWorkspace.ubA[30] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[96] + acadoVariables.x[96];
acadoWorkspace.lbA[31] = - tmp;
acadoWorkspace.ubA[31] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[99] + acadoVariables.x[99];
acadoWorkspace.lbA[32] = - tmp;
acadoWorkspace.ubA[32] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[102] + acadoVariables.x[102];
acadoWorkspace.lbA[33] = - tmp;
acadoWorkspace.ubA[33] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[105] + acadoVariables.x[105];
acadoWorkspace.lbA[34] = - tmp;
acadoWorkspace.ubA[34] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[108] + acadoVariables.x[108];
acadoWorkspace.lbA[35] = - tmp;
acadoWorkspace.ubA[35] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[111] + acadoVariables.x[111];
acadoWorkspace.lbA[36] = - tmp;
acadoWorkspace.ubA[36] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[114] + acadoVariables.x[114];
acadoWorkspace.lbA[37] = - tmp;
acadoWorkspace.ubA[37] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[117] + acadoVariables.x[117];
acadoWorkspace.lbA[38] = - tmp;
acadoWorkspace.ubA[38] = (real_t)4.7222222222222221e+01 - tmp;
tmp = acadoWorkspace.sbar[120] + acadoVariables.x[120];
acadoWorkspace.lbA[39] = - tmp;
acadoWorkspace.ubA[39] = (real_t)4.7222222222222221e+01 - tmp;

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
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.d[0];
acadoWorkspace.sbar[4] = acadoWorkspace.d[1];
acadoWorkspace.sbar[5] = acadoWorkspace.d[2];
acadoWorkspace.sbar[6] = acadoWorkspace.d[3];
acadoWorkspace.sbar[7] = acadoWorkspace.d[4];
acadoWorkspace.sbar[8] = acadoWorkspace.d[5];
acadoWorkspace.sbar[9] = acadoWorkspace.d[6];
acadoWorkspace.sbar[10] = acadoWorkspace.d[7];
acadoWorkspace.sbar[11] = acadoWorkspace.d[8];
acadoWorkspace.sbar[12] = acadoWorkspace.d[9];
acadoWorkspace.sbar[13] = acadoWorkspace.d[10];
acadoWorkspace.sbar[14] = acadoWorkspace.d[11];
acadoWorkspace.sbar[15] = acadoWorkspace.d[12];
acadoWorkspace.sbar[16] = acadoWorkspace.d[13];
acadoWorkspace.sbar[17] = acadoWorkspace.d[14];
acadoWorkspace.sbar[18] = acadoWorkspace.d[15];
acadoWorkspace.sbar[19] = acadoWorkspace.d[16];
acadoWorkspace.sbar[20] = acadoWorkspace.d[17];
acadoWorkspace.sbar[21] = acadoWorkspace.d[18];
acadoWorkspace.sbar[22] = acadoWorkspace.d[19];
acadoWorkspace.sbar[23] = acadoWorkspace.d[20];
acadoWorkspace.sbar[24] = acadoWorkspace.d[21];
acadoWorkspace.sbar[25] = acadoWorkspace.d[22];
acadoWorkspace.sbar[26] = acadoWorkspace.d[23];
acadoWorkspace.sbar[27] = acadoWorkspace.d[24];
acadoWorkspace.sbar[28] = acadoWorkspace.d[25];
acadoWorkspace.sbar[29] = acadoWorkspace.d[26];
acadoWorkspace.sbar[30] = acadoWorkspace.d[27];
acadoWorkspace.sbar[31] = acadoWorkspace.d[28];
acadoWorkspace.sbar[32] = acadoWorkspace.d[29];
acadoWorkspace.sbar[33] = acadoWorkspace.d[30];
acadoWorkspace.sbar[34] = acadoWorkspace.d[31];
acadoWorkspace.sbar[35] = acadoWorkspace.d[32];
acadoWorkspace.sbar[36] = acadoWorkspace.d[33];
acadoWorkspace.sbar[37] = acadoWorkspace.d[34];
acadoWorkspace.sbar[38] = acadoWorkspace.d[35];
acadoWorkspace.sbar[39] = acadoWorkspace.d[36];
acadoWorkspace.sbar[40] = acadoWorkspace.d[37];
acadoWorkspace.sbar[41] = acadoWorkspace.d[38];
acadoWorkspace.sbar[42] = acadoWorkspace.d[39];
acadoWorkspace.sbar[43] = acadoWorkspace.d[40];
acadoWorkspace.sbar[44] = acadoWorkspace.d[41];
acadoWorkspace.sbar[45] = acadoWorkspace.d[42];
acadoWorkspace.sbar[46] = acadoWorkspace.d[43];
acadoWorkspace.sbar[47] = acadoWorkspace.d[44];
acadoWorkspace.sbar[48] = acadoWorkspace.d[45];
acadoWorkspace.sbar[49] = acadoWorkspace.d[46];
acadoWorkspace.sbar[50] = acadoWorkspace.d[47];
acadoWorkspace.sbar[51] = acadoWorkspace.d[48];
acadoWorkspace.sbar[52] = acadoWorkspace.d[49];
acadoWorkspace.sbar[53] = acadoWorkspace.d[50];
acadoWorkspace.sbar[54] = acadoWorkspace.d[51];
acadoWorkspace.sbar[55] = acadoWorkspace.d[52];
acadoWorkspace.sbar[56] = acadoWorkspace.d[53];
acadoWorkspace.sbar[57] = acadoWorkspace.d[54];
acadoWorkspace.sbar[58] = acadoWorkspace.d[55];
acadoWorkspace.sbar[59] = acadoWorkspace.d[56];
acadoWorkspace.sbar[60] = acadoWorkspace.d[57];
acadoWorkspace.sbar[61] = acadoWorkspace.d[58];
acadoWorkspace.sbar[62] = acadoWorkspace.d[59];
acadoWorkspace.sbar[63] = acadoWorkspace.d[60];
acadoWorkspace.sbar[64] = acadoWorkspace.d[61];
acadoWorkspace.sbar[65] = acadoWorkspace.d[62];
acadoWorkspace.sbar[66] = acadoWorkspace.d[63];
acadoWorkspace.sbar[67] = acadoWorkspace.d[64];
acadoWorkspace.sbar[68] = acadoWorkspace.d[65];
acadoWorkspace.sbar[69] = acadoWorkspace.d[66];
acadoWorkspace.sbar[70] = acadoWorkspace.d[67];
acadoWorkspace.sbar[71] = acadoWorkspace.d[68];
acadoWorkspace.sbar[72] = acadoWorkspace.d[69];
acadoWorkspace.sbar[73] = acadoWorkspace.d[70];
acadoWorkspace.sbar[74] = acadoWorkspace.d[71];
acadoWorkspace.sbar[75] = acadoWorkspace.d[72];
acadoWorkspace.sbar[76] = acadoWorkspace.d[73];
acadoWorkspace.sbar[77] = acadoWorkspace.d[74];
acadoWorkspace.sbar[78] = acadoWorkspace.d[75];
acadoWorkspace.sbar[79] = acadoWorkspace.d[76];
acadoWorkspace.sbar[80] = acadoWorkspace.d[77];
acadoWorkspace.sbar[81] = acadoWorkspace.d[78];
acadoWorkspace.sbar[82] = acadoWorkspace.d[79];
acadoWorkspace.sbar[83] = acadoWorkspace.d[80];
acadoWorkspace.sbar[84] = acadoWorkspace.d[81];
acadoWorkspace.sbar[85] = acadoWorkspace.d[82];
acadoWorkspace.sbar[86] = acadoWorkspace.d[83];
acadoWorkspace.sbar[87] = acadoWorkspace.d[84];
acadoWorkspace.sbar[88] = acadoWorkspace.d[85];
acadoWorkspace.sbar[89] = acadoWorkspace.d[86];
acadoWorkspace.sbar[90] = acadoWorkspace.d[87];
acadoWorkspace.sbar[91] = acadoWorkspace.d[88];
acadoWorkspace.sbar[92] = acadoWorkspace.d[89];
acadoWorkspace.sbar[93] = acadoWorkspace.d[90];
acadoWorkspace.sbar[94] = acadoWorkspace.d[91];
acadoWorkspace.sbar[95] = acadoWorkspace.d[92];
acadoWorkspace.sbar[96] = acadoWorkspace.d[93];
acadoWorkspace.sbar[97] = acadoWorkspace.d[94];
acadoWorkspace.sbar[98] = acadoWorkspace.d[95];
acadoWorkspace.sbar[99] = acadoWorkspace.d[96];
acadoWorkspace.sbar[100] = acadoWorkspace.d[97];
acadoWorkspace.sbar[101] = acadoWorkspace.d[98];
acadoWorkspace.sbar[102] = acadoWorkspace.d[99];
acadoWorkspace.sbar[103] = acadoWorkspace.d[100];
acadoWorkspace.sbar[104] = acadoWorkspace.d[101];
acadoWorkspace.sbar[105] = acadoWorkspace.d[102];
acadoWorkspace.sbar[106] = acadoWorkspace.d[103];
acadoWorkspace.sbar[107] = acadoWorkspace.d[104];
acadoWorkspace.sbar[108] = acadoWorkspace.d[105];
acadoWorkspace.sbar[109] = acadoWorkspace.d[106];
acadoWorkspace.sbar[110] = acadoWorkspace.d[107];
acadoWorkspace.sbar[111] = acadoWorkspace.d[108];
acadoWorkspace.sbar[112] = acadoWorkspace.d[109];
acadoWorkspace.sbar[113] = acadoWorkspace.d[110];
acadoWorkspace.sbar[114] = acadoWorkspace.d[111];
acadoWorkspace.sbar[115] = acadoWorkspace.d[112];
acadoWorkspace.sbar[116] = acadoWorkspace.d[113];
acadoWorkspace.sbar[117] = acadoWorkspace.d[114];
acadoWorkspace.sbar[118] = acadoWorkspace.d[115];
acadoWorkspace.sbar[119] = acadoWorkspace.d[116];
acadoWorkspace.sbar[120] = acadoWorkspace.d[117];
acadoWorkspace.sbar[121] = acadoWorkspace.d[118];
acadoWorkspace.sbar[122] = acadoWorkspace.d[119];
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
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
acadoVariables.x[44] += acadoWorkspace.sbar[44];
acadoVariables.x[45] += acadoWorkspace.sbar[45];
acadoVariables.x[46] += acadoWorkspace.sbar[46];
acadoVariables.x[47] += acadoWorkspace.sbar[47];
acadoVariables.x[48] += acadoWorkspace.sbar[48];
acadoVariables.x[49] += acadoWorkspace.sbar[49];
acadoVariables.x[50] += acadoWorkspace.sbar[50];
acadoVariables.x[51] += acadoWorkspace.sbar[51];
acadoVariables.x[52] += acadoWorkspace.sbar[52];
acadoVariables.x[53] += acadoWorkspace.sbar[53];
acadoVariables.x[54] += acadoWorkspace.sbar[54];
acadoVariables.x[55] += acadoWorkspace.sbar[55];
acadoVariables.x[56] += acadoWorkspace.sbar[56];
acadoVariables.x[57] += acadoWorkspace.sbar[57];
acadoVariables.x[58] += acadoWorkspace.sbar[58];
acadoVariables.x[59] += acadoWorkspace.sbar[59];
acadoVariables.x[60] += acadoWorkspace.sbar[60];
acadoVariables.x[61] += acadoWorkspace.sbar[61];
acadoVariables.x[62] += acadoWorkspace.sbar[62];
acadoVariables.x[63] += acadoWorkspace.sbar[63];
acadoVariables.x[64] += acadoWorkspace.sbar[64];
acadoVariables.x[65] += acadoWorkspace.sbar[65];
acadoVariables.x[66] += acadoWorkspace.sbar[66];
acadoVariables.x[67] += acadoWorkspace.sbar[67];
acadoVariables.x[68] += acadoWorkspace.sbar[68];
acadoVariables.x[69] += acadoWorkspace.sbar[69];
acadoVariables.x[70] += acadoWorkspace.sbar[70];
acadoVariables.x[71] += acadoWorkspace.sbar[71];
acadoVariables.x[72] += acadoWorkspace.sbar[72];
acadoVariables.x[73] += acadoWorkspace.sbar[73];
acadoVariables.x[74] += acadoWorkspace.sbar[74];
acadoVariables.x[75] += acadoWorkspace.sbar[75];
acadoVariables.x[76] += acadoWorkspace.sbar[76];
acadoVariables.x[77] += acadoWorkspace.sbar[77];
acadoVariables.x[78] += acadoWorkspace.sbar[78];
acadoVariables.x[79] += acadoWorkspace.sbar[79];
acadoVariables.x[80] += acadoWorkspace.sbar[80];
acadoVariables.x[81] += acadoWorkspace.sbar[81];
acadoVariables.x[82] += acadoWorkspace.sbar[82];
acadoVariables.x[83] += acadoWorkspace.sbar[83];
acadoVariables.x[84] += acadoWorkspace.sbar[84];
acadoVariables.x[85] += acadoWorkspace.sbar[85];
acadoVariables.x[86] += acadoWorkspace.sbar[86];
acadoVariables.x[87] += acadoWorkspace.sbar[87];
acadoVariables.x[88] += acadoWorkspace.sbar[88];
acadoVariables.x[89] += acadoWorkspace.sbar[89];
acadoVariables.x[90] += acadoWorkspace.sbar[90];
acadoVariables.x[91] += acadoWorkspace.sbar[91];
acadoVariables.x[92] += acadoWorkspace.sbar[92];
acadoVariables.x[93] += acadoWorkspace.sbar[93];
acadoVariables.x[94] += acadoWorkspace.sbar[94];
acadoVariables.x[95] += acadoWorkspace.sbar[95];
acadoVariables.x[96] += acadoWorkspace.sbar[96];
acadoVariables.x[97] += acadoWorkspace.sbar[97];
acadoVariables.x[98] += acadoWorkspace.sbar[98];
acadoVariables.x[99] += acadoWorkspace.sbar[99];
acadoVariables.x[100] += acadoWorkspace.sbar[100];
acadoVariables.x[101] += acadoWorkspace.sbar[101];
acadoVariables.x[102] += acadoWorkspace.sbar[102];
acadoVariables.x[103] += acadoWorkspace.sbar[103];
acadoVariables.x[104] += acadoWorkspace.sbar[104];
acadoVariables.x[105] += acadoWorkspace.sbar[105];
acadoVariables.x[106] += acadoWorkspace.sbar[106];
acadoVariables.x[107] += acadoWorkspace.sbar[107];
acadoVariables.x[108] += acadoWorkspace.sbar[108];
acadoVariables.x[109] += acadoWorkspace.sbar[109];
acadoVariables.x[110] += acadoWorkspace.sbar[110];
acadoVariables.x[111] += acadoWorkspace.sbar[111];
acadoVariables.x[112] += acadoWorkspace.sbar[112];
acadoVariables.x[113] += acadoWorkspace.sbar[113];
acadoVariables.x[114] += acadoWorkspace.sbar[114];
acadoVariables.x[115] += acadoWorkspace.sbar[115];
acadoVariables.x[116] += acadoWorkspace.sbar[116];
acadoVariables.x[117] += acadoWorkspace.sbar[117];
acadoVariables.x[118] += acadoWorkspace.sbar[118];
acadoVariables.x[119] += acadoWorkspace.sbar[119];
acadoVariables.x[120] += acadoWorkspace.sbar[120];
acadoVariables.x[121] += acadoWorkspace.sbar[121];
acadoVariables.x[122] += acadoWorkspace.sbar[122];
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
for (index = 0; index < 40; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[120] = xEnd[0];
acadoVariables.x[121] = xEnd[1];
acadoVariables.x[122] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[120];
acadoWorkspace.state[1] = acadoVariables.x[121];
acadoWorkspace.state[2] = acadoVariables.x[122];
if (uEnd != 0)
{
acadoWorkspace.state[15] = uEnd[0];
}
else
{
acadoWorkspace.state[15] = acadoVariables.u[39];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[120] = acadoWorkspace.state[0];
acadoVariables.x[121] = acadoWorkspace.state[1];
acadoVariables.x[122] = acadoWorkspace.state[2];
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
for (index = 0; index < 40; ++index)
{
prd = acadoWorkspace.y[index + 40];
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
/** Row vector of size: 4 */
real_t tmpDy[ 4 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 40; ++lRun1)
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
acadoWorkspace.objValueIn[0] = acadoVariables.x[120];
acadoWorkspace.objValueIn[1] = acadoVariables.x[121];
acadoWorkspace.objValueIn[2] = acadoVariables.x[122];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 40; ++lRun1)
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

