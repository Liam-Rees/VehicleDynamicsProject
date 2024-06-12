/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState vx;
    DifferentialState vy;
    DifferentialState r;
    Control Mz;
    BMatrix acadodata_M1;
    acadodata_M1.read( "PF_problem_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "PF_problem_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << vx;
    acadodata_f1 << vy;
    acadodata_f1 << r;
    acadodata_f1 << Mz;
    Function acadodata_f2;
    acadodata_f2 << vx;
    acadodata_f2 << vy;
    acadodata_f2 << r;
    OCP ocp1(0, 0.4, 40);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo(0.00000000000000000000e+00 <= vx <= 4.72222222222222214327e+01);
    ocp1.subjectTo((-1.00000000000000000000e+04) <= Mz <= 1.00000000000000000000e+04);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(vx) == r*vy;
    acadodata_f3 << dot(vy) == ((-3.10000000000000000000e+05)/1.38000000000000000000e+03/vx*vy+(1.01060000000000000000e+05/1.38000000000000000000e+03/vx-vx)*r);
    acadodata_f3 << dot(r) == (1.01060000000000000000e+05/2.63450000000000000000e+03/vx*vy-1/2.63450000000000000000e+03*6.05453560000000055879e+05*r/vx+1/2.63450000000000000000e+03*Mz);

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 1 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 0 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 120 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-04 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( MAX_NUM_QP_ITERATIONS, 20 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: MAX_NUM_QP_ITERATIONS");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( GENERATE_SIMULINK_INTERFACE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: GENERATE_SIMULINK_INTERFACE");
    options_flag = ExportModule1.set( PRINTLEVEL, NONE );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: PRINTLEVEL");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "export_MPC" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

