/**
 * acado_solver_sfun.c
 *
 *    ABSTRACT:
 *      The purpose of this sfunction is to call a simple legacy
 *      function during simulation:
 *
 *         acado_step(ACADOinput u1[1], ACADOoutput y1[1])
 *
 *    Simulink version           : 23.2 (R2023b) 01-Aug-2023
 *    C source code generated on : 11-Jun-2024 21:02:56
 *
 * THIS S-FUNCTION IS GENERATED BY THE LEGACY CODE TOOL AND MAY NOT WORK IF MODIFIED
 */

/**
     %%%-MATLAB_Construction_Commands_Start
     def = legacy_code('initialize');
     def.SFunctionName = 'acado_solver_sfun';
     def.OutputFcnSpec = 'acado_step(ACADOinput u1[1], ACADOoutput y1[1])';
     def.StartFcnSpec = 'acado_initialize( void )';
     def.TerminateFcnSpec = 'acado_cleanup( void )';
     def.HeaderFiles = {'acado_common.h', 'acado_solver_sfunction.h'};
     def.SourceFiles = {'acado_solver.c', 'acado_integrator.c', 'acado_auxiliary_functions.c', 'acado_solver_sfunction.c', 'acado_qpoases3_interface.c', 'qpoases3\src\Bounds.c', 'qpoases3\src\Constraints.c', 'qpoases3\src\Indexlist.c', 'qpoases3\src\Matrices.c', 'qpoases3\src\MessageHandling.c', 'qpoases3\src\Options.c', 'qpoases3\src\Flipper.c', 'qpoases3\src\QProblem.c', 'qpoases3\src\QProblemB.c', 'qpoases3\src\Utils.c'};
     def.IncPaths = {'qpoases3', 'qpoases3/include', 'qpoases3/src'};
     legacy_code('sfcn_cmex_generate', def);
     legacy_code('compile', def);
     %%%-MATLAB_Construction_Commands_End
 */

/* Must specify the S_FUNCTION_NAME as the name of the S-function */
#define S_FUNCTION_NAME  acado_solver_sfun
#define S_FUNCTION_LEVEL 2

/**
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

#include <string.h>
#include <stdlib.h>

/* Specific header file(s) required by the legacy code function */
#include "acado_common.h"
#include "acado_solver_sfunction.h"

#define IS_SIMULATION_TARGET(S) (ssRTWGenIsAccelerator(S) || ssIsRapidAcceleratorActive(S) || ssRTWGenIsModelReferenceSimTarget(S) || (ssGetSimMode(S)==SS_SIMMODE_NORMAL) || (ssGetSimMode(S)==SS_SIMMODE_SIZES_CALL_ONLY) || !((ssRTWGenIsCodeGen(S) || ssGetSimMode(S)==SS_SIMMODE_EXTERNAL) && GetRTWEnvironmentMode(S)==0))


/* Utility function prototypes */
static void CheckDataTypes(SimStruct *S);
static int_T GetRTWEnvironmentMode(SimStruct *S);

/* Function: mdlInitializeSizes ===========================================
 * Abstract:
 *   The sizes information is used by Simulink to determine the S-function
 *   block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    /* Flag for detecting standalone or simulation target mode */
    boolean_T isSimulationTarget = IS_SIMULATION_TARGET(S);

    /* Number of expected parameters */
    ssSetNumSFcnParams(S, 0);


    /* Set the number of work vectors */
    if (!IS_SIMULATION_TARGET(S)) {
        ssSetNumPWork(S, 0);
        if (!ssSetNumDWork(S, 0)) return;
    } else {
        ssSetNumPWork(S, 2);
        if (!ssSetNumDWork(S, 2)) return;

        /* Configure the dwork 1 (__dtSizeInfo) */
        ssSetDWorkDataType(S, 0, SS_INT32);
        ssSetDWorkUsageType(S, 0, SS_DWORK_USED_AS_DWORK);
        ssSetDWorkName(S, 0, "dtSizeInfo");
        ssSetDWorkWidth(S, 0, 5);
        ssSetDWorkComplexSignal(S, 0, COMPLEX_NO);

        /* Configure the dwork 2 (__dtBusInfo) */
        ssSetDWorkDataType(S, 1, SS_INT32);
        ssSetDWorkUsageType(S, 1, SS_DWORK_USED_AS_DWORK);
        ssSetDWorkName(S, 1, "dtBusInfo");
        ssSetDWorkWidth(S, 1, 46);
        ssSetDWorkComplexSignal(S, 1, COMPLEX_NO);
    } /* if */

    /* Set the number of input ports */
    if (!ssSetNumInputPorts(S, 1)) return;

    /* Configure the input port 1 */
    #if defined(MATLAB_MEX_FILE) 
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
        DTypeId dataTypeIdReg;

        ssRegisterTypeFromNamedObject(S, "ACADOinput", &dataTypeIdReg);
        if(dataTypeIdReg == INVALID_DTYPE_ID) return;

        ssSetInputPortDataType(S, 0, dataTypeIdReg);
    } /* if */
    #endif
    ssSetBusInputAsStruct(S, 0, 1);
    {
        int_T u1Width = 1;
        ssSetInputPortWidth(S, 0, u1Width);
    }
    ssSetInputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortAcceptExprInRTW(S, 0, 0);
    ssSetInputPortOverWritable(S, 0, 0);
    ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    ssSetInputPortDimensionsMode(S, 0, FIXED_DIMS_MODE);

    /* Set the number of output ports */
    if (!ssSetNumOutputPorts(S, 1)) return;

    /* Configure the output port 1 */
    #if defined(MATLAB_MEX_FILE) 
    if (ssGetSimMode(S) != SS_SIMMODE_SIZES_CALL_ONLY) {
        DTypeId dataTypeIdReg;

        ssRegisterTypeFromNamedObject(S, "ACADOoutput", &dataTypeIdReg);
        if(dataTypeIdReg == INVALID_DTYPE_ID) return;

        ssSetOutputPortDataType(S, 0, dataTypeIdReg);
    } /* if */
    #endif
    ssSetBusOutputObjectName(S, 0, (void *)"ACADOoutput");
    ssSetBusOutputAsStruct(S, 0, 1);
    {
        int_T y1Width = 1;
        ssSetOutputPortWidth(S, 0, y1Width);
    }
    ssSetOutputPortComplexSignal(S, 0, COMPLEX_NO);
    ssSetOutputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
    ssSetOutputPortOutputExprInRTW(S, 0, 0);
    ssSetOutputPortDimensionsMode(S, 0, FIXED_DIMS_MODE);

    /* Register reserved identifiers to avoid name conflict */
    if (ssRTWGenIsCodeGen(S) || ssGetSimMode(S)==SS_SIMMODE_EXTERNAL) {

        /* Register reserved identifier for  */
        ssRegMdlInfo(S, "acado_initialize", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath(S));

        /* Register reserved identifier for  */
        ssRegMdlInfo(S, "acado_step", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath(S));

        /* Register reserved identifier for  */
        ssRegMdlInfo(S, "acado_cleanup", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath(S));

        /* Register reserved identifier for wrappers */
        if (isSimulationTarget) {

            /* Register reserved identifier for  */
            ssRegMdlInfo(S, "acado_initialize_wrapper_Start", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath(S));

            /* Register reserved identifier for  */
            ssRegMdlInfo(S, "acado_step_wrapper_Output", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath(S));

            /* Register reserved identifier for  */
            ssRegMdlInfo(S, "acado_cleanup_wrapper_Terminate", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath(S));

            /* Register reserved identifier for allocating PWork for SimulationTarget */
            ssRegMdlInfo(S, "acado_solver_sfun_wrapper_allocmem", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath(S));

            /* Register reserved identifier for freeing PWork for SimulationTarget */
            ssRegMdlInfo(S, "acado_solver_sfun_wrapper_freemem", MDL_INFO_ID_RESERVED, 0, 0, ssGetPath(S));
        } /* if */
    } /* if */

    /* This S-function can be used in referenced model simulating in normal mode */
    ssSetModelReferenceNormalModeSupport(S, MDL_START_AND_MDL_PROCESS_PARAMS_OK);

    /* Set the number of sample time */
    ssSetNumSampleTimes(S, 1);

    /* Set the compliance for the operating point save/restore. */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);

    ssSetArrayLayoutForCodeGen(S, SS_ALL);

    /* Set the Simulink version this S-Function has been generated in */
    ssSetSimulinkVersionGeneratedIn(S, "23.2");

    /**
     * All options have the form SS_OPTION_<name> and are documented in
     * matlabroot/simulink/include/simstruc.h. The options should be
     * bitwise or'd together as in
     *    ssSetOptions(S, (SS_OPTION_name1 | SS_OPTION_name2))
     */
    ssSetOptions(S,
        SS_OPTION_USE_TLC_WITH_ACCELERATOR |
        SS_OPTION_CAN_BE_CALLED_CONDITIONALLY |
        SS_OPTION_EXCEPTION_FREE_CODE |
        SS_OPTION_WORKS_WITH_CODE_REUSE |
        SS_OPTION_SFUNCTION_INLINED_FOR_RTW |
        SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME
    );

    /* Verify Data Type consistency with specification */
    #if defined(MATLAB_MEX_FILE)
    if ((ssGetSimMode(S)!=SS_SIMMODE_SIZES_CALL_ONLY)) {
        CheckDataTypes(S);
    } /* if */
    #endif
}

/* Function: mdlInitializeSampleTimes =====================================
 * Abstract:
 *   This function is used to specify the sample time(s) for your
 *   S-function. You must register the same number of sample times as
 *   specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);

    #if defined(ssSetModelReferenceSampleTimeDefaultInheritance)
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
    #endif
}

#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
/* Function: mdlSetWorkWidths =============================================
 * Abstract:
 *   The optional method, mdlSetWorkWidths is called after input port
 *   width, output port width, and sample times of the S-function have
 *   been determined to set any state and work vector sizes which are
 *   a function of the input, output, and/or sample times. 
 *   Run-time parameters are registered in this method using methods 
 *   ssSetNumRunTimeParams, ssSetRunTimeParamInfo, and related methods.
 */
static void mdlSetWorkWidths(SimStruct *S)
{
}
#endif

#define MDL_START
#if defined(MDL_START)
/* Function: mdlStart =====================================================
 * Abstract:
 *   This function is called once at start of model execution. If you
 *   have states that should be initialized once, this is the place
 *   to do it.
 */
static void mdlStart(SimStruct *S)
{
    if (IS_SIMULATION_TARGET(S)) {

        /* Access bus/struct information */
        int32_T* __dtSizeInfo = (int32_T*) ssGetDWork(S, 0);
        int32_T* __dtBusInfo = (int32_T*) ssGetDWork(S, 1);


        /* Get common data type Id */
        DTypeId __ACADOoutputId = ssGetDataTypeId(S, "ACADOoutput");
        DTypeId __int32Id = ssGetDataTypeId(S, "int32");
        DTypeId __doubleId = ssGetDataTypeId(S, "double");
        DTypeId __ACADOdataId = ssGetDataTypeId(S, "ACADOdata");
        DTypeId __ACADOinputId = ssGetDataTypeId(S, "ACADOinput");

        /* Get common data type size */
        __dtSizeInfo[0] = ssGetDataTypeSize(S, __ACADOoutputId);
        __dtSizeInfo[1] = ssGetDataTypeSize(S, __int32Id);
        __dtSizeInfo[2] = ssGetDataTypeSize(S, __doubleId);
        __dtSizeInfo[3] = ssGetDataTypeSize(S, __ACADOdataId);
        __dtSizeInfo[4] = ssGetDataTypeSize(S, __ACADOinputId);

        /* Get information for accessing ACADOoutput.status */
        __dtBusInfo[0] = ssGetBusElementOffset(S, __ACADOoutputId, 0);
        __dtBusInfo[1] = __dtSizeInfo[1];

        /* Get information for accessing ACADOoutput.kktValue */
        __dtBusInfo[2] = ssGetBusElementOffset(S, __ACADOoutputId, 1);
        __dtBusInfo[3] = __dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.objValue */
        __dtBusInfo[4] = ssGetBusElementOffset(S, __ACADOoutputId, 2);
        __dtBusInfo[5] = __dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.execTime */
        __dtBusInfo[6] = ssGetBusElementOffset(S, __ACADOoutputId, 3);
        __dtBusInfo[7] = __dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.data.x */
        __dtBusInfo[8] = ssGetBusElementOffset(S, __ACADOoutputId, 4) + ssGetBusElementOffset(S, __ACADOdataId, 0);
        __dtBusInfo[9] = 123*__dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.data.u */
        __dtBusInfo[10] = ssGetBusElementOffset(S, __ACADOoutputId, 4) + ssGetBusElementOffset(S, __ACADOdataId, 1);
        __dtBusInfo[11] = 160*__dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.data.od */
        __dtBusInfo[12] = ssGetBusElementOffset(S, __ACADOoutputId, 4) + ssGetBusElementOffset(S, __ACADOdataId, 2);
        __dtBusInfo[13] = 82*__dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.data.y */
        __dtBusInfo[14] = ssGetBusElementOffset(S, __ACADOoutputId, 4) + ssGetBusElementOffset(S, __ACADOdataId, 3);
        __dtBusInfo[15] = 280*__dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.data.yN */
        __dtBusInfo[16] = ssGetBusElementOffset(S, __ACADOoutputId, 4) + ssGetBusElementOffset(S, __ACADOdataId, 4);
        __dtBusInfo[17] = 3*__dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.data.W */
        __dtBusInfo[18] = ssGetBusElementOffset(S, __ACADOoutputId, 4) + ssGetBusElementOffset(S, __ACADOdataId, 5);
        __dtBusInfo[19] = 49*__dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.data.WN */
        __dtBusInfo[20] = ssGetBusElementOffset(S, __ACADOoutputId, 4) + ssGetBusElementOffset(S, __ACADOdataId, 6);
        __dtBusInfo[21] = 9*__dtSizeInfo[2];

        /* Get information for accessing ACADOoutput.data.x0 */
        __dtBusInfo[22] = ssGetBusElementOffset(S, __ACADOoutputId, 4) + ssGetBusElementOffset(S, __ACADOdataId, 7);
        __dtBusInfo[23] = 3*__dtSizeInfo[2];

        /* Get information for accessing ACADOinput.control */
        __dtBusInfo[24] = ssGetBusElementOffset(S, __ACADOinputId, 0);
        __dtBusInfo[25] = __dtSizeInfo[1];

        /* Get information for accessing ACADOinput.shifting */
        __dtBusInfo[26] = ssGetBusElementOffset(S, __ACADOinputId, 1);
        __dtBusInfo[27] = __dtSizeInfo[1];

        /* Get information for accessing ACADOinput.initialization */
        __dtBusInfo[28] = ssGetBusElementOffset(S, __ACADOinputId, 2);
        __dtBusInfo[29] = __dtSizeInfo[1];

        /* Get information for accessing ACADOinput.data.x */
        __dtBusInfo[30] = ssGetBusElementOffset(S, __ACADOinputId, 3) + ssGetBusElementOffset(S, __ACADOdataId, 0);
        __dtBusInfo[31] = 123*__dtSizeInfo[2];

        /* Get information for accessing ACADOinput.data.u */
        __dtBusInfo[32] = ssGetBusElementOffset(S, __ACADOinputId, 3) + ssGetBusElementOffset(S, __ACADOdataId, 1);
        __dtBusInfo[33] = 160*__dtSizeInfo[2];

        /* Get information for accessing ACADOinput.data.od */
        __dtBusInfo[34] = ssGetBusElementOffset(S, __ACADOinputId, 3) + ssGetBusElementOffset(S, __ACADOdataId, 2);
        __dtBusInfo[35] = 82*__dtSizeInfo[2];

        /* Get information for accessing ACADOinput.data.y */
        __dtBusInfo[36] = ssGetBusElementOffset(S, __ACADOinputId, 3) + ssGetBusElementOffset(S, __ACADOdataId, 3);
        __dtBusInfo[37] = 280*__dtSizeInfo[2];

        /* Get information for accessing ACADOinput.data.yN */
        __dtBusInfo[38] = ssGetBusElementOffset(S, __ACADOinputId, 3) + ssGetBusElementOffset(S, __ACADOdataId, 4);
        __dtBusInfo[39] = 3*__dtSizeInfo[2];

        /* Get information for accessing ACADOinput.data.W */
        __dtBusInfo[40] = ssGetBusElementOffset(S, __ACADOinputId, 3) + ssGetBusElementOffset(S, __ACADOdataId, 5);
        __dtBusInfo[41] = 49*__dtSizeInfo[2];

        /* Get information for accessing ACADOinput.data.WN */
        __dtBusInfo[42] = ssGetBusElementOffset(S, __ACADOinputId, 3) + ssGetBusElementOffset(S, __ACADOdataId, 6);
        __dtBusInfo[43] = 9*__dtSizeInfo[2];

        /* Get information for accessing ACADOinput.data.x0 */
        __dtBusInfo[44] = ssGetBusElementOffset(S, __ACADOinputId, 3) + ssGetBusElementOffset(S, __ACADOdataId, 7);
        __dtBusInfo[45] = 3*__dtSizeInfo[2];


        /* Alloc memory for the pwork 1 (__y1BUS) */
        {
            ACADOoutput* __y1BUS = (ACADOoutput*)calloc(sizeof(ACADOoutput), ssGetOutputPortWidth(S, 0));
            if (__y1BUS==NULL) {
                ssSetErrorStatus(S, "Unexpected error during the memory allocation for __y1BUS");
                return;
            } /* if */
            ssSetPWorkValue(S, 0, __y1BUS);
        }

        /* Alloc memory for the pwork 2 (__u1BUS) */
        {
            ACADOinput* __u1BUS = (ACADOinput*)calloc(sizeof(ACADOinput), ssGetInputPortWidth(S, 0));
            if (__u1BUS==NULL) {
                ssSetErrorStatus(S, "Unexpected error during the memory allocation for __u1BUS");
                return;
            } /* if */
            ssSetPWorkValue(S, 1, __u1BUS);
        }

        {

            /* Call the legacy code function */
            acado_initialize();
        }
    } /* if */
}
#endif

/* Function: mdlOutputs ===================================================
 * Abstract:
 *   In this function, you compute the outputs of your S-function
 *   block. Generally outputs are placed in the output vector(s),
 *   ssGetOutputPortSignal.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    if (IS_SIMULATION_TARGET(S)) {

        /* Access bus/struct information */
        int32_T* __dtSizeInfo = (int32_T*) ssGetDWork(S, 0);
        int32_T* __dtBusInfo = (int32_T*) ssGetDWork(S, 1);


        /* Get access to Parameter/Input/Output/DWork data */
        char* u1 = (char*) ssGetInputPortSignal(S, 0);
        char* y1 = (char*) ssGetOutputPortSignal(S, 0);

        ACADOinput* __u1BUS = (ACADOinput*) ssGetPWorkValue(S, 1);
        ACADOoutput* __y1BUS = (ACADOoutput*) ssGetPWorkValue(S, 0);

        /* Assign the Simulink structure u1 to user structure __u1BUS */
        (void) memcpy(&__u1BUS[0].control, u1 +  __dtBusInfo[24],  __dtBusInfo[25]);
        (void) memcpy(&__u1BUS[0].shifting, u1 +  __dtBusInfo[26],  __dtBusInfo[27]);
        (void) memcpy(&__u1BUS[0].initialization, u1 +  __dtBusInfo[28],  __dtBusInfo[29]);
        (void) memcpy(__u1BUS[0].data.x, u1 +  __dtBusInfo[30],  __dtBusInfo[31]);
        (void) memcpy(__u1BUS[0].data.u, u1 +  __dtBusInfo[32],  __dtBusInfo[33]);
        (void) memcpy(__u1BUS[0].data.od, u1 +  __dtBusInfo[34],  __dtBusInfo[35]);
        (void) memcpy(__u1BUS[0].data.y, u1 +  __dtBusInfo[36],  __dtBusInfo[37]);
        (void) memcpy(__u1BUS[0].data.yN, u1 +  __dtBusInfo[38],  __dtBusInfo[39]);
        (void) memcpy(__u1BUS[0].data.W, u1 +  __dtBusInfo[40],  __dtBusInfo[41]);
        (void) memcpy(__u1BUS[0].data.WN, u1 +  __dtBusInfo[42],  __dtBusInfo[43]);
        (void) memcpy(__u1BUS[0].data.x0, u1 +  __dtBusInfo[44],  __dtBusInfo[45]);

        /* Call the legacy code function */
        acado_step(__u1BUS, __y1BUS);

        /* Assign the user structure __y1BUS to the Simulink structure y1 */
        (void) memcpy(y1 +  __dtBusInfo[0], &__y1BUS[0].status,  __dtBusInfo[1]);
        (void) memcpy(y1 +  __dtBusInfo[2], &__y1BUS[0].kktValue,  __dtBusInfo[3]);
        (void) memcpy(y1 +  __dtBusInfo[4], &__y1BUS[0].objValue,  __dtBusInfo[5]);
        (void) memcpy(y1 +  __dtBusInfo[6], &__y1BUS[0].execTime,  __dtBusInfo[7]);
        (void) memcpy(y1 +  __dtBusInfo[8], __y1BUS[0].data.x,  __dtBusInfo[9]);
        (void) memcpy(y1 +  __dtBusInfo[10], __y1BUS[0].data.u,  __dtBusInfo[11]);
        (void) memcpy(y1 +  __dtBusInfo[12], __y1BUS[0].data.od,  __dtBusInfo[13]);
        (void) memcpy(y1 +  __dtBusInfo[14], __y1BUS[0].data.y,  __dtBusInfo[15]);
        (void) memcpy(y1 +  __dtBusInfo[16], __y1BUS[0].data.yN,  __dtBusInfo[17]);
        (void) memcpy(y1 +  __dtBusInfo[18], __y1BUS[0].data.W,  __dtBusInfo[19]);
        (void) memcpy(y1 +  __dtBusInfo[20], __y1BUS[0].data.WN,  __dtBusInfo[21]);
        (void) memcpy(y1 +  __dtBusInfo[22], __y1BUS[0].data.x0,  __dtBusInfo[23]);
    } /* if */
}

/* Function: mdlTerminate =================================================
 * Abstract:
 *   In this function, you should perform any actions that are necessary
 *   at the termination of a simulation.
 */
static void mdlTerminate(SimStruct *S)
{
    if (IS_SIMULATION_TARGET(S)) {

        /* Access bus/struct information */
        int32_T* __dtSizeInfo = (int32_T*) ssGetDWork(S, 0);
        int32_T* __dtBusInfo = (int32_T*) ssGetDWork(S, 1);

        {

            /* Call the legacy code function */
            acado_cleanup();
        }

        /* Free memory for the pwork 1 (__y1BUS) */
        {
            ACADOoutput* __y1BUS = (ACADOoutput*)ssGetPWorkValue(S, 0);
            if (__y1BUS!=NULL) {
                free(__y1BUS);
                ssSetPWorkValue(S, 0, NULL);
            } /* if */
        }

        /* Free memory for the pwork 2 (__u1BUS) */
        {
            ACADOinput* __u1BUS = (ACADOinput*)ssGetPWorkValue(S, 1);
            if (__u1BUS!=NULL) {
                free(__u1BUS);
                ssSetPWorkValue(S, 1, NULL);
            } /* if */
        }

    } /* if */
}

/* Function: CheckDataTypeChecksum ========================================
 * Abstract:
 *   CheckDataTypeChecksum invokes a MATLAB helper for checking the consistency
 *   between the data type definition used when this S-Function was generated
 *   and the data type used when calling the S-Function.
 */
static int_T CheckDataTypeChecksum(SimStruct *S, const char* dtypeName, uint32_T* chkRef)
{
    mxArray *plhs[1] = {NULL};
    mxArray *prhs[3];
    mxArray *err = NULL;
    const char *bpath = ssGetPath(S);
    int_T status = -1;

    prhs[0] = mxCreateString(bpath);
    prhs[1] = mxCreateString(dtypeName);
    prhs[2] = mxCreateDoubleMatrix(1, 4, mxREAL);
    mxGetPr(prhs[2])[0] = chkRef[0];
    mxGetPr(prhs[2])[1] = chkRef[1];
    mxGetPr(prhs[2])[2] = chkRef[2];
    mxGetPr(prhs[2])[3] = chkRef[3];

    err = mexCallMATLABWithTrap(1, plhs, 3, prhs, "legacycode.LCT.getOrCompareDataTypeChecksum");
    mxDestroyArray(prhs[0]);
    mxDestroyArray(prhs[1]);
    mxDestroyArray(prhs[2]);

    if (err==NULL && plhs[0]!=NULL) {
        status = mxIsEmpty(plhs[0]) ? -1 : (int_T) (mxGetScalar(plhs[0]) != 0);
        mxDestroyArray(plhs[0]);
    } /* if */

    return status;
}

/* Function: CheckDataTypes ===============================================
 * Abstract:
 *   CheckDataTypes verifies data type consistency between the data type 
 *   definition used when this S-Function was generated and the data type
 *   used when calling the S-Function.
 */
static void CheckDataTypes(SimStruct *S)
{

    /* Verify Bus/StructType 'ACADOoutput', specification */
    {
        uint32_T chk[] = {4082626726, 1948146466, 3056144785, 1797633266};
        int_T status;
        status = CheckDataTypeChecksum(S, "ACADOoutput", &chk[0]);
        if (status==-1) {
          ssSetErrorStatus(S, "Unexpected error when checking the validity of the Simulink Bus/StructType 'ACADOoutput'");
        } else if (status==0) {
          ssSetErrorStatus(S, "The Simulink Bus/StructType 'ACADOoutput' definition has changed since the S-Function was generated");
        }
    }

    /* Verify Bus/StructType 'ACADOdata', specification */
    {
        uint32_T chk[] = {1143753743, 1670352161, 3802876698, 1197366725};
        int_T status;
        status = CheckDataTypeChecksum(S, "ACADOdata", &chk[0]);
        if (status==-1) {
          ssSetErrorStatus(S, "Unexpected error when checking the validity of the Simulink Bus/StructType 'ACADOdata'");
        } else if (status==0) {
          ssSetErrorStatus(S, "The Simulink Bus/StructType 'ACADOdata' definition has changed since the S-Function was generated");
        }
    }

    /* Verify Bus/StructType 'ACADOinput', specification */
    {
        uint32_T chk[] = {276710772, 734126488, 3383850239, 2512823045};
        int_T status;
        status = CheckDataTypeChecksum(S, "ACADOinput", &chk[0]);
        if (status==-1) {
          ssSetErrorStatus(S, "Unexpected error when checking the validity of the Simulink Bus/StructType 'ACADOinput'");
        } else if (status==0) {
          ssSetErrorStatus(S, "The Simulink Bus/StructType 'ACADOinput' definition has changed since the S-Function was generated");
        }
    }
}

/* Function: GetRTWEnvironmentMode ========================================
 * Abstract:
 *   Must be called when ssRTWGenIsCodeGen(S)==true. This function
 *   returns the code generation mode:
 *       -1 if an error occurred
 *        0 for standalone code generation target
 *        1 for simulation target (Accelerator, RTW-SFcn,...)
 */
static int_T GetRTWEnvironmentMode(SimStruct *S)
{
    int_T status;
    mxArray * err;
    mxArray *plhs[1];
    mxArray *prhs[1];

    /* Get the name of the Simulink block diagram */
    prhs[0] = mxCreateString(ssGetBlockDiagramName(S));
    plhs[0] = NULL;

    /* Call "isSimulationTarget = rtwenvironmentmode(modelName)" in MATLAB */
    err = mexCallMATLABWithTrap(1, plhs, 1, prhs, "rtwenvironmentmode");

    mxDestroyArray(prhs[0]);

    /* Set the error status if an error occurred */
    if (err) {
        if (plhs[0]) {
            mxDestroyArray(plhs[0]);
            plhs[0] = NULL;
        } /* if: } */
        ssSetErrorStatus(S, "Unknown error during call to 'rtwenvironmentmode'.");
        return -1;
    } /* if */

    /* Get the value returned by rtwenvironmentmode(modelName) */
    if (plhs[0]) {
        status = (int_T) (mxGetScalar(plhs[0]) != 0);
        mxDestroyArray(plhs[0]);
        plhs[0] = NULL;
    } /* if */

    return status;
}

/* Required S-function trailer */
#ifdef    MATLAB_MEX_FILE
# include "simulink.c"
#else
# include "cg_sfun.h"
#endif

