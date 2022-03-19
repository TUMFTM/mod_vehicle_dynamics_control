/* Defines and includes */
#define S_FUNCTION_NAME  osqp_sfun  // Define function name
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "osqp_wrapper.h"

enum {PARAM = 0, PARAM_1, PARAM_2, PARAM_3, PARAM_4, PARAM_5, PARAM_6, PARAM_7, PARAM_8, PARAM_9, PARAM_10, PARAM_11, PARAM_12, NUM_PARAMS}; // define three macros: PARAM = 0, PARAM_1 = 1,.., NUM_PARMAS = 13
#define PARAM_ARG ssGetSFcnParam(S, PARAM)  // returns value of the S-funciton parameter with the index PARAM=0
#define PARAM_1_ARG ssGetSFcnParam(S, PARAM_1)  // returns value of the S-funciton parameter with the index PARAM_1=1
#define PARAM_2_ARG ssGetSFcnParam(S, PARAM_2)  // returns value of the S-funciton parameter with the index PARAM_2=2
#define PARAM_3_ARG ssGetSFcnParam(S, PARAM_3)  // returns value of the S-funciton parameter with the index PARAM_3=3
#define PARAM_4_ARG ssGetSFcnParam(S, PARAM_4)  // returns value of the S-funciton parameter with the index PARAM_4=4
#define PARAM_5_ARG ssGetSFcnParam(S, PARAM_5)  // returns value of the S-funciton parameter with the index PARAM_5=5
#define PARAM_6_ARG ssGetSFcnParam(S, PARAM_6)  // returns value of the S-funciton parameter with the index PARAM_6=6
#define PARAM_7_ARG ssGetSFcnParam(S, PARAM_7)  // returns value of the S-funciton parameter with the index PARAM_7=7
#define PARAM_8_ARG ssGetSFcnParam(S, PARAM_8)  // returns value of the S-funciton parameter with the index PARAM_8=8
#define PARAM_9_ARG ssGetSFcnParam(S, PARAM_9)  // returns value of the S-funciton parameter with the index PARAM_9=9
#define PARAM_10_ARG ssGetSFcnParam(S, PARAM_10)  // returns value of the S-funciton parameter with the index PARAM_10=10
#define PARAM_11_ARG ssGetSFcnParam(S, PARAM_11)  // returns value of the S-funciton parameter with the index PARAM_11=11
#define PARAM_12_ARG ssGetSFcnParam(S, PARAM_12)  // returns value of the S-funciton parameter with the index PARAM_12=12

// for compiling include osqp library --> use: mex CFLAGS='$CFLAGS -w' misc/osqp/osqp_sfun.c misc/osqp/osqp_wrapper.c -Lmisc/osqp/lib/ -losqp_mingw -outdir build -silent -DWINDOWS_BUILD
// include "misc/osqp/lib/libosqp_mingw.lib" in the code generation settings of Simulink

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    // set number of expected s-function parameters to NUM_PARAMS(=14)
    ssSetNumSFcnParams(S, NUM_PARAMS);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }
    // S-Function Inputs:
    // u0Ptr => pointer to q_upd
    // u1Ptr => pointer to l_upd
    // u2Ptr => pointer to u_upd
    // u3Ptr => pointer to P_x
    // u4Ptr => pointer to A_x

    // S-function parameters:
    // *p_0 => q = linear cost vector
    // *p_1 => m = number of constraints
    // *p_2 => n = number of optimization variables
    // *p_3 => P_x = values of UPPER TRIANGULAR of P
    // *p_4 => P_i = corresponding row indices of values
    // *p_5 => P_p = number of nonzero entries in each column of P
    // *p_6 => A_x = values of FULL matrix A
    // *p_7 => A_i = corresponding row indices of values
    // *p_8 => A_p = number of nonzero entries in each column of A
    // *p_9 => l = lower bound vector
    // *p_10 => u = upper bound vector
    // *p_11 => trigger_print = activate debugging code snippets (e.g. via ssPrintf)
	// *p_12 => max_iter = number of allowed solver iterations

    const real_T      *p_1     = mxGetPr(PARAM_1_ARG);  // pointer to the first element of data (PARAM_1_ARG)
    const real_T      *p_2     = mxGetPr(PARAM_2_ARG);  // pointer to the first element of data (PARAM_2_ARG)
    const real_T      *p_5     = mxGetPr(PARAM_5_ARG);  // pointer to the first element of data (PARAM_5_ARG)
    const real_T      *p_8     = mxGetPr(PARAM_8_ARG);  // pointer to the first element of data (PARAM_8_ARG)
    const real_T      *p_11     = mxGetPr(PARAM_11_ARG);  // pointer to the first element of data (PARAM_11_ARG)
    size_t            p1Width = mxGetNumberOfElements(PARAM_1_ARG);    // get number of elements in numeric array of the block paramters

    // read problem size
    c_int n = p_2[0];

    /* Allow signal dimensions greater than 2 */
    ssAllowSignalsWithMoreThan2D(S);

    // create 5 input port (indexing begins with 0..4)
    if (!ssSetNumInputPorts(S, 5)) return;

    // settings for input port 1 --> q
    ssSetInputPortWidth(S, 0, p_2[0]);
    ssSetInputPortDirectFeedThrough(S, 0, 1);   // Feedthrough = 1 (true) is mandatory to enable usage of input in mdlOutputs

    // settings for input port 2 --> l
    ssSetInputPortWidth(S, 1, p_1[0]);
    ssSetInputPortDirectFeedThrough(S, 1, 1);   // Feedthrough = 1 (true) is mandatory to enable usage of input in mdlOutputs

    // settings for input port 3 --> u
    ssSetInputPortWidth(S, 2, p_1[0]);
    ssSetInputPortDirectFeedThrough(S, 2, 1);   // Feedthrough = 1 (true) is mandatory to enable usage of input in mdlOutputs

    // settings for input port 4 --> A or A_x
    ssSetInputPortWidth(S, 3, p_8[n]);
    ssSetInputPortDirectFeedThrough(S, 3, 1);   // Feedthrough = 1 (true) is mandatory to enable usage of input in mdlOutputs
    
    // settings for input port 5 --> reset osqp
    ssSetInputPortWidth(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);   // Feedthrough = 1 (true) is mandatory to enable usage of input in mdlOutputs

    // create 20 output ports
    if (!ssSetNumOutputPorts(S,20)) return;
    ssSetOutputPortWidth(S, 0, p_2[0]);  // 1st output port for (whole) solution: an array with n entries
    ssSetOutputPortWidth(S, 1, p_1[0]);  // 2nd output port for (whole) dual solution: an array with m entries
    ssSetOutputPortWidth(S, 2, 1);  // 3rd output port for solve_time: a scalar
    ssSetOutputPortWidth(S, 3, 1);  // 4th output port for run_time: a scalar
    ssSetOutputPortWidth(S, 4, 1);  // 5th output port for solver status: a scalar 
    ssSetOutputPortWidth(S, 5, 1);  // 6th output port for number of iterations: a scalar 
    ssSetOutputPortWidth(S, 6, 1);  // 7th output port for update_time: a scalar
    ssSetOutputPortWidth(S, 7, 1);  // 8th output port for total_sfun_time: a scalar
    ssSetOutputPortWidth(S, 8, 1);  // 9th output port for primal residual: a scalar
    ssSetOutputPortWidth(S, 9, 1);  // 10th output port for dual residual: a scalar
    ssSetOutputPortWidth(S, 10, p_1[0]);  // 17th output port for (whole) z: an array with m entries
    ssSetOutputPortWidth(S, 11, 1);  // 11th output port for exitflag bound update: a scalar
    ssSetOutputPortWidth(S, 12, 1);  // 12th output port for exitflag A update: a scalar
    ssSetOutputPortWidth(S, 13, 1);  // 13th output port for exitflag P update: a scalar
    ssSetOutputPortWidth(S, 14, 1);  // 14th output port for exitflag q update: a scalar
    ssSetOutputPortWidth(S, 15, 1);  // 15th output port for exitflag solve: a scalar
    ssSetOutputPortWidth(S, 16, 1);  // 16th output port for exitflag setup: a scalar
    ssSetOutputPortWidth(S, 17, 1);  // 18th output port for cost function scaling factor c: a scalar
    ssSetOutputPortWidth(S, 18, p_2[0]);  // 19th output port for diagonal scaling matrix D: an array with n entries
    ssSetOutputPortWidth(S, 19, p_1[0]);  // 20th output port for diagonal scaling matrix E: an array with m entries

    // set number of pointer work vectors to 1
    ssSetNumPWork(S, 1);

    ssSetNumSampleTimes(S, 1);

    /* Take care when specifying exception free code - see sfuntmpl.doc */
    // Specify applicable options of S-function -> always at the end of mdlInitializeSizes
    ssSetOptions(S, SS_OPTION_WORKS_WITH_CODE_REUSE | SS_OPTION_EXCEPTION_FREE_CODE);
 } /* end mdlInitializeSizes */

/* Function: mdlStart ===============================================
 * Abstract:
 *   Setup workspace and solver settings.
 */
#define MDL_START   // activate mdlStart since it is optional
static void mdlStart(SimStruct *S)
{
    // access input signals and parameter
    InputRealPtrsType u0Ptr   = ssGetInputPortRealSignalPtrs(S,0);   // get pointer to s-function block input (here: port 0) of type double
    const real_T      *p_0     = mxGetPr(PARAM_ARG);  // pointer to the first element of "q_par" as an S-Function parameter
    const real_T      *p_1     = mxGetPr(PARAM_1_ARG);  // pointer to the first element of "m" as an S-Function parameter
    const real_T      *p_2     = mxGetPr(PARAM_2_ARG);  // pointer to the first element of "n" as an S-Function parameter
    const real_T      *p_3     = mxGetPr(PARAM_3_ARG);  // pointer to the first element of "P_x" as an S-Function parameter
    const real_T      *p_4     = mxGetPr(PARAM_4_ARG);  // pointer to the first element of "P_i" as an S-Function parameter
    const real_T      *p_5     = mxGetPr(PARAM_5_ARG);  // pointer to the first element of "P_p" as an S-Function parameter
    const real_T      *p_6     = mxGetPr(PARAM_6_ARG);  // pointer to the first element of "A_x" as an S-Function parameter
    const real_T      *p_7     = mxGetPr(PARAM_7_ARG);  // pointer to the first element of "A_i" as an S-Function parameter
    const real_T      *p_8     = mxGetPr(PARAM_8_ARG);  // pointer to the first element of "A_p" as an S-Function parameter
    const real_T      *p_9     = mxGetPr(PARAM_9_ARG);  // pointer to the first element of "l_par" as an S-Function parameter
    const real_T      *p_10     = mxGetPr(PARAM_10_ARG);  // pointer to the first element of "u_par" as an S-Function parameter
    const real_T      *p_11     = mxGetPr(PARAM_11_ARG);  // pointer to the first element of "trigger_print" as an S-Function parameter
    const real_T      *p_12     = mxGetPr(PARAM_12_ARG);  // pointer to the first element of "max_iter" as an S-Function parameter
    size_t            pWidth = mxGetNumberOfElements(PARAM_ARG);    // get number of elements in numeric array of the block paramters
    size_t            p6Width = mxGetNumberOfElements(PARAM_6_ARG);    // get number of elements in numeric array of the block paramters

    // initialize OSQP wrapper
    osqp_wrapper *wrapper;
    wrapper = (osqp_wrapper *)c_malloc(sizeof(osqp_wrapper));
    init_osqp_wrapper(wrapper, p_0, p_1, p_2, p_3, p_4, p_5, p_6, p_7, p_8, p_9, p_10, p_11, p_12);

    // Copy workspace structure to Pwork vector
    ssSetPWorkValue(S, 0, wrapper);
} /* end mdlStart */

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
} /* end mdlInitializeSampleTimes */

/* Function: mdlOutputs =======================================================
 solve updated QP using osqp-package. output exitflags of used functions, the optimal solution sequence,
 * the necessary solve/update/run time and the number of iterations needed.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // access output signals
    real_T *y = ssGetOutputPortRealSignal(S,0);     // output for (whole) primal solution
    real_T *y_1 = ssGetOutputPortRealSignal(S,1);   // output for for (whole) dual solution
    real_T *y_2 = ssGetOutputPortRealSignal(S,2);   // output for solve_time
    real_T *y_3 = ssGetOutputPortRealSignal(S,3);   // output for run_time
    real_T *y_4 = ssGetOutputPortRealSignal(S,4);   // output for solver status
    real_T *y_5 = ssGetOutputPortRealSignal(S,5);   // output for number of iterations
    real_T *y_6 = ssGetOutputPortRealSignal(S,6);   // output for update_time
    real_T *y_7 = ssGetOutputPortRealSignal(S,7);   // output for total_sfun_time
    real_T *y_8 = ssGetOutputPortRealSignal(S,8);   // output for for primal residual
    real_T *y_9 = ssGetOutputPortRealSignal(S,9);   // output for for dual residual
    real_T *y_10 = ssGetOutputPortRealSignal(S,10);   // output for for (whole) z
    real_T *y_11 = ssGetOutputPortRealSignal(S,11);   // output for exitflag of bound update
    real_T *y_12 = ssGetOutputPortRealSignal(S,12);   // output for exitflag of A update
    real_T *y_13 = ssGetOutputPortRealSignal(S,13);   // output for exitflag of P update
    real_T *y_14 = ssGetOutputPortRealSignal(S,14);   // output for exitflag of q update
    real_T *y_15 = ssGetOutputPortRealSignal(S,15);   // output for exitflag of solve
    real_T *y_16 = ssGetOutputPortRealSignal(S,16);   // output for exitflag of setup in mdlStart
    real_T *y_17 = ssGetOutputPortRealSignal(S,17);   // output for for cost function scaling factor c
    real_T *y_18 = ssGetOutputPortRealSignal(S,18);   // output for for diagonal scaling matrix D
    real_T *y_19 = ssGetOutputPortRealSignal(S,19);   // output for for diagonal scaling matrix E

    // access input signals and parameter
    InputRealPtrsType u0Ptr   = ssGetInputPortRealSignalPtrs(S,0);   // get pointer to q_upd
    InputRealPtrsType u1Ptr   = ssGetInputPortRealSignalPtrs(S,1);   // get pointer to l_upd
    InputRealPtrsType u2Ptr   = ssGetInputPortRealSignalPtrs(S,2);   // get pointer to u_upd
    InputRealPtrsType u3Ptr   = ssGetInputPortRealSignalPtrs(S,3);   // get pointer to A_x
    InputRealPtrsType u4Ptr   = ssGetInputPortRealSignalPtrs(S,4);   // get pointer to reset bit
    const real_T      *p     = mxGetPr(PARAM_ARG);  // pointer to the first element of q_par

    // get corresponding signal/parameter widths
    int_T             u0Width = ssGetInputPortWidth(S,0);    // get the number of elements (width) of q_upd
    int_T             u1Width = ssGetInputPortWidth(S,1);    // get the number of elements (width) of l_upd
    int_T             u2Width = ssGetInputPortWidth(S,2);    // get the number of elements (width) of u_upd
    int_T             u3Width = ssGetInputPortWidth(S,3);    // get the number of elements (width) of P_x
    int_T             u4Width = ssGetInputPortWidth(S,4);    // get the number of elements (width) of A_x
    size_t            pWidth = mxGetNumberOfElements(PARAM_ARG);    // get number of elements of q_par
    int_T             yWidth = ssGetOutputPortWidth(S,0);   // get the number of elements (width) in the output "primal solution"
    int_T             y1Width = ssGetOutputPortWidth(S,1);   // get the number of elements (width) in the output "dual solution"
    int_T             y10Width = ssGetOutputPortWidth(S,10);   // get the number of elements (width) in the output "z"
    int_T             y18Width = ssGetOutputPortWidth(S,18);   // get the number of elements (width) in the output "D"
    int_T             y19Width = ssGetOutputPortWidth(S,19);   // get the number of elements (width) in the output "E"

    // update problem
    osqp_wrapper *wrapper;
    wrapper = ssGetPWorkValue(S, 0); // Get element of the block's pointer work vector at specified index(here: 0)
    
    // if the reset bit is set, use restart with zeros
    if(*u4Ptr[0] == 1)
    {
        restart_osqp_wrapper(wrapper);
    }
    // do the full update
    update_osqp_wrapper(wrapper, *u0Ptr, *u1Ptr, *u2Ptr, *u3Ptr);

    for(int i = 0; i < yWidth; i++){
        y[i] = wrapper->work->solution->x[i];
    }    
    for(int i = 0; i < y1Width; i++){
        y_1[i] = wrapper->work->solution->y[i];
    }
    for(int i = 0; i < y10Width; i++){
        y_10[i] = wrapper->work->Ax[i] - wrapper->work->z[i];
    }
    if (wrapper->work->settings->scaling){
        *y_17 = wrapper->work->scaling->c;
    }else{
        *y_17 = 1;
    }
    for(int i = 0; i < y18Width; i++){
        if (wrapper->work->settings->scaling){
            y_18[i] = wrapper->work->scaling->D[i];
        }else{
            y_18[i] = 1;
        }
    }
    for(int i = 0; i < y19Width; i++){
        if (wrapper->work->settings->scaling){
            y_19[i] = wrapper->work->scaling->E[i];
        }else{
            y_19[i] = 1;
        } 
    }

    *y_2 = wrapper->work->info->solve_time;
    *y_3 = wrapper->work->info->run_time;
    *y_4 = wrapper->work->info->status_val;  // see "constant.h" for meaning of status values
    *y_5 = wrapper->work->info->iter;
    *y_6 = wrapper->work->info->update_time;
    *y_8 = wrapper->work->info->pri_res;
    *y_9 = wrapper->work->info->dua_res;
    *y_11 = wrapper->flag_bound_upd;
    *y_12 = wrapper->flag_A_upd;
    *y_13 = wrapper->flag_P_upd;
    *y_14 = wrapper->flag_q_upd;
    *y_15 = wrapper->flag_solve;
    *y_16 = wrapper->flag_setup;

} /* end mdlOutputs */


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    // free memory of osqp wrapper internally
    cleanup_osqp_wrapper(ssGetPWorkValue(S, 0));
    // cleanup osqp wrapper itself
    free(ssGetPWorkValue(S, 0)); 
}

// Attach S-function to Simulink --> Trailer is mandatory
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
