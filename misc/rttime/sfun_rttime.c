/* SIMULINK BLOCK FOR REAL-TIME EXECUTION
 *
 * compile command for Windows (needs windows SDK)
 * mex -O sfun_rttime.c
 %
 * compile command for Linux 
 * mex -O sfun_rttime.c -lrt
 *
 * Ivo Houtzager
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  sfun_rttime

#define TIME_SCALE_FACTOR(S) ssGetSFcnParam(S,0)

/* Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions. */
#include <simstruc.h>

#if defined(_WIN32)
/* Include the windows SDK header for handling time functions. */
#include <windows.h>
#include <math.h>

/* Function of the high performance counter (in seconds). */
__inline double hightimer()
{
    HANDLE hCurrentProcess = GetCurrentProcess();
    DWORD dwProcessAffinity;
    DWORD dwSystemAffinity;    
    LARGE_INTEGER frequency, counter;
    double sec_per_tick, total_ticks;
    
    /* force thread on first cpu */
    GetProcessAffinityMask(hCurrentProcess,&dwProcessAffinity,&dwSystemAffinity);
    SetProcessAffinityMask(hCurrentProcess, 1);
    
    /* retrieves the frequency of the high-resolution performance counter */
    QueryPerformanceFrequency(&frequency);
    
    /* retrieves the current value of the high-resolution performance counter */
    QueryPerformanceCounter(&counter);
    
     /* reset thread */
    SetProcessAffinityMask(hCurrentProcess,dwProcessAffinity);

    /* time in seconds */
    sec_per_tick = (double)1/(double)frequency.QuadPart;
    total_ticks = (double)counter.QuadPart;    
    return sec_per_tick*total_ticks;
}   /* end hightimer */
#else
/* Include the standard ANSI C header for handling time functions. */
#if !defined(_POSIX_C_SOURCE) || _POSIX_C_SOURCE < 199309L
#define _POSIX_C_SOURCE 199309L
#endif
#include <time.h>

/* Function of the high performance counter (in seconds). */
__inline double hightimer()
{    
    struct timespec time;
    
    /* retrieves the monotonic time */
    clock_gettime(CLOCK_MONOTONIC, &time); 

    /* time in seconds */
    /* tv_sec is long with max 10 digits, 
       gives remainder 6 digits of double for tv_nsec */
    return ( (double)time.tv_sec + 1e-9*(double)time.tv_nsec ); 
}   /* end hightimer */
#endif

static void mdlInitializeSizes(SimStruct *S)
{
   ssSetNumSFcnParams(S, 1);  /* Number of expected parameters */
   if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);
   if (!ssSetNumInputPorts(S, 0)) return;
   if (!ssSetNumOutputPorts(S, 1)) return;
   ssSetOutputPortWidth(S, 0, 1);
   ssSetNumSampleTimes(S, 1);
   ssSetNumRWork(S, 1);
   ssSetNumIWork(S, 0);
   ssSetNumPWork(S, 0);
   ssSetNumModes(S, 0);
   ssSetNumNonsampledZCs(S, 0);
   ssSetOptions(S, 0);
}

#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
   ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME);
   ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
   ssSetRWorkValue(S,0,ssGetTStart(S));
}

#ifndef min
#define min(a,b) ((a) <= (b) ? (a) : (b))
#endif

static void mdlOutputs(SimStruct *S, int_T tid)
{
   double            *t_x = ssGetDiscStates(S);
   double            *t_y = ssGetOutputPortRealSignal(S,0);
   double             t_previousSimTime = ssGetRWorkValue(S,0);
   const double      *scaleFactor = mxGetPr(TIME_SCALE_FACTOR(S));
   time_T             t_SimTime = ssGetT(S);
   double             t_diff = 0.0;
   double             dt;
   double             t_current;
   double             t_0;
   double             t_previous;
   double             t_elapsed;
   double             t_execution;
     
   /* Desired Delta time */
   dt = (t_SimTime - t_previousSimTime)*(scaleFactor[0]);
   
   /* Get clock time at the beginning of this step*/   
   t_previous = hightimer();
   t_0 = t_previous;
   
   /* Wait to reach the desired time */
   t_execution = t_0-t_x[0];
   while (t_diff < (dt - min(dt,t_execution))) {
       t_current = hightimer();
       /* Look for wrap-up */
       if (t_current<t_previous){
           t_elapsed = t_previous - t_0;
           t_0 = hightimer() - t_elapsed;
       }
       t_diff = t_current - t_0;
       t_previous = t_current;
   }
       
   /* Store current time to be used in next time step*/
   t_y[0] = dt - t_execution;
   t_x[0] = t_previous;
   ssSetRWorkValue(S,0,t_SimTime);
}

static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S); /* unused input argument */
}

/* Required S-function trailer */
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
