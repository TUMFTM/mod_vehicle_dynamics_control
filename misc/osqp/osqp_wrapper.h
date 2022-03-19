#ifndef OSQP_WRAPPER_H
#define OSQP_WRAPPER_H

#include "time.h"
#include "header/osqp.h"

typedef struct osqp_wrapper{
    // problem configuration
    c_int n; // number of optimization variables
    c_int m; // number of constraints
    c_int P_nnz; // number of nonzero elements in P matrix
    c_int A_nnz; // number of nonzero elements in A matrix

    // OSQP data
    OSQPWorkspace *work;
    OSQPSettings  *settings;
    OSQPData      *data;

    // output data, public to be accessible from s-function
    c_int flag_bound_upd;
    c_int flag_A_upd;
    c_int flag_P_upd;
    c_int flag_q_upd;
    c_int flag_solve;
    c_int flag_setup;
} osqp_wrapper;

void init_osqp_wrapper(osqp_wrapper* wrapper,
    c_float *p_0, c_float *p_1, c_float *p_2, c_float *p_3,
  c_float *p_4, c_float *p_5, c_float *p_6, c_float *p_7,
c_float *p_8, c_float *p_9, c_float *p_10, c_float *p_11, c_float *p_13);

void cleanup_osqp_wrapper(osqp_wrapper* wrapper);

void restart_osqp_wrapper(osqp_wrapper* wrapper);

void update_osqp_wrapper(osqp_wrapper* wrapper, c_float* q_upd, c_float* l_upd,
    c_float* u_upd, c_float* A_x_upd);

#endif
