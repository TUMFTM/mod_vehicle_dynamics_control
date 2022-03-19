#include "osqp_wrapper.h"

void init_osqp_wrapper(osqp_wrapper* wrapper,
    c_float *p_0, c_float *p_1, c_float *p_2, c_float *p_3,
  c_float *p_4, c_float *p_5, c_float *p_6, c_float *p_7,
c_float *p_8, c_float *p_9, c_float *p_10, c_float *p_11, c_float *p_12)
{
    // parse input data
    c_int n = p_2[0];   // *p_2 => n = number of optimization variables
    c_int m = p_1[0];   // *p_1 => m = number of constraints

    c_int P_nnz = p_5[n]; // P_nnz = number of nonzero elements in upper triangular of P
    c_float* P_x;
    P_x = malloc(P_nnz*sizeof(c_float)); // P_x (dimension = 1xP_nnz) contains !! in C/C++ !! values of the UPPER TRIANGULAR, NOT full matrix P as in Matlab
    for(int i = 0; i < P_nnz; i++){
        P_x[i] = p_3[i];
    }

    c_int* P_i;
    P_i = malloc(P_nnz*sizeof(c_int)); // P_i (dimension = 1xP_nnz) contains corresponding row indices of P_x values
    for(int i = 0; i < P_nnz; i++){
        P_i[i] = p_4[i];
    }
    c_int* P_p;
    P_p = malloc((n+1)*sizeof(c_int)); // P_p (dimension = 1x(n+1)) contains numbers of nonzero entries in each column of P
    for(int i = 0; i < (n + 1); i++){
        P_p[i] = p_5[i];
    }
    // initialize q vector using the values of the specified s-function parameter
    c_float* q;
    q = malloc(n*sizeof(c_float));
    for(int i = 0; i < n; i++){
        q[i] = p_0[i];
    }

    // check for correct dimensions of passed A_param: number of elements in p_6 = p6width must equal m*n
    c_int A_nnz = p_8[n];  // A is always represented as full matrix --> number of nonzero entries = m*n
    c_float* A_x;
    A_x = malloc(A_nnz*sizeof(c_float)); // A_x (dimension = 1x(m*n)) contains values of the full matrix A
    c_int* A_i;
    A_i = malloc(A_nnz*sizeof(c_int));
    c_int* A_p;
    A_p = malloc((n + 1)*sizeof(c_int));

    for(int i = 0; i < A_nnz; i++){
        A_x[i] = p_6[i];
    }
    for(int i = 0; i < A_nnz; i++){
        A_i[i] = p_7[i];
    }
    for(int i = 0; i < (n + 1); i++){
        A_p[i] = p_8[i];
    }

    // initialize lower and upper bound vector
    c_float* l;
    l = malloc(m*sizeof(c_float));
    c_float* u;
    u = malloc(m*sizeof(c_float));
    for(int i = 0; i < m; i++){
        l[i] = p_9[i];
        u[i] = p_10[i];
    }


    // copy problem configuration
    wrapper->n = n;
    wrapper->m = m;
    wrapper->P_nnz = P_nnz;
    wrapper->A_nnz = A_nnz;

    // initialize OSQP
    wrapper->settings = (OSQPSettings *) malloc(sizeof(OSQPSettings));
    wrapper->data     = (OSQPData *) malloc(sizeof(OSQPData));

    // fill data structure
    wrapper->data->n = n;
    wrapper->data->m = m;
    wrapper->data->P = csc_matrix(wrapper->data->n, wrapper->data->n,
      wrapper->P_nnz, P_x, P_i, P_p);
    wrapper->data->q = q;
    wrapper->data->A = csc_matrix(wrapper->data->m, wrapper->data->n,
      wrapper->A_nnz, A_x, A_i, A_p);
    wrapper->data->l = l;
    wrapper->data->u = u;

    // create an default settings object and set some parameters
    osqp_set_default_settings(wrapper->settings);
    wrapper->settings->alpha = 1.0;
    wrapper->settings->verbose = 0;
    wrapper->settings->scaling = 0;
    wrapper->settings->max_iter = p_12[0];

    // initialize all flags to be ok
    wrapper->flag_bound_upd = 0;
    wrapper->flag_A_upd = 0;
    wrapper->flag_P_upd = 0;
    wrapper->flag_q_upd = 0;
    wrapper->flag_solve = 0;
    wrapper->flag_setup = 0;

    // create OSQP workspace and pass return value
    wrapper->flag_setup = osqp_setup(&(wrapper->work), wrapper->data, wrapper->settings);
}

void cleanup_osqp_wrapper(osqp_wrapper* wrapper)
{
    osqp_cleanup(wrapper->work); 
    free(wrapper->data->A);
    free(wrapper->data->P);
    free(wrapper->data);
    free(wrapper->settings);
}

void restart_osqp_wrapper(osqp_wrapper* wrapper)
{
    c_float* x;
    x = malloc((wrapper->data->n)*sizeof(c_float));
    for(int i = 0; i < wrapper->data->n; i++){
        x[i] = 0;
    }
    c_float* y;
    y = malloc((wrapper->data->m)*sizeof(c_float));
    for(int i = 0; i < wrapper->data->m; i++){
        y[i] = 0;
    }
    osqp_warm_start(wrapper->work, x, y);
}

void update_osqp_wrapper(osqp_wrapper* wrapper, c_float* q_upd, c_float* l_upd,
    c_float* u_upd, c_float* A_x_upd)
{
       
    // update problem data
    wrapper->flag_q_upd =
      osqp_update_lin_cost(wrapper->work, q_upd);
    wrapper->flag_bound_upd =
      osqp_update_bounds(wrapper->work, l_upd, u_upd);
    wrapper->flag_A_upd = osqp_update_A(wrapper->work, A_x_upd, OSQP_NULL, wrapper->A_nnz);
        
    // specify warmstart
    // osqp_warm_start(wrapper->work, x, y);
    
    // solve problem
    wrapper->flag_solve = osqp_solve(wrapper->work);
}

