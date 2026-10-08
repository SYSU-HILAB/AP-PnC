#include "nmpc_solver.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_tailsitter_flu.h"

static_assert(TAILSITTER_FLU_NX == NMPC_NX,
              "stale NMPC state ABI: regenerate bundle");
static_assert(TAILSITTER_FLU_NU == NMPC_NU, "stale NMPC control ABI");
static_assert(TAILSITTER_FLU_NP == NMPC_NP, "stale NMPC parameter ABI");
static_assert(TAILSITTER_FLU_NY0 == NMPC_NY && TAILSITTER_FLU_NY == NMPC_NY,
              "stale NMPC stage cost ABI");
static_assert(TAILSITTER_FLU_NYN == NMPC_NYN, "stale NMPC terminal cost ABI");
struct nmpc_solver
{
  tailsitter_flu_solver_capsule *capsule;
  ocp_nlp_config                *nlp_config;
  ocp_nlp_dims                  *nlp_dims;
  ocp_nlp_in                    *nlp_in;
  ocp_nlp_out                   *nlp_out;
  int                            N;
};

static int _set_reference_stage(int stage, void *y_ref, ocp_nlp_config *config,
                                ocp_nlp_dims *dims, ocp_nlp_in *nlp_in,
                                ocp_nlp_out *nlp_out);
static int _set_model_parameter(int stage, double *p,
                                tailsitter_flu_solver_capsule *capsule);

static int _set_current_state_lbx(void *x, ocp_nlp_config *config,
                                  ocp_nlp_dims *dims, ocp_nlp_in *nlp_in,
                                  ocp_nlp_out *nlp_out);
static int _set_current_state_ubx(void *x, ocp_nlp_config *config,
                                  ocp_nlp_dims *dims, ocp_nlp_in *nlp_in,
                                  ocp_nlp_out *nlp_out);

static void _get_nlp_out_u(void *u, ocp_nlp_config *config, ocp_nlp_dims *dims,
                           ocp_nlp_out *nlp_out);

static void _get_nlp_out_x_next(void *x, ocp_nlp_config *config,
                                ocp_nlp_dims *dims, ocp_nlp_out *nlp_out);

int nmpc_solver_abi_version(void)
{
  return NMPC_SOLVER_ABI;
}

nmpc_solver_t *nmpc_solver_create(void)
{
  nmpc_solver_t *s = (nmpc_solver_t *)calloc(1, sizeof(nmpc_solver_t));
  if (!s)
    return NULL;

  s->capsule = tailsitter_flu_acados_create_capsule();
  if (!s->capsule)
  {
    free(s);
    return NULL;
  }

  int status = tailsitter_flu_acados_create(s->capsule);
  if (status)
  {
    printf("nmpc_solver: acados_create returned %d\n", status);
    tailsitter_flu_acados_free_capsule(s->capsule);
    free(s);
    return NULL;
  }

  s->nlp_config = tailsitter_flu_acados_get_nlp_config(s->capsule);
  s->nlp_dims   = tailsitter_flu_acados_get_nlp_dims(s->capsule);
  s->nlp_in     = tailsitter_flu_acados_get_nlp_in(s->capsule);
  s->nlp_out    = tailsitter_flu_acados_get_nlp_out(s->capsule);
  s->N          = s->nlp_dims->N;

  return s;
}

void nmpc_solver_destroy(nmpc_solver_t *s)
{
  if (!s)
    return;
  tailsitter_flu_acados_free(s->capsule);
  tailsitter_flu_acados_free_capsule(s->capsule);
  free(s);
}

int nmpc_solver_horizon(const nmpc_solver_t *s)
{
  return s->N;
}

double nmpc_solver_stage_dt(const nmpc_solver_t *s, int stage)
{
  if (!s || stage < 0 || stage >= s->N)
    return -1.0;
  double dt = 0.0;
  ocp_nlp_in_get(s->nlp_config, s->nlp_dims, s->nlp_in, stage, "Ts", &dt);
  return dt;
}

void nmpc_solver_set_state(nmpc_solver_t *s, const double x_cur[NMPC_NX])
{
  _set_current_state_lbx((void *)x_cur, s->nlp_config, s->nlp_dims, s->nlp_in,
                         s->nlp_out);
  _set_current_state_ubx((void *)x_cur, s->nlp_config, s->nlp_dims, s->nlp_in,
                         s->nlp_out);
}

void nmpc_solver_set_ref_stage(nmpc_solver_t *s, int stage,
                               const double y_ref[NMPC_NY],
                               const double p[NMPC_NP])
{
  _set_reference_stage(stage, (void *)y_ref, s->nlp_config, s->nlp_dims,
                       s->nlp_in, s->nlp_out);
  if (stage < s->N)
  {
    _set_model_parameter(stage, (double *)p, s->capsule);
  }
}

void nmpc_solver_set_ref_terminal(nmpc_solver_t *s,
                                  const double   y_ref_e[NMPC_NYN])
{
  _set_reference_stage(s->N, (void *)y_ref_e, s->nlp_config, s->nlp_dims,
                       s->nlp_in, s->nlp_out);
}

int nmpc_solver_solve(nmpc_solver_t *s, double u_out[NMPC_NU],
                      double x_next[NMPC_NX])
{
  int status = tailsitter_flu_acados_solve(s->capsule);
  if (status == 0 || status == 2)
  {
    _get_nlp_out_u(u_out, s->nlp_config, s->nlp_dims, s->nlp_out);
    _get_nlp_out_x_next(x_next, s->nlp_config, s->nlp_dims, s->nlp_out);
  }
  return status;
}

static int _set_reference_stage(int stage, void *y_ref, ocp_nlp_config *config,
                                ocp_nlp_dims *dims, ocp_nlp_in *nlp_in,
                                ocp_nlp_out *nlp_out)
{
  return ocp_nlp_cost_model_set(config, dims, nlp_in, stage, "yref", y_ref);
}

static int _set_model_parameter(int stage, double *p,
                                tailsitter_flu_solver_capsule *capsule)
{
  return tailsitter_flu_acados_update_params(capsule, stage, p, NMPC_NP);
}

static int _set_current_state_lbx(void *x, ocp_nlp_config *config,
                                  ocp_nlp_dims *dims, ocp_nlp_in *nlp_in,
                                  ocp_nlp_out *nlp_out)
{
  return ocp_nlp_constraints_model_set(config, dims, nlp_in, nlp_out, 0, "lbx",
                                       x);
}

static int _set_current_state_ubx(void *x, ocp_nlp_config *config,
                                  ocp_nlp_dims *dims, ocp_nlp_in *nlp_in,
                                  ocp_nlp_out *nlp_out)
{
  return ocp_nlp_constraints_model_set(config, dims, nlp_in, nlp_out, 0, "ubx",
                                       x);
}

static void _get_nlp_out_u(void *u, ocp_nlp_config *config, ocp_nlp_dims *dims,
                           ocp_nlp_out *nlp_out)
{
  ocp_nlp_out_get(config, dims, nlp_out, 0, "u", u);
}

static void _get_nlp_out_x_next(void *x, ocp_nlp_config *config,
                                ocp_nlp_dims *dims, ocp_nlp_out *nlp_out)
{
  ocp_nlp_out_get(config, dims, nlp_out, 1, "x", x);
}
