#ifndef NMPC_SOLVER_H_
#define NMPC_SOLVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define NMPC_NX 18
#define NMPC_NU 4
#define NMPC_NP 3
#define NMPC_NY 16
#define NMPC_NYN 12

  typedef struct nmpc_solver nmpc_solver_t;

  nmpc_solver_t *nmpc_solver_create(void);
  void           nmpc_solver_destroy(nmpc_solver_t *s);

  int nmpc_solver_horizon(const nmpc_solver_t *s);

  void nmpc_solver_set_state(nmpc_solver_t *s, const double x_cur[NMPC_NX]);

  void nmpc_solver_set_ref_stage(nmpc_solver_t *s, int stage,
                                 const double y_ref[NMPC_NY],
                                 const double p[NMPC_NP]);

  void nmpc_solver_set_ref_terminal(nmpc_solver_t *s,
                                    const double   y_ref_e[NMPC_NYN]);

  int nmpc_solver_solve(nmpc_solver_t *s, double u_out[NMPC_NU],
                        double x_next[NMPC_NX]);

#ifdef __cplusplus
}
#endif

#endif /* NMPC_SOLVER_H_ */
