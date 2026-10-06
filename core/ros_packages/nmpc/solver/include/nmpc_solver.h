#ifndef NMPC_SOLVER_H_
#define NMPC_SOLVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* x = [p(3), v(3), rate_command(3), q_wxyz(4), thrust_command N/kg, k_aero].
 * u = d[thrust_command, rate_command_xyz]/dt. No actuator lag or rotor states.
 * y = [p(3), v(3), yb(3), command_derivative(4)]; terminal = [p,v,yb]. */
#define NMPC_SOLVER_ABI 3
#define NMPC_NX 15
#define NMPC_NU 4
#define NMPC_NP 1
#define NMPC_NY 13
#define NMPC_NYN 9

  typedef struct nmpc_solver nmpc_solver_t;

  int            nmpc_solver_abi_version(void);
  nmpc_solver_t *nmpc_solver_create(void);
  void           nmpc_solver_destroy(nmpc_solver_t *s);

  int    nmpc_solver_horizon(const nmpc_solver_t *s);
  double nmpc_solver_stage_dt(const nmpc_solver_t *s, int stage);

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
