"""Build-only CasADi -> standalone C flatness values, Jacobian and VJP.

This kernel mirrors the *four optimization outputs* of tailsitter_df.cpp,
not its attitude/force side effects. It is not yet a replacement for Data.
The first output is tangent acceleration (despite legacy 'thrust' names).
"""

import json
import os

import casadi as ca
from flu import symbolic_flu_kernel

from ap_pnc.env import artifact_path, get_git_root


def symbolic_kernel():
    z = ca.SX.sym("z", 9)
    params = ca.SX.sym("params", 4)  # g_z, phi_33, vThreshold, apThreshold
    seed = ca.SX.sym("seed", 4)
    v, a, j = z[:3], z[3:6], z[6:9]
    gravity = ca.vertcat(0, 0, params[0])
    speed = ca.norm_2(v)
    xs = v / speed
    temp = -(ca.SX.eye(3) - xs @ xs.T) @ (a - gravity)
    zs = temp / ca.norm_2(temp)
    ys = ca.cross(zs, xs)
    apx, apz = ca.dot(xs, a - gravity), ca.dot(zs, a - gravity)
    wsz = ca.dot(ys, gravity) / speed
    wsx = (wsz * apx - ca.dot(ys, j)) / apz
    wsy = -ca.dot(zs, a) / speed
    apz_dot = ca.dot(zs, j) + wsy * apx
    apx_dot = ca.dot(xs, j) - wsy * apz
    increment = apx - params[1] * speed**2
    numerator = (
        params[1] * (speed**2 * apz_dot - 2 * apz * ca.dot(v, a)) + apz * apx_dot - apx * apz_dot
    )
    alpha_dot = numerator / (apz**2 + increment**2)
    nominal = ca.vertcat(ca.dot(xs, a), wsx, wsy + alpha_dot, wsz)
    projected = ca.vertcat(ca.dot(xs, a), 0, 0, 0)

    # Select branchwise derivatives explicitly: differentiating a conditional
    # DAG can still evaluate singular intermediate tangents (0 * NaN at v=0).
    # Switching surfaces have no claimed classical derivative.
    def select(low, proj, regular):
        return ca.if_else(
            speed < params[2],
            low,
            ca.if_else(ca.norm_2(temp) < params[3], proj, regular, True),
            True,
        )

    value = select(ca.SX.zeros(4), projected, nominal)
    jac = ca.densify(select(ca.SX.zeros(4, 9), ca.jacobian(projected, z), ca.jacobian(nominal, z)))
    vjp = select(
        ca.SX.zeros(9), ca.jtimes(projected, z, seed, True), ca.jtimes(nominal, z, seed, True)
    )
    full = ca.Function("ap_flatness_kernel", [z, params, seed], [value, jac, vjp])
    backward = ca.Function("ap_flatness_vjp", [z, params, seed], [value, vjp])
    return full, backward


def main():
    root = get_git_root()
    # Paths are resolved by project utilities, never by caller cwd.
    out = artifact_path(root / ".artifacts" / "flatness" / f"linux-{os.uname().machine}")
    out.mkdir(parents=True, exist_ok=True)
    f, backward = symbolic_kernel()
    flu = symbolic_flu_kernel()
    generator = ca.CodeGenerator("flatness_generated.c", {"with_header": True})
    generator.add(f)
    generator.add(backward)
    generator.add(flu)
    generator.generate(str(out) + "/")
    wrapper = f"""#include "flatness_generated.h"
/* Direct main-frame ABI: values a_T, omega_FLU(3), R_WB row-major(9).
 * J[row + 13*col]. No simulation legacy-wing adapter. */
int ap_pnc_flatness_flu(const double z[9], const double params[4],
                         const double seed[13], double value[13],
                         double jacobian[117], double gradient[9]) {{
  const casadi_real *arg[{max(3, flu.sz_arg())}] = {{z, params, seed}};
  casadi_real *res[{max(3, flu.sz_res())}] = {{value, jacobian, gradient}};
  casadi_int iw[{max(1, flu.sz_iw())}];
  casadi_real w[{max(1, flu.sz_w())}];
  return ap_flu_kernel(arg, res, iw, w, 0);
}}
/* Column-major J: J[row + 4 * col]. No global work buffers or heap. */
int ap_pnc_flatness(const double z[9], const double params[4],
                    const double seed[4], double value[4],
                    double jacobian[36], double backward[9]) {{
  const casadi_real *arg[{max(3, f.sz_arg())}] = {{z, params, seed}};
  casadi_real *res[{max(3, f.sz_res())}] = {{value, jacobian, backward}};
  casadi_int iw[{max(1, f.sz_iw())}];
  casadi_real w[{max(1, f.sz_w())}];
  return ap_flatness_kernel(arg, res, iw, w, 0);
}}
/* Production VJP entry: no dense Jacobian is constructed. */
int ap_pnc_flatness_backward(const double z[9], const double params[4],
                             const double seed[4], double value[4],
                             double gradient[9]) {{
  const casadi_real *arg[{max(3, backward.sz_arg())}] = {{z, params, seed}};
  casadi_real *res[{max(2, backward.sz_res())}] = {{value, gradient}};
  casadi_int iw[{max(1, backward.sz_iw())}];
  casadi_real w[{max(1, backward.sz_w())}];
  return ap_flatness_vjp(arg, res, iw, w, 0);
}}
"""
    (out / "flatness_wrapper.c").write_text(wrapper)
    (out / "flatness.h").write_text("""#pragma once
#ifdef __cplusplus
extern "C" {
#endif
/* FLU ABI: params g_z,k_body_x,guards; value a_T,omega_B,R_WB row-major;
 * Jacobian is column-major 13x9. Legacy four-output ABI below is unchanged. */
int ap_pnc_flatness_flu(const double z[9], const double params[4],
                         const double seed[13], double value[13],
                         double jacobian[117], double gradient[9]);
/* params = {g_z, phi_33, vThreshold, apThreshold}; J is column-major. */
int ap_pnc_flatness(const double z[9], const double params[4],
                    const double seed[4], double value[4],
                    double jacobian[36], double backward[9]);
int ap_pnc_flatness_backward(const double z[9], const double params[4],
                             const double seed[4], double value[4],
                             double gradient[9]);
#ifdef __cplusplus
}
#endif
""")
    (out / "manifest.json").write_text(
        json.dumps(
            {
                "casadi_version": ca.__version__,
                "source_root": str(root),
                "input": "v_xyz,a_xyz,jerk_xyz",
                "output": "tangent_acc,omega_sx,omega_by,omega_sz",
                "jacobian_layout": "column-major 4x9",
                "backward": "J.T @ seed",
                "default_params": [-9.8, -0.09408, 0.5, 0.05],
                "scope": "legacy optimization ABI unchanged; separate direct FLU physical ABI",
                "flu": {
                    "input_frame": "world_ENU",
                    "body_frame": "body_FLU",
                    "thrust_axis": "+Z",
                    "aero_axis": "+X",
                    "zero_wind": True,
                    "output": "collective_specific_thrust,omega_B_xyz,R_WB_row_major",
                    "jacobian": "column-major 13x9",
                    "params": [-9.81, -0.09408, 0.5, 0.05],
                },
            },
            indent=2,
        )
    )
    print(out)


if __name__ == "__main__":
    main()
