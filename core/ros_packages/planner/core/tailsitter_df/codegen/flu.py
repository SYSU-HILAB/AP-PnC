"""Direct ENU/FLU flatness; +Z collective, +X simplified aerodynamic force.

World is ENU and gravity acts along -z, so eta = a + [0,0,g]. The previous
NED-style `a - [0,0,g]` was inconsistent with an ENU acceleration input and
implied a gravity-up world (2g residual in the dynamics identity).

No simulation legacy frame conversion. Values: a_T, omega_B xyz, R_WB row-major.
The active MINCO optimizer still uses its legacy cost kernel; this new ABI is
validated independently and is not a silent change to that cost's units.
"""

import casadi as ca


def symbolic_flu_kernel():
    z = ca.SX.sym("z", 9)
    p = ca.SX.sym("params", 4)  # g_z, k_body_x, speed guard, projection guard
    seed = ca.SX.sym("seed", 13)
    v, a, j = z[:3], z[3:6], z[6:9]
    eta = a + ca.vertcat(0, 0, p[0])
    speed = ca.norm_2(v)
    h = eta - p[1] * speed * v
    zb = h / ca.norm_2(h)
    y = ca.cross(eta, v)
    yb = y / ca.norm_2(y)
    nominal_r = ca.horzcat(ca.cross(yb, zb), yb, zb)
    fallback_z = eta / ca.norm_2(eta)

    def fallback_r(heading):
        y = ca.cross(fallback_z, heading)
        y /= ca.norm_2(y)
        return ca.horzcat(ca.cross(y, fallback_z), y, fallback_z)

    def quantities(r):
        dr = ca.reshape(ca.jtimes(ca.vec(r), z, ca.vertcat(a, j, ca.SX.zeros(3))), 3, 3)
        s = r.T @ dr
        omega = ca.vertcat(s[2, 1] - s[1, 2], s[0, 2] - s[2, 0], s[1, 0] - s[0, 1]) / 2
        value = ca.vertcat(
            ca.dot(eta, r[:, 2]), omega, *[r[row, col] for row in range(3) for col in range(3)]
        )
        return value, ca.densify(ca.jacobian(value, z)), ca.jtimes(value, z, seed, True)

    nominal = quantities(nominal_r)
    fx = quantities(fallback_r(ca.vertcat(1, 0, 0)))
    fy = quantities(fallback_r(ca.vertcat(0, 1, 0)))
    free = quantities(ca.SX.eye(3))
    projected = eta - v * ca.dot(v, eta) / ca.dot(v, v)
    low = speed < p[2]
    x_aligned = ca.norm_2(ca.cross(fallback_z, ca.vertcat(1, 0, 0))) < p[3]
    selected = []
    for n, x, y, zero in zip(nominal, fx, fy, free, strict=True):
        fallback = ca.if_else(ca.norm_2(eta) > p[3], ca.if_else(x_aligned, y, x, True), zero, True)
        selected.append(
            ca.if_else(
                low, fallback, ca.if_else(ca.norm_2(projected) < p[3], fallback, n, True), True
            )
        )
    return ca.Function("ap_flu_kernel", [z, p, seed], selected)
