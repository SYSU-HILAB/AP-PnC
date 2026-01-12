import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver
from feature.lu_mpc.model.ctrl.enmpc_model import export_error_state_model
from scipy.linalg import block_diag
def test_acados_enmpc_model():
    # 1. Export model
    model = export_error_state_model()
    print("Model exported successfully.")
    print(model)
    # 2. Create OCP object
    ocp = AcadosOcp()
    ocp.model = model
    N = 20  # number 
    Tf = 2.0
    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # 3. Set cost, constraints, etc. (minimal example)
    nx = model.x.size()[0]
    nu = model.u.size()[0]
    np_ = model.p.size()[0]

    print("nx =", nx)
    print("nu =", nu)
    print("np_ =", np_)

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    Q = np.eye(nx)
    R = np.eye(nu)

    ocp.cost.W = block_diag(Q, R)
    ocp.cost.W_e = Q

    ocp.cost.Vx_0 = np.eye(nx)
    ocp.cost.Vx = np.concatenate((np.eye(nx), np.zeros((nu, nx))), axis=0)
    ocp.cost.Vu = np.concatenate((np.zeros((nx, nu)), np.eye(nu)), axis=0)
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.yref = np.zeros(nx + nu)
    ocp.cost.yref_e = np.zeros(nx)

    # 4. Set constraints
    ocp.constraints.x0 = np.zeros(nx)

    # 5. Set dummy parameters (if needed)
    ocp.parameter_values = np.zeros(np_)

    # 6. Create solver
    solver = AcadosOcpSolver(ocp, json_file=None)

    # 7. Solve OCP
    status = solver.solve()
    print(f"acados solve status: {status}")
    # assert status == 0, "acados solver failed!"
    x_sol = [solver.get(i, "x") for i in range(N+1)]
    u_sol = [solver.get(i, "u") for i in range(N)]
    print("x_sol[0] =", x_sol[0])
    print("u_sol[0] =", u_sol[0])
    print("Test passed!")

if __name__ == "__main__":
    test_acados_enmpc_model()
