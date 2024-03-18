#
# Created on Wed Aug 16 2023
#
# Copyright (c) 2023 SYSU
# author: WarriorHanamy ; email: rongerch@mail2.sysu.edu.cn
#
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosModel, AcadosSimOpts, AcadosSim
from config.config import MpcConfig
from create_ocp_or_sim import *
import numpy as np
import scipy.linalg
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import copy
from utils.terminal_color import * 
# TODO it will take model as argument to generate dynamic model optimization output


class MpcSolver:
    """mpc solver 
    """

    def __init__(self, cython=False) -> None:
        # super().__init__('MpcSolver')
        # self.mode = mode
        # Acados settting
        # TODO your ocp is tlightly coupled with your ocp create, we may not need model

        self.__ocp: AcadosOcp = ocp_nls()
            
        warmstart_ocp = ocp_nls()
        warmstart_ocp.solver_options.nlp_solver_type = "SQP"
        if cython:
            AcadosOcpSolver.generate(
                self.__ocp, json_file="acados_ocp_" + self.__ocp.model.name + ".json"
            )
            AcadosOcpSolver.build(
                self.__ocp.code_export_directory, with_cython=True
            )
            self.__acados_ocp_solver: AcadosOcpSolver = AcadosOcpSolver.create_cython_solver(
                "acados_ocp_" + self.__ocp.model.name + ".json")
            
            # For now, only cython enable warmstart.
            # move config to config 
            AcadosOcpSolver.generate(
                warmstart_ocp, json_file="warm_start_ocp_" + warmstart_ocp.model.name + ".json"
            )
            AcadosOcpSolver.build(
                warmstart_ocp.code_export_directory, with_cython=True
            )
            self.warm_ocp_solver: AcadosOcpSolver = AcadosOcpSolver.create_cython_solver(
                "warm_start_ocp_" + warmstart_ocp.model.name + ".json"
            )
        else:
            self.__acados_ocp_solver:AcadosOcpSolver = AcadosOcpSolver(
                self.__ocp, json_file="acados_ocp_" + self.__ocp.model.name + ".json"
            )
            print(warmstart_ocp)
            self.warm_ocp_solver: AcadosOcpSolver = AcadosOcpSolver(
                warmstart_ocp, json_file= "warm_start_ocp_" + warmstart_ocp.model.name + ".json"
            )
        self.warmstart = False
        self.Nodes = MpcConfig.nodes
        self.nx = self.__ocp.model.x.size()[0]
        self.nu = self.__ocp.model.u.size()[0]
        self.ny = self.nx + self.nu
        # if MpcConfig.airframe == MpcConfig.Airframe.Tailsitter:
        #     self.__sim = crea
        self.__sim = create_sim_solver_with_model_defined(
            Nodes= self.Nodes, T=MpcConfig.T_horizon)
        AcadosSimSolver.generate(
            self.__sim, json_file="acados_sim_" + self.__sim.model.name + ".json"
        )
        AcadosSimSolver.build(self.__sim.code_export_directory, with_cython=True)
        self.__acados_sim_solver = AcadosSimSolver.create_cython_solver('acados_sim_' + self.__sim.model.name + '.json')


        self.simX = np.ndarray((self.Nodes+1, self.nx))
        self.simU = np.ndarray((self.Nodes, self.nu))

        self.mpc_predicts = np.ndarray((self.Nodes, self.nx))

        for stage in range(self.Nodes + 1):
            self.__acados_ocp_solver.set(stage, "x", np.zeros((self.nx,)))

        for stage in range(self.Nodes):
            self.__acados_ocp_solver.set(stage, "u", np.zeros(self.nu,))


    def setRefPath(self, xTarget):
        """
       change ref_states(trajectory) for mpc

       Args:
           xTarget: Nx13D (pos, vel, rate)
       """
        while xTarget.shape[0] < self.Nodes + 1: ## finally its shape[0] becomes self.Nodes + 1
            xTarget = np.concatenate([xTarget, np.expand_dims(xTarget[-1,:], 0)], 0)
        uTarget = np.zeros(4,)     # All stage

        for j in range(self.Nodes): 
            ref = xTarget[j,:]
            yref = np.concatenate([ref, uTarget])
            self.__acados_ocp_solver.set(j, "yref", yref)
        self.__acados_ocp_solver.set(self.Nodes, "yref", xTarget[self.Nodes,:])


    def solve(self, x0: np.ndarray) -> int:
        """
        solve the optimization

        Args:
            xCur (np.ndarray): 13D current state to formulate the problem

        Returns:
            status: the optimization result
        """


        xCur = x0.copy()
        xCur[9:] = xCur[9:] / np.linalg.norm(xCur[9:])

        if not self.warmstart:
            print(f'{TerminalColor.OKCYAN} WarmStart Begins!')
            self.warm_ocp_solver.set(0, "lbx", xCur)
            self.warm_ocp_solver.set(0, "ubx", xCur)
            status = self.warm_ocp_solver.solve()
            if status not in [0,2]:
                self.__acados_ocp_solver.print_statistics()
                raise Exception(
                    f"acados acados_ocp_solver returned status {status} with x current {xCur}"
                )
            for i in range(self.Nodes + 1):
                self.__acados_ocp_solver.set(i, "x", self.warm_ocp_solver.get(i, 'x'))
                self.__acados_ocp_solver.set(i, "u", self.warm_ocp_solver.get(i, 'u'))
                self.warmstart = True
            print(f'WarmStart Ends! {TerminalColor.ENDC}')
        # return
        self.__acados_ocp_solver.set(0, "lbx", xCur)
        self.__acados_ocp_solver.set(0, "ubx", xCur)
        status = self.__acados_ocp_solver.solve()
        # print(status)
        # status 2 is maximum iteration 
        if status not in [0,2]:
            self.__acados_ocp_solver.print_statistics()
            raise Exception(
                f"acados acados_ocp_solver returned status {status} with x current {xCur}"
            )

        # if status == 2:
        #     print(
        #         f"acados acados_ocp_solver returned status {status} with {xCur}"
        #     )

        # TODO figure out acados integrator
        for i in range(self.Nodes):
            self.simX[i, :] = self.__acados_ocp_solver.get(i, "x")
            self.simU[i, :] = self.__acados_ocp_solver.get(i, "u")
        self.simX[self.Nodes, :] = self.__acados_ocp_solver.get(
            self.Nodes, "x")
        return status

    def sim_traj(self, xCur=None):
        if xCur == None:
            xCur = self.simX[0, :]
        x0 = xCur.copy()
        for i in range(self.Nodes):
            self.mpc_predicts[i,:] = np.array(self.__acados_sim_solver.simulate(x=x0, u=self.simU[i,:]))
            x0 = self.mpc_predicts[i,:]
        return self.mpc_predicts
    
    def sim_one_step(self, xCur=None):
        if xCur == None:
            xCur = self.simX[0, :]
        x0 = xCur.copy()
        # TODO
        return np.array(self.__acados_sim_solver.simulate(x=x0, u=self.u))
    
    def sim_one_step_rate(self, xCur=None):
        if xCur == None:
            xCur = self.simX[0, :]
        x0 = xCur.copy()
        print(f'x0 {x0}')
        x1 = self.__acados_sim_solver.simulate(x=x0, u=self.u)
        
        return x1[6:9]

    # def setHorizon(self, T):
    #     self.__acados_ocp_solver.set("")
        
    @property
    def u(self) -> np.ndarray:
        return self.simU[0, :]
    
    def u_seq(self, len) -> np.ndarray:
        return self.simU[:len, :]


    @property
    def solve_time(self):
        return self.__acados_ocp_solver.get_stats('time_tot')

    def print_residuals(self):
        print(self.__acados_ocp_solver.get_residuals())


def main():
    pass


if __name__ == '__main__':
    main()
