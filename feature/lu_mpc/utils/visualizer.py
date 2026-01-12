# -*- coding: future_fstrings -*-
#
# Visualization Utilities for Aerodynamic Simulation
# Created on 2024-12-28
#
# This module provides visualization tools for simulation results,
# including 3D trajectory plotting, state evolution, and aerodynamic forces.
#

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from typing import Optional, Dict, List
import dataclasses
from feature.lu_mpc.utils.math_utils import quat_to_euler


@dataclasses.dataclass
class PlotConfig:
    """Configuration for plotting."""

    # Figure settings
    figure_size: tuple = (14, 10)
    dpi: int = 100

    # Plot styling
    trajectory_color: str = "blue"
    trajectory_linewidth: float = 1.5
    start_marker: str = "o"
    start_color: str = "green"
    end_marker: str = "o"
    end_color: str = "red"
    marker_size: int = 8

    # Axis labels
    position_labels: List[str] = dataclasses.field(
        default_factory=lambda: ["X [m]", "Y [m]", "Z [m]"]
    )
    velocity_labels: List[str] = dataclasses.field(
        default_factory=lambda: ["Vx [m/s]", "Vy [m/s]", "Vz [m/s]"]
    )
    angle_labels: List[str] = dataclasses.field(
        default_factory=lambda: ["Roll [deg]", "Pitch [deg]", "Yaw [deg]"]
    )

    # Grid settings
    show_grid: bool = True
    grid_alpha: float = 0.3

    # Title fontsize
    title_fontsize: int = 12


# Default configuration
DEFAULT_PLOT_CONFIG = PlotConfig()


class SimulationVisualizer:
    """Visualizer for simulation results."""

    def __init__(self, config: PlotConfig = DEFAULT_PLOT_CONFIG):
        """
        Initialize the visualizer.

        Args:
            config: Plotting configuration
        """
        self.config = config

    def plot_trajectory_3d(self, results: Dict, title: str = "3D Trajectory"):
        """
        Plot 3D trajectory of the vehicle.

        Args:
            results: Simulation results dictionary
            title: Plot title
        """
        position = results["position"]
        times = results["times"]

        fig = plt.figure(figsize=self.config.figure_size)
        ax = fig.add_subplot(111, projection="3d")

        # Plot trajectory
        ax.plot(
            position[:, 0],
            position[:, 1],
            position[:, 2],
            color=self.config.trajectory_color,
            linewidth=self.config.trajectory_linewidth,
            label="Trajectory",
        )

        # Mark start and end points
        ax.scatter(
            position[0, 0],
            position[0, 1],
            position[0, 2],
            marker=self.config.start_marker,
            c=self.config.start_color,
            s=self.config.marker_size**2,
            label="Start",
        )
        ax.scatter(
            position[-1, 0],
            position[-1, 1],
            position[-1, 2],
            marker=self.config.end_marker,
            c=self.config.end_color,
            s=self.config.marker_size**2,
            label="End",
        )

        # Labels and title
        ax.set_xlabel(self.config.position_labels[0])
        ax.set_ylabel(self.config.position_labels[1])
        ax.set_zlabel(self.config.position_labels[2])
        ax.set_title(title)
        ax.legend()

        if self.config.show_grid:
            ax.grid(True, alpha=self.config.grid_alpha)

        plt.tight_layout()
        return fig

    def plot_state_evolution(self, results: Dict, title: str = "State Evolution"):
        """
        Plot evolution of all states over time.

        Args:
            results: Simulation results dictionary
            title: Plot title
        """
        times = results["times"]
        position = results["position"]
        velocity = results["velocity"]
        angular_rates = results["angular_rates"]
        quaternion = results["quaternion"]
        motor_thrusts = results["motor_thrusts"]

        fig = plt.figure(figsize=self.config.figure_size)
        gs = gridspec.GridSpec(3, 2, hspace=0.3, wspace=0.3)

        # Position
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(times, position[:, 0], label="X")
        ax1.plot(times, position[:, 1], label="Y")
        ax1.plot(times, position[:, 2], label="Z")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Position [m]")
        ax1.set_title("Position")
        ax1.legend()
        ax1.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        # Velocity
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(times, velocity[:, 0], label="Vx")
        ax2.plot(times, velocity[:, 1], label="Vy")
        ax2.plot(times, velocity[:, 2], label="Vz")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Velocity [m/s]")
        ax2.set_title("Velocity")
        ax2.legend()
        ax2.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        # Angular rates
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.plot(times, np.rad2deg(angular_rates[:, 0]), label="p")
        ax3.plot(times, np.rad2deg(angular_rates[:, 1]), label="q")
        ax3.plot(times, np.rad2deg(angular_rates[:, 2]), label="r")
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Angular Rate [deg/s]")
        ax3.set_title("Angular Rates")
        ax3.legend()
        ax3.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        # Euler angles (roll, pitch, yaw from quaternion)
        ax4 = fig.add_subplot(gs[1, 1])
        euler = quat_to_euler(quaternion)
        ax4.plot(times, np.rad2deg(euler[:, 0]), label="Roll")
        ax4.plot(times, np.rad2deg(euler[:, 1]), label="Pitch")
        ax4.plot(times, np.rad2deg(euler[:, 2]), label="Yaw")
        ax4.set_xlabel("Time [s]")
        ax4.set_ylabel("Angle [deg]")
        ax4.set_title("Euler Angles")
        ax4.legend()
        ax4.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        # Motor thrusts
        ax5 = fig.add_subplot(gs[2, 0])
        ax5.plot(times, motor_thrusts[:, 0], label="Motor 1")
        ax5.plot(times, motor_thrusts[:, 1], label="Motor 2")
        ax5.plot(times, motor_thrusts[:, 2], label="Motor 3")
        ax5.plot(times, motor_thrusts[:, 3], label="Motor 4")
        ax5.set_xlabel("Time [s]")
        ax5.set_ylabel("Thrust [N]")
        ax5.set_title("Motor Thrusts")
        ax5.legend()
        ax5.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        # Altitude
        ax6 = fig.add_subplot(gs[2, 1])
        altitude = -position[:, 2]  # Convert NED to altitude
        ax6.plot(times, altitude)
        ax6.set_xlabel("Time [s]")
        ax6.set_ylabel("Altitude [m]")
        ax6.set_title("Altitude")
        ax6.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        fig.suptitle(title, fontsize=self.config.title_fontsize + 2)

        return fig

    def plot_aerodynamic_forces(
        self, results: Dict, aero_data: List[Dict], title: str = "Aerodynamic Forces"
    ):
        """
        Plot aerodynamic forces evolution.

        Args:
            results: Simulation results dictionary
            aero_data: List of aerodynamic data dictionaries
            title: Plot title
        """
        times = results["times"][:-1]  # aero_data has one less point (no initial state)

        forces_body = np.array([d["forces_body"] for d in aero_data])
        moments_body = np.array([d["moments_body"] for d in aero_data])
        alpha = np.array([d["alpha"] for d in aero_data])
        beta = np.array([d["beta"] for d in aero_data])

        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle(title, fontsize=self.config.title_fontsize + 2)

        # Forces
        ax1 = axes[0, 0]
        ax1.plot(times, forces_body[:, 0], label="Fx")
        ax1.plot(times, forces_body[:, 1], label="Fy")
        ax1.plot(times, forces_body[:, 2], label="Fz")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Force [N]")
        ax1.set_title("Aerodynamic Forces (Body Frame)")
        ax1.legend()
        ax1.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        # Moments
        ax2 = axes[0, 1]
        ax2.plot(times, moments_body[:, 0], label="Mx")
        ax2.plot(times, moments_body[:, 1], label="My")
        ax2.plot(times, moments_body[:, 2], label="Mz")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Moment [Nm]")
        ax2.set_title("Aerodynamic Moments")
        ax2.legend()
        ax2.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        # Angle of attack
        ax3 = axes[1, 0]
        ax3.plot(times, np.rad2deg(alpha))
        ax3.set_xlabel("Time [s]")
        ax3.set_ylabel("Angle of Attack [deg]")
        ax3.set_title("Angle of Attack (α)")
        ax3.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        # Sideslip angle
        ax4 = axes[1, 1]
        ax4.plot(times, np.rad2deg(beta))
        ax4.set_xlabel("Time [s]")
        ax4.set_ylabel("Sideslip Angle [deg]")
        ax4.set_title("Sideslip Angle (β)")
        ax4.grid(self.config.show_grid, alpha=self.config.grid_alpha)

        plt.tight_layout()
        return fig

    def plot_comprehensive(
        self,
        results: Dict,
        aero_data: Optional[List[Dict]] = None,
        title: str = "Simulation Results",
    ):
        """
        Create comprehensive visualization with all plots.

        Args:
            results: Simulation results dictionary
            aero_data: Optional list of aerodynamic data dictionaries
            title: Overall plot title
        """
        # Plot 3D trajectory
        fig1 = self.plot_trajectory_3d(results, title=f"{title} - 3D Trajectory")

        # Plot state evolution
        fig2 = self.plot_state_evolution(results, title=f"{title} - States")

        # Plot aerodynamic forces if available
        if aero_data is not None:
            fig3 = self.plot_aerodynamic_forces(
                results, aero_data, title=f"{title} - Aerodynamics"
            )
            return [fig1, fig2, fig3]

        return [fig1, fig2]


def create_visualizer(config: PlotConfig = DEFAULT_PLOT_CONFIG) -> SimulationVisualizer:
    """
    Factory function to create a visualizer.

    Args:
        config: Plotting configuration

    Returns:
        SimulationVisualizer instance
    """
    return SimulationVisualizer(config)


def plot_simulation_results(
    results: Dict,
    aero_data: Optional[List[Dict]] = None,
    show: bool = True,
    save_path: Optional[str] = None,
):
    """
    Quick function to plot simulation results.

    Args:
        results: Simulation results dictionary
        aero_data: Optional aerodynamic data
        show: Whether to display plots
        save_path: Optional path to save figures
    """
    viz = create_visualizer()
    figs = viz.plot_comprehensive(results, aero_data)

    if save_path:
        for i, fig in enumerate(figs):
            fig.savefig(f"{save_path}_fig{i}.png", dpi=100, bbox_inches="tight")

    if show:
        plt.show()

    return figs
