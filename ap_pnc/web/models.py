"""
Pydantic data models for the AP-PnC FastAPI Web Visualizer.
"""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, Field


class ActionRequest(BaseModel):
    """Request payload for dispatching orchestrator actions."""

    action: str = Field(
        ...,
        description="Action to perform: play, pause, toggle_play, step, reset, set_scenario, set_speed, set_wind, sync_config",
    )
    scenario: str | None = Field(default=None, description="Scenario key for set_scenario")
    speed: float | None = Field(default=None, description="Target velocity [m/s] for set_speed")
    wind_side: float | None = Field(
        default=None, description="Crosswind disturbance [m/s] for set_wind"
    )
    wind_longitudinal: float | None = Field(
        default=None, description="Longitudinal wind disturbance [m/s] for set_wind"
    )


class ActionResponse(BaseModel):
    """Response returned after executing an action."""

    status: str = Field(default="ok", description="Status code or message")
    message: str = Field(default="OK", description="Human-readable result summary")
    snapshot: dict[str, Any] | None = Field(default=None, description="Updated snapshot state")


class ConfigSyncResponse(BaseModel):
    """Response returned when applying writes to YAML configs."""

    status: str
    details: str
