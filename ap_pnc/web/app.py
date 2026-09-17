"""
FastAPI application for the AP-PnC 3D Web Visualizer & Orchestrator.
"""

from __future__ import annotations

import asyncio
import json
import logging
from collections.abc import AsyncGenerator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from ap_pnc.tui.orchestrator_bridge import OrchestratorBridge
from ap_pnc.web.models import ActionRequest, ActionResponse, ConfigSyncResponse

logger = logging.getLogger("ap_pnc.web")


class ConnectionManager:
    """Manages active WebSocket connections and broadcasts."""

    def __init__(self) -> None:
        self.active_connections: set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        async with self._lock:
            self.active_connections.add(websocket)

    async def disconnect(self, websocket: WebSocket) -> None:
        async with self._lock:
            self.active_connections.discard(websocket)

    async def broadcast(self, message: dict) -> None:
        async with self._lock:
            dead_connections = set()
            payload = json.dumps(message)
            for connection in self.active_connections:
                try:
                    await connection.send_text(payload)
                except Exception:
                    dead_connections.add(connection)
            for dead in dead_connections:
                self.active_connections.discard(dead)


def create_app(bridge: OrchestratorBridge) -> FastAPI:
    """Create and configure the FastAPI application."""
    manager = ConnectionManager()
    static_dir = Path(__file__).resolve().parent / "static"
    step_interval = bridge.sim_dt  # 0.02s

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
        # Background simulation and broadcast task
        running = True

        async def simulation_loop() -> None:
            while running:
                start_t = asyncio.get_event_loop().time()
                if bridge.is_running:
                    try:
                        snap = await asyncio.to_thread(bridge.step)
                        if snap.is_complete:
                            bridge.is_running = False
                    except Exception as e:
                        logger.error("Simulation step error: %s", e)
                        bridge.is_running = False

                snap_dict = await asyncio.to_thread(bridge.get_snapshot_dict)
                if snap_dict is not None and manager.active_connections:
                    await manager.broadcast(snap_dict)

                elapsed = asyncio.get_event_loop().time() - start_t
                sleep_time = max(0.005, step_interval - elapsed)
                await asyncio.sleep(sleep_time)

        loop_task = asyncio.create_task(simulation_loop())
        yield
        running = False
        loop_task.cancel()
        try:
            await loop_task
        except asyncio.CancelledError:
            pass

    app = FastAPI(
        title="AP-PnC Tailsitter 3D Visualizer",
        description="High-fidelity 3D simulation visualization and orchestration API for AP-PnC.",
        version="0.1.0",
        lifespan=lifespan,
    )

    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    if static_dir.exists():
        app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

    @app.get("/", include_in_schema=False)
    async def get_index() -> FileResponse:
        index_file = static_dir / "index.html"
        if not index_file.exists():
            raise HTTPException(status_code=404, detail="index.html not found")
        return FileResponse(index_file)

    @app.get("/api/snapshot", response_model=dict)
    async def get_snapshot() -> dict:
        snap_dict = await asyncio.to_thread(bridge.get_snapshot_dict)
        if snap_dict is None:
            raise HTTPException(status_code=503, detail="No snapshot available")
        return snap_dict

    @app.post("/api/action", response_model=ActionResponse)
    async def post_action(req: ActionRequest) -> ActionResponse:
        action = req.action
        msg = "OK"

        if action == "play":
            bridge.is_running = True
        elif action == "pause":
            bridge.is_running = False
        elif action == "toggle_play":
            bridge.is_running = not bridge.is_running
        elif action == "step":
            bridge.is_running = False
            await asyncio.to_thread(bridge.step)
        elif action == "reset":
            bridge.is_running = False
            await asyncio.to_thread(bridge.reset)
        elif action == "set_scenario":
            if req.scenario:
                await asyncio.to_thread(bridge.switch_scenario, req.scenario)
        elif action == "set_speed":
            if req.speed is not None:
                await asyncio.to_thread(bridge.set_speed, req.speed)
        elif action == "set_wind":
            wind_side = req.wind_side if req.wind_side is not None else bridge.wind_side
            wind_long = (
                req.wind_longitudinal
                if req.wind_longitudinal is not None
                else bridge.wind_longitudinal
            )
            await asyncio.to_thread(bridge.set_wind, wind_side, wind_long)
        elif action == "sync_config":
            msg = await asyncio.to_thread(bridge.apply_writes_to_config)
        else:
            raise HTTPException(status_code=400, detail=f"Unknown action: {action}")

        snap = await asyncio.to_thread(bridge.get_snapshot_dict)
        if snap is not None:
            await manager.broadcast(snap)

        return ActionResponse(status="ok", message=msg, snapshot=snap)

    @app.post("/api/sync-config", response_model=ConfigSyncResponse)
    async def sync_config() -> ConfigSyncResponse:
        msg = await asyncio.to_thread(bridge.apply_writes_to_config)
        return ConfigSyncResponse(status="ok", details=msg)

    @app.websocket("/ws")
    async def websocket_endpoint(websocket: WebSocket) -> None:
        await manager.connect(websocket)
        # Send initial snapshot immediately
        snap = await asyncio.to_thread(bridge.get_snapshot_dict)
        if snap is not None:
            await websocket.send_text(json.dumps(snap))

        try:
            while True:
                data_text = await websocket.receive_text()
                try:
                    data = json.loads(data_text)
                    action = data.get("action")
                    if action:
                        req = ActionRequest(**data)
                        await post_action(req)
                except Exception as e:
                    logger.warning("Error processing websocket message: %s", e)
        except WebSocketDisconnect:
            await manager.disconnect(websocket)

    return app
