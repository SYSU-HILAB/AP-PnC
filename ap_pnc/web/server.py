"""
Uvicorn runner for the AP-PnC FastAPI 3D Web Visualizer.
"""

from __future__ import annotations

import threading
import webbrowser

import uvicorn

from ap_pnc.tui.orchestrator_bridge import OrchestratorBridge
from ap_pnc.web.app import create_app


def run_web_server(
    bridge: OrchestratorBridge,
    host: str = "127.0.0.1",
    port: int = 8080,
    open_browser: bool = True,
) -> None:
    """Run the FastAPI 3D Web visualizer server with Uvicorn."""
    app = create_app(bridge)
    url = f"http://{'localhost' if host in ('0.0.0.0', '127.0.0.1') else host}:{port}"
    print(f"\n🚀 AP-PnC 3D Web Simulation Visualizer running at: {url}")
    print(f"📖 OpenAPI Swagger docs available at: {url}/docs")
    print("Press Ctrl+C to stop the server.\n")

    if open_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()

    config = uvicorn.Config(
        app=app,
        host=host,
        port=port,
        log_level="warning",
        access_log=False,
    )
    server = uvicorn.Server(config)
    server.run()
