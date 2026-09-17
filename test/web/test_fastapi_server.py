"""
Tests for FastAPI 3D Web Visualizer and WebSocket streaming.
"""

from __future__ import annotations

import json

import pytest
from fastapi.testclient import TestClient

from ap_pnc.tui.orchestrator_bridge import OrchestratorBridge
from ap_pnc.web.app import create_app


@pytest.fixture(scope="module")
def web_test_client():
    """Create a test client with a headless OrchestratorBridge."""
    bridge = OrchestratorBridge(initial_scenario="open_circle")
    app = create_app(bridge)
    with TestClient(app) as client:
        yield client, bridge


def test_static_index(web_test_client):
    """Test that the index.html is served properly."""
    client, _ = web_test_client
    response = client.get("/")
    assert response.status_code == 200
    assert "text/html" in response.headers["content-type"]
    assert "AP-PnC Tailsitter" in response.text
    assert "Visualizer" in response.text


def test_api_snapshot(web_test_client):
    """Test that the /api/snapshot endpoint returns valid telemetry."""
    client, _ = web_test_client
    response = client.get("/api/snapshot")
    assert response.status_code == 200
    data = response.json()
    assert "current_pos" in data
    assert "ref_pos" in data
    assert "pred_trajectory" in data
    assert "ref_trajectory" in data
    assert "sim_time" in data
    assert len(data["current_pos"]) == 3
    assert len(data["pred_trajectory"]) > 0


def test_api_actions(web_test_client):
    """Test REST action dispatches."""
    client, bridge = web_test_client

    # 1. Toggle play
    res = client.post("/api/action", json={"action": "toggle_play"})
    assert res.status_code == 200
    assert bridge.is_running is True

    # 2. Pause
    res = client.post("/api/action", json={"action": "pause"})
    assert res.status_code == 200
    assert bridge.is_running is False

    # 3. Step
    initial_step = bridge.last_snapshot.step_idx
    res = client.post("/api/action", json={"action": "step"})
    assert res.status_code == 200
    assert bridge.last_snapshot.step_idx == initial_step + 1

    # 4. Set wind
    res = client.post(
        "/api/action", json={"action": "set_wind", "wind_side": 2.0, "wind_longitudinal": 1.5}
    )
    assert res.status_code == 200
    assert bridge.wind_side == 2.0
    assert bridge.wind_longitudinal == 1.5

    # 5. Set speed
    res = client.post("/api/action", json={"action": "set_speed", "speed": 14.0})
    assert res.status_code == 200
    assert bridge.target_speed == 14.0

    # 6. Reset
    res = client.post("/api/action", json={"action": "reset"})
    assert res.status_code == 200
    assert bridge.last_snapshot.step_idx == 0


def test_websocket_streaming(web_test_client):
    """Test bi-directional WebSocket telemetry streaming and action execution."""
    client, _ = web_test_client

    with client.websocket_connect("/ws") as websocket:
        # Receive initial telemetry snapshot
        msg = websocket.receive_text()
        data = json.loads(msg)
        assert "current_pos" in data
        assert "sim_time" in data

        # Send action via WebSocket
        websocket.send_text(json.dumps({"action": "step"}))
        # Receive updated state broadcast
        update_msg = websocket.receive_text()
        update_data = json.loads(update_msg)
        assert update_data["step_idx"] >= 0


def test_openapi_docs(web_test_client):
    """Test OpenAPI schema and Swagger UI availability."""
    client, _ = web_test_client
    res_docs = client.get("/docs")
    assert res_docs.status_code == 200
    res_json = client.get("/openapi.json")
    assert res_json.status_code == 200
    schema = res_json.json()
    assert "paths" in schema
    assert "/api/action" in schema["paths"]
    assert "/api/snapshot" in schema["paths"]
