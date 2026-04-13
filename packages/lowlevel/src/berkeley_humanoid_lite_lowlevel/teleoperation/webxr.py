from __future__ import annotations

import copy
import json
import mimetypes
import shutil
import socket
import ssl
import subprocess
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import numpy as np

from berkeley_humanoid_lite_lowlevel.runtime_paths import get_artifacts_dir, resolve_workspace_path
from berkeley_humanoid_lite_lowlevel.teleoperation.input_frame import TeleopHandInput, TeleopInputFrame

WEBXR_DEFAULT_HTTPS_PORT = 8443
WEBXR_DEFAULT_WEBSOCKET_PORT = 8442
DEFAULT_WEBXR_POSITION_SCALE = 1.0

_LEFT_HOME_TRANSLATION = np.array([0.2, 0.2, 0.6], dtype=np.float64)
_RIGHT_HOME_TRANSLATION = np.array([0.2, -0.2, 0.6], dtype=np.float64)

# WebXR 常见坐标是 x=右, y=上, z=朝向用户；当前 teleop/robot 约定是 x=前, y=左, z=上。
_WEBXR_TO_ROBOT_BASIS = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)


@dataclass(slots=True)
class QuestWebXrServerConfig:
    host: str = "0.0.0.0"
    https_port: int = WEBXR_DEFAULT_HTTPS_PORT
    websocket_port: int = WEBXR_DEFAULT_WEBSOCKET_PORT
    position_scale: float = DEFAULT_WEBXR_POSITION_SCALE
    certificate_path: str | None = None
    private_key_path: str | None = None


def get_webxr_ui_dir() -> Path:
    return Path(__file__).resolve().parent / "webxr_ui"


def get_default_certificate_paths() -> tuple[Path, Path]:
    base_dir = get_artifacts_dir() / "teleoperation" / "webxr"
    base_dir.mkdir(parents=True, exist_ok=True)
    return (base_dir / "cert.pem", base_dir / "key.pem")


def get_local_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
            udp_socket.connect(("8.8.8.8", 80))
            return str(udp_socket.getsockname()[0])
    except OSError:
        try:
            return str(socket.gethostbyname(socket.gethostname()))
        except OSError:
            return "127.0.0.1"


def _is_ipv4_address(value: str) -> bool:
    try:
        socket.inet_aton(value)
    except OSError:
        return False
    return True


def _normalize_display_host(host: str) -> str:
    normalized_host = str(host).strip()
    if normalized_host in {"", "0.0.0.0", "::"}:
        return get_local_ip()
    return normalized_host


def _resolve_certificate_paths(
    certificate_path: str | None,
    private_key_path: str | None,
) -> tuple[Path, Path]:
    default_certificate_path, default_private_key_path = get_default_certificate_paths()
    resolved_certificate_path = (
        resolve_workspace_path(certificate_path)
        if certificate_path is not None
        else default_certificate_path
    )
    resolved_private_key_path = (
        resolve_workspace_path(private_key_path)
        if private_key_path is not None
        else default_private_key_path
    )
    resolved_certificate_path.parent.mkdir(parents=True, exist_ok=True)
    resolved_private_key_path.parent.mkdir(parents=True, exist_ok=True)
    return (resolved_certificate_path, resolved_private_key_path)


def _build_subject_alt_names(host: str) -> str:
    alt_names = ["DNS:localhost", "IP:127.0.0.1"]
    local_ip = get_local_ip()
    if local_ip not in {"", "127.0.0.1"}:
        alt_names.append(f"IP:{local_ip}")

    display_host = _normalize_display_host(host)
    if display_host not in {"localhost", "127.0.0.1"}:
        if _is_ipv4_address(display_host):
            alt_names.append(f"IP:{display_host}")
        else:
            alt_names.append(f"DNS:{display_host}")

    deduplicated_alt_names = list(dict.fromkeys(alt_names))
    return ",".join(deduplicated_alt_names)


def ensure_self_signed_certificate(
    *,
    host: str,
    certificate_path: str | None = None,
    private_key_path: str | None = None,
) -> tuple[Path, Path]:
    resolved_certificate_path, resolved_private_key_path = _resolve_certificate_paths(
        certificate_path,
        private_key_path,
    )
    if resolved_certificate_path.exists() and resolved_private_key_path.exists():
        return (resolved_certificate_path, resolved_private_key_path)

    openssl_path = shutil.which("openssl")
    if openssl_path is None:
        raise RuntimeError(
            "Quest WebXR requires HTTPS, but openssl is not available to generate a self-signed certificate."
        )

    subject = "/C=US/ST=Local/L=Local/O=BerkeleyHumanoid/OU=Teleoperation/CN=localhost"
    subject_alt_name = _build_subject_alt_names(host)
    base_command = [
        openssl_path,
        "req",
        "-x509",
        "-newkey",
        "rsa:2048",
        "-keyout",
        str(resolved_private_key_path),
        "-out",
        str(resolved_certificate_path),
        "-sha256",
        "-days",
        "365",
        "-nodes",
        "-subj",
        subject,
    ]

    command_with_alt_name = base_command + ["-addext", f"subjectAltName={subject_alt_name}"]
    try:
        subprocess.run(
            command_with_alt_name,
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError:
        subprocess.run(
            base_command,
            check=True,
            capture_output=True,
            text=True,
        )

    return (resolved_certificate_path, resolved_private_key_path)


def _identity_rotation_matrix() -> np.ndarray:
    return np.eye(3, dtype=np.float64)


def _coerce_xyz(value: object) -> np.ndarray:
    if isinstance(value, Mapping):
        return np.array(
            [
                float(value.get("x", 0.0)),
                float(value.get("y", 0.0)),
                float(value.get("z", 0.0)),
            ],
            dtype=np.float64,
        )
    return np.zeros((3,), dtype=np.float64)


def _coerce_quaternion(value: object) -> np.ndarray:
    if isinstance(value, Mapping):
        quaternion = np.array(
            [
                float(value.get("x", 0.0)),
                float(value.get("y", 0.0)),
                float(value.get("z", 0.0)),
                float(value.get("w", 1.0)),
            ],
            dtype=np.float64,
        )
    else:
        quaternion = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion / norm


def _quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _coerce_pose(value: object) -> np.ndarray | None:
    if value is None:
        return None
    try:
        pose = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if pose.shape != (4, 4):
        return None
    return pose.copy()


def _extract_controller_payload(
    message: Mapping[str, Any],
    *,
    hand: str,
) -> Mapping[str, Any]:
    controllers_payload = message.get("controllers")
    if isinstance(controllers_payload, Mapping):
        controller_payload = controllers_payload.get(hand)
        if isinstance(controller_payload, Mapping):
            return controller_payload

    controller_payload = message.get(f"{hand}Controller")
    if isinstance(controller_payload, Mapping):
        return controller_payload

    controller_payload = message.get(hand)
    if isinstance(controller_payload, Mapping):
        return controller_payload

    return {}


def _build_pose_from_controller_payload(
    controller_payload: Mapping[str, Any],
    *,
    home_translation: np.ndarray,
    position_scale: float,
) -> np.ndarray:
    direct_pose = _coerce_pose(controller_payload.get("pose"))
    if direct_pose is not None:
        return direct_pose

    webxr_position = _coerce_xyz(controller_payload.get("position"))
    webxr_quaternion = _coerce_quaternion(controller_payload.get("quaternion"))
    webxr_rotation = _quaternion_to_rotation_matrix(webxr_quaternion)

    robot_pose = np.eye(4, dtype=np.float64)
    robot_pose[:3, :3] = _WEBXR_TO_ROBOT_BASIS @ webxr_rotation @ _WEBXR_TO_ROBOT_BASIS.T
    robot_pose[:3, 3] = home_translation + position_scale * (_WEBXR_TO_ROBOT_BASIS @ webxr_position)
    return robot_pose


def _build_hand_input_from_controller_payload(
    controller_payload: Mapping[str, Any],
    *,
    home_translation: np.ndarray,
    position_scale: float,
) -> TeleopHandInput:
    pose = _build_pose_from_controller_payload(
        controller_payload,
        home_translation=home_translation,
        position_scale=position_scale,
    )
    button_pressed = bool(
        controller_payload.get("gripActive", controller_payload.get("button_pressed", False))
    )
    trigger = float(controller_payload.get("trigger", 0.0))
    return TeleopHandInput(
        pose=pose,
        button_pressed=button_pressed,
        trigger=trigger,
    )


def build_idle_bridge_data() -> dict[str, dict[str, object]]:
    return TeleopInputFrame(
        left=TeleopHandInput(
            pose=_build_pose_from_controller_payload(
                {},
                home_translation=_LEFT_HOME_TRANSLATION,
                position_scale=0.0,
            ),
            button_pressed=False,
            trigger=0.0,
        ),
        right=TeleopHandInput(
            pose=_build_pose_from_controller_payload(
                {},
                home_translation=_RIGHT_HOME_TRANSLATION,
                position_scale=0.0,
            ),
            button_pressed=False,
            trigger=0.0,
        ),
    ).to_bridge_data()


def build_bridge_data_from_webxr_message(
    message: Mapping[str, Any],
    *,
    position_scale: float = DEFAULT_WEBXR_POSITION_SCALE,
) -> dict[str, dict[str, object]]:
    if position_scale <= 0.0:
        raise ValueError("position_scale must be positive")

    left_payload = _extract_controller_payload(message, hand="left")
    right_payload = _extract_controller_payload(message, hand="right")
    return TeleopInputFrame(
        left=_build_hand_input_from_controller_payload(
            left_payload,
            home_translation=_LEFT_HOME_TRANSLATION,
            position_scale=position_scale,
        ),
        right=_build_hand_input_from_controller_payload(
            right_payload,
            home_translation=_RIGHT_HOME_TRANSLATION,
            position_scale=position_scale,
        ),
    ).to_bridge_data()


class _WebUiServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address: tuple[str, int], handler_class: type[BaseHTTPRequestHandler], bridge: object):
        super().__init__(server_address, handler_class)
        self.bridge = bridge


class _WebUiRequestHandler(BaseHTTPRequestHandler):
    server: _WebUiServer

    def log_message(self, format: str, *args: object) -> None:
        del format, args

    def _send_bytes(
        self,
        *,
        status: HTTPStatus,
        content_type: str,
        payload: bytes,
    ) -> None:
        self.send_response(int(status))
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:
        parsed_path = urlparse(self.path)
        request_path = parsed_path.path
        bridge = self.server.bridge
        if request_path in {"", "/"}:
            payload = bridge.render_ui_index()
            self._send_bytes(
                status=HTTPStatus.OK,
                content_type="text/html; charset=utf-8",
                payload=payload,
            )
            return

        if request_path == "/api/status":
            payload = json.dumps(bridge.status_snapshot(), ensure_ascii=True).encode("utf-8")
            self._send_bytes(
                status=HTTPStatus.OK,
                content_type="application/json; charset=utf-8",
                payload=payload,
            )
            return

        static_path = bridge.resolve_static_asset(request_path)
        if static_path is None:
            self.send_error(int(HTTPStatus.NOT_FOUND), "Not Found")
            return

        content_type = mimetypes.guess_type(static_path.name)[0] or "application/octet-stream"
        self._send_bytes(
            status=HTTPStatus.OK,
            content_type=content_type,
            payload=static_path.read_bytes(),
        )


class QuestWebXrBridgeServer:
    def __init__(self, config: QuestWebXrServerConfig) -> None:
        self.config = config
        self._display_host = _normalize_display_host(config.host)
        self._certificate_path: Path | None = None
        self._private_key_path: Path | None = None
        self._http_server: _WebUiServer | None = None
        self._http_thread: threading.Thread | None = None
        self._websocket_server: object | None = None
        self._clients: set[object] = set()
        self._state_lock = threading.Lock()
        self._latest_bridge_data = build_idle_bridge_data()
        self._message_count = 0
        self._last_message_time: float | None = None

    @property
    def page_url(self) -> str:
        return f"https://{self._display_host}:{self.config.https_port}"

    @property
    def websocket_url(self) -> str:
        return f"wss://{self._display_host}:{self.config.websocket_port}"

    @property
    def latest_bridge_data(self) -> dict[str, dict[str, object]]:
        with self._state_lock:
            return copy.deepcopy(self._latest_bridge_data)

    def status_snapshot(self) -> dict[str, object]:
        with self._state_lock:
            return {
                "pageUrl": self.page_url,
                "websocketUrl": self.websocket_url,
                "clients": len(self._clients),
                "messageCount": self._message_count,
                "lastMessageTime": self._last_message_time,
            }

    def render_ui_index(self) -> bytes:
        index_path = get_webxr_ui_dir() / "index.html"
        html = index_path.read_text(encoding="utf-8")
        rendered_html = (
            html.replace("__PAGE_URL__", self.page_url)
            .replace("__WS_URL__", self.websocket_url)
            .replace("__DISPLAY_HOST__", self._display_host)
        )
        return rendered_html.encode("utf-8")

    def resolve_static_asset(self, request_path: str) -> Path | None:
        normalized_path = request_path.lstrip("/")
        candidate_path = (get_webxr_ui_dir() / normalized_path).resolve()
        webxr_ui_dir = get_webxr_ui_dir().resolve()
        if webxr_ui_dir not in candidate_path.parents and candidate_path != webxr_ui_dir:
            return None
        if not candidate_path.is_file():
            return None
        return candidate_path

    async def start(self) -> None:
        self._certificate_path, self._private_key_path = ensure_self_signed_certificate(
            host=self.config.host,
            certificate_path=self.config.certificate_path,
            private_key_path=self.config.private_key_path,
        )
        self._start_https_server()
        await self._start_websocket_server()

    async def stop(self) -> None:
        clients = list(self._clients)
        for client in clients:
            try:
                await client.close(code=1001, reason="Server shutdown")
            except Exception:
                pass

        if self._websocket_server is not None:
            self._websocket_server.close()
            await self._websocket_server.wait_closed()
            self._websocket_server = None

        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
            self._http_server = None

        if self._http_thread is not None:
            self._http_thread.join(timeout=2.0)
            self._http_thread = None

    def _create_ssl_context(self) -> ssl.SSLContext:
        if self._certificate_path is None or self._private_key_path is None:
            raise RuntimeError("SSL certificate paths are not initialized")
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_context.load_cert_chain(
            certfile=str(self._certificate_path),
            keyfile=str(self._private_key_path),
        )
        return ssl_context

    def _start_https_server(self) -> None:
        ssl_context = self._create_ssl_context()
        self._http_server = _WebUiServer(
            (self.config.host, self.config.https_port),
            _WebUiRequestHandler,
            self,
        )
        self._http_server.socket = ssl_context.wrap_socket(
            self._http_server.socket,
            server_side=True,
        )
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            daemon=True,
        )
        self._http_thread.start()

    async def _start_websocket_server(self) -> None:
        import websockets

        ssl_context = self._create_ssl_context()
        self._websocket_server = await websockets.serve(
            self._websocket_handler,
            self.config.host,
            self.config.websocket_port,
            ssl=ssl_context,
        )

    async def _websocket_handler(self, websocket: object, path: str | None = None) -> None:
        del path
        self._clients.add(websocket)
        try:
            async for raw_message in websocket:
                bridge_data = self._decode_websocket_message(raw_message)
                with self._state_lock:
                    self._latest_bridge_data = bridge_data
                    self._message_count += 1
                    self._last_message_time = time.time()
        finally:
            self._clients.discard(websocket)

    def _decode_websocket_message(self, raw_message: str | bytes) -> dict[str, dict[str, object]]:
        if isinstance(raw_message, bytes):
            decoded_message = raw_message.decode("utf-8")
        else:
            decoded_message = raw_message
        payload = json.loads(decoded_message)
        if not isinstance(payload, Mapping):
            raise ValueError("WebXR payload must be a JSON object")
        return build_bridge_data_from_webxr_message(
            payload,
            position_scale=self.config.position_scale,
        )
