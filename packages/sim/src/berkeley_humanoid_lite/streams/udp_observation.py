from __future__ import annotations

import threading
from typing import Any

import numpy as np


class UdpObservationReceiver:
    """持续接收策略观测 UDP 数据。"""

    def __init__(
        self,
        *,
        listen_host: str,
        remote_host: str,
        port: int,
        observation_size: int,
    ) -> None:
        self.listen_host = listen_host
        self.remote_host = remote_host
        self.port = int(port)
        self.buffer = np.zeros((observation_size,), dtype=np.float32)
        self._thread: threading.Thread | None = None
        self._stopped = threading.Event()
        self._udp: Any | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stopped.clear()
        self._thread = threading.Thread(target=self._receive_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stopped.set()
        udp = self._udp
        if udp is None:
            return
        stop_method = getattr(udp, "stop", None)
        if callable(stop_method):
            stop_method()

    def close(self) -> None:
        self.stop()

    def _receive_forever(self) -> None:
        from cc.udp import UDP

        udp = UDP(
            (self.listen_host, self.port),
            (self.remote_host, self.port),
        )
        self._udp = udp
        try:
            while not self._stopped.is_set():
                robot_observations = udp.recv_numpy(dtype=np.float32)
                if robot_observations is None:
                    continue
                if robot_observations.shape != self.buffer.shape:
                    raise ValueError(
                        f"UDP 观测长度不匹配: 期望 {self.buffer.shape[0]}, 实际 {robot_observations.shape[0]}"
                    )
                self.buffer[:] = robot_observations
        finally:
            self._udp = None
            stop_method = getattr(udp, "stop", None)
            if callable(stop_method):
                stop_method()
