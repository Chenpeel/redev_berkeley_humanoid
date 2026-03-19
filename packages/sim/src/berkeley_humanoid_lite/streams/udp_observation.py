from __future__ import annotations

import threading

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

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._receive_forever, daemon=True)
        self._thread.start()

    def _receive_forever(self) -> None:
        from cc.udp import UDP

        udp = UDP(
            (self.listen_host, self.port),
            (self.remote_host, self.port),
        )
        while True:
            robot_observations = udp.recv_numpy(dtype=np.float32)
            if robot_observations is None:
                continue
            if robot_observations.shape != self.buffer.shape:
                raise ValueError(
                    f"UDP 观测长度不匹配: 期望 {self.buffer.shape[0]}, 实际 {robot_observations.shape[0]}"
                )
            self.buffer[:] = robot_observations
