"""
手机/地面站 TCP 控制客户端

连接船体端（默认 192.168.1.241:9000），接收 state 上报，下发 start/stop 命令。

船端 → 客户端（周期性）:
    {"type":"state","current":{"lat":..,"lon":..},"target":{"lat":..,"lon":..},"heading":..,"running":false}

客户端 → 船端（事件触发）:
    {"type":"command","action":"start","target":{"lat":..,"lon":..}}
    {"type":"command","action":"stop"}

用法:
    python lan_control_server.py
    python lan_control_server.py --host 192.168.1.241 --port 9000
"""

import argparse
import json
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class HullState:
    """最近一次收到的船端状态。"""

    current_lat: float = 0.0
    current_lon: float = 0.0
    target_lat: float = 0.0
    target_lon: float = 0.0
    heading: float = 0.0
    running: bool = False
    updated_at: float = 0.0
    raw: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_message(cls, msg: Dict[str, Any]) -> "HullState":
        current = msg.get("current") or {}
        target = msg.get("target") or {}
        return cls(
            current_lat=float(current.get("lat", 0.0) or 0.0),
            current_lon=float(current.get("lon", 0.0) or 0.0),
            target_lat=float(target.get("lat", 0.0) or 0.0),
            target_lon=float(target.get("lon", 0.0) or 0.0),
            heading=float(msg.get("heading", 0.0) or 0.0),
            running=bool(msg.get("running", False)),
            updated_at=time.time(),
            raw=msg,
        )

    def format_summary(self) -> str:
        age = time.time() - self.updated_at if self.updated_at else float("inf")
        status = "运行中" if self.running else "待机"
        return (
            f"[{status}] "
            f"当前=({self.current_lat:.6f}, {self.current_lon:.6f}) "
            f"目标=({self.target_lat:.6f}, {self.target_lon:.6f}) "
            f"航向={self.heading:.1f}° "
            f"({age:.1f}s 前更新)"
        )


class LanControlClient:
    """手机/地面站 TCP 客户端。"""

    def __init__(self, host: str = "192.168.1.241", port: int = 9000, reconnect_interval: float = 3.0):
        self.host = host
        self.port = port
        self.reconnect_interval = reconnect_interval
        self._lock = threading.Lock()
        self._sock: Optional[socket.socket] = None
        self._latest_state = HullState()
        self._running = True
        self._connected = False

    @property
    def client_connected(self) -> bool:
        return self._connected

    def get_latest_state(self) -> HullState:
        with self._lock:
            return self._latest_state

    def _send_command(self, action: str, target: Optional[Dict[str, float]] = None) -> bool:
        with self._lock:
            sock = self._sock

        if sock is None:
            print("[错误] 未连接船体，无法发送命令")
            return False

        payload: Dict[str, Any] = {"type": "command", "action": action}
        if target is not None:
            payload["target"] = {"lat": float(target["lat"]), "lon": float(target["lon"])}

        line = json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        try:
            sock.sendall(line.encode("utf-8"))
            print(f"[发送] -> {self.host}:{self.port}: {line.strip()}")
            return True
        except OSError as exc:
            print(f"[错误] 发送失败: {exc}")
            self._mark_disconnected()
            return False

    def send_start(self, lat: float, lon: float) -> bool:
        return self._send_command("start", {"lat": lat, "lon": lon})

    def send_stop(self) -> bool:
        return self._send_command("stop")

    def _mark_disconnected(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
        self._connected = False

    def _connect(self) -> bool:
        self._mark_disconnected()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        try:
            sock.connect((self.host, self.port))
        except OSError as exc:
            print(f"[网络] 连接失败 {self.host}:{self.port} -> {exc}")
            try:
                sock.close()
            except OSError:
                pass
            return False

        sock.settimeout(1.0)
        with self._lock:
            self._sock = sock
        self._connected = True
        print(f"[网络] 已连接船体 {self.host}:{self.port}")
        return True

    def _recv_loop(self) -> None:
        buffer = ""
        while self._running and self._connected:
            with self._lock:
                sock = self._sock
            if sock is None:
                break
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                print(f"[网络] 接收异常: {exc}")
                break

            if not chunk:
                print("[网络] 船体关闭连接")
                break

            buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if line:
                    self._process_line(line)

        self._mark_disconnected()

    def _process_line(self, line: str) -> None:
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as exc:
            print(f"[警告] 无效 JSON: {exc} | {line[:120]}")
            return

        msg_type = msg.get("type")
        if msg_type == "state":
            state = HullState.from_message(msg)
            with self._lock:
                self._latest_state = state
            print(f"[状态] {state.format_summary()}")
            return

        if msg_type == "status":
            print(f"[应答] {json.dumps(msg, ensure_ascii=False)}")
            return

        print(f"[消息] 未知 type={msg_type!r}: {json.dumps(msg, ensure_ascii=False)}")

    def _interactive_loop(self) -> None:
        while self._running:
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()

            if cmd in ("quit", "exit", "q"):
                self._running = False
                break
            if cmd == "status":
                state = self.get_latest_state()
                if state.updated_at == 0.0:
                    print("[状态] 尚未收到船端上报")
                else:
                    print(state.format_summary())
                if not self.client_connected:
                    print("[连接] 当前未连接船体")
                continue
            if cmd == "start":
                if len(parts) != 3:
                    print("用法: start <lat> <lon>")
                    continue
                try:
                    lat = float(parts[1])
                    lon = float(parts[2])
                except ValueError:
                    print("[错误] 经纬度必须是数字")
                    continue
                self.send_start(lat, lon)
                continue
            if cmd == "stop":
                self.send_stop()
                continue

            print("未知命令。可用: status | start <lat> <lon> | stop | quit")

    def start(self) -> None:
        print("=" * 60)
        print("  手机/地面站 TCP 控制客户端")
        print("=" * 60)
        print(f"船体地址: {self.host}:{self.port}")
        print()
        print("交互命令:")
        print("  status                         显示最新状态")
        print("  start <lat> <lon>              下发目标并启动")
        print("  stop                           停止导航")
        print("  quit                           退出")
        print("=" * 60)

        def connection_manager() -> None:
            while self._running:
                if not self._connected:
                    if self._connect():
                        threading.Thread(
                            target=self._recv_loop,
                            daemon=True,
                            name="tcp-recv",
                        ).start()
                    else:
                        time.sleep(self.reconnect_interval)
                else:
                    time.sleep(0.5)

        threading.Thread(target=connection_manager, daemon=True, name="connect-loop").start()

        try:
            self._interactive_loop()
        except KeyboardInterrupt:
            print("\n[退出] 用户中断")
        finally:
            self.stop()

    def stop(self) -> None:
        self._running = False
        self._mark_disconnected()
        print("[退出] 客户端已停止")


def main() -> None:
    parser = argparse.ArgumentParser(description="手机/地面站 TCP 控制客户端（连接船体）")
    parser.add_argument("--host", type=str, default="192.168.1.241", help="船体 IP，默认 192.168.1.241")
    parser.add_argument("--port", type=int, default=9000, help="船体端口，默认 9000")
    parser.add_argument("--reconnect", type=float, default=3.0, help="断线重连间隔(秒)")
    args = parser.parse_args()

    client = LanControlClient(host=args.host, port=args.port, reconnect_interval=args.reconnect)
    client.start()


if __name__ == "__main__":
    main()
