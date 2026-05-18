"""
GPS 往返任务：与 heading_lock_control / GPSNavigationController 默认一致的控制逻辑
（前进车头校准、HeadingLock 默认周期与死区、每个导航周期重算方位角）-> 驶向目的地
-> 到达后停电机并单次倒计时停泊 -> 返回启动点。

可调参数见下方常量与命令行参数。磁力/辅助融合可通过 MagneticAssistProvider 预留接口接入。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional, Protocol, runtime_checkable

# 项目根目录 (Hull_control_ststem)，与 gps_navigation_controller 一致
_Gps_dir = os.path.dirname(os.path.abspath(__file__))
_root_path = os.path.dirname(_Gps_dir)
if _root_path not in sys.path:
    sys.path.insert(0, _root_path)

from Gps.gps_navigation_controller import GPSNavigationController, NavigationState
from compass import OutputMode

# ---------------------------------------------------------------------------
# 任务默认（可在命令行覆盖到达阈值等；此处便于后期统一改默认值）
# ---------------------------------------------------------------------------
ARRIVAL_THRESHOLD_M = 1.0
DWELL_SEC = 5.0
# 与 gps_navigation_controller 默认一致：0 表示每个导航周期都按当前 GPS 重算方位角
# （heading_lock_control 内嵌 GPSNavigationController 未改 bearing 时即此行为）
BEARING_RECOMPUTE_SEC = 5.0

# ---------------------------------------------------------------------------
# PID / 航向死区：与 heading_lock_control.HeadingLockController.__init__ 默认一致。
# 只改本段常量即可作用到本脚本的 heading_lock_config。
# ---------------------------------------------------------------------------
PID_KP = 0.0005
PID_KI = 0.02
PID_KD = 0.5
DEVIATION_THRESHOLD_DEG = 5.0

# 与 HeadingLockController 默认 update_interval 一致（影响 PID 的 dt）
HEADING_LOCK_UPDATE_INTERVAL_SEC = 0.02

# 导航线程 on_state_update 打印节流（秒）；0 表示不节流（每个导航周期都打印）
NAV_TELEMETRY_INTERVAL_SEC = 0.1


def _make_nav_telemetry_handler(min_interval_sec: float):
    """由导航线程回调：打印目标/当前经纬度、目标航线角、当前航向。"""
    last_print = 0.0

    def _on_state(state: NavigationState) -> None:
        nonlocal last_print
        now = time.time()
        if min_interval_sec > 0.0 and (now - last_print) < min_interval_sec:
            return
        last_print = now
        print(
            '[导航] '
            f'目标 {state.target_lat:.7f},{state.target_lon:.7f} | '
            f'当前 {state.current_lat:.7f},{state.current_lon:.7f} | '
            f'目标航线角 {state.target_heading:.1f}° | '
            f'当前航向 {state.current_heading:.1f}°',
            flush=True,
        )

    return _on_state


@runtime_checkable
class MagneticAssistProvider(Protocol):
    """预留：后期接入磁力计/多传感器融合时，在停泊期周期回调。"""

    def on_dwell_tick(self, navigation: GPSNavigationController, elapsed_in_dwell: float) -> None:
        ...


class _NoOpMagneticAssist:
    def on_dwell_tick(self, navigation: GPSNavigationController, elapsed_in_dwell: float) -> None:
        return None


def _snapshot_home(nav: GPSNavigationController) -> tuple[float, float]:
    pos = nav.get_current_position()
    if pos is not None:
        return float(pos[0]), float(pos[1])
    st = nav.get_state()
    return float(st.current_lat), float(st.current_lon)


def _emergency_brake_motors(nav: GPSNavigationController) -> None:
    """意外断连时尽快切断差速输出（再调用 nav.stop() 做完整清理）。"""
    hl = getattr(nav, '_heading_lock', None)
    driver = getattr(hl, '_driver', None) if hl is not None else None
    if driver is not None:
        try:
            driver.stop()
            print('[往返任务] 紧急刹车: 电机已停止')
        except Exception as exc:
            print(f'[往返任务] 紧急刹车调用失败: {exc}')


def _shutdown_on_ctrl_c(nav: GPSNavigationController) -> None:
    """Ctrl+C：先刹车再释放导航/GPS/罗盘资源。"""
    print('\n[往返任务] 收到 Ctrl+C，正在刹车并停止导航...')
    _emergency_brake_motors(nav)
    try:
        nav.stop()
    except Exception as exc:
        print(f'[往返任务] 停止导航时异常（可忽略）: {exc}')


def _wait_arrival_with_link_watch(
    nav: GPSNavigationController,
    *,
    timeout: float | None,
    poll_sec: float = 0.25,
) -> bool:
    """
    等待到达；若 GPS 串口/读线程异常或导航线程崩溃，则紧急刹车并 stop。
    正常到达时返回 True；超时、主动停止、链路异常返回 False。
    """
    start = time.time()
    while nav._is_running:
        if nav._is_arrived:
            return True
        if timeout is not None and (time.time() - start) > timeout:
            print('[GPS导航] 等待到达超时')
            return False
        if not nav.is_gps_link_healthy():
            print('[往返任务] GPS 连接异常（串口关闭或读取线程结束），执行紧急刹车')
            _emergency_brake_motors(nav)
            nav.stop()
            return False
        if nav._nav_thread is not None and not nav.is_navigation_thread_alive() and not nav._is_arrived:
            print('[往返任务] 导航线程异常结束，执行紧急刹车')
            _emergency_brake_motors(nav)
            nav.stop()
            return False
        time.sleep(poll_sec)
    return False


def run_roundtrip_mission(
    dest_lat: float,
    dest_lon: float,
    *,
    compass_port: str = '/dev/ttyS0',
    gps_port: str = '/dev/ttyS1',
    # 与 heading_lock_control._init_gps_navigation 默认一致（硬件不同可用 CLI 覆盖）
    gps_baudrate: int = 38400,
    arrival_threshold_m: float = ARRIVAL_THRESHOLD_M,
    dwell_sec: float = DWELL_SEC,
    bearing_recompute_sec: float = BEARING_RECOMPUTE_SEC,
    base_speed: int = 80,
    magnetic_assist: Optional[MagneticAssistProvider] = None,
    telemetry_interval_sec: float = NAV_TELEMETRY_INTERVAL_SEC,
) -> None:
    """
    执行：前进车头校准（与默认 GPS 导航 / heading_lock GPS 流程一致）
    -> 去目的地 -> 停泊(停电机, 单次 dwell 倒计时) -> 回启动经纬度。
    """
    assist: MagneticAssistProvider = magnetic_assist or _NoOpMagneticAssist()

    heading_lock_config = {
        'base_speed': base_speed,
        'deviation_threshold': DEVIATION_THRESHOLD_DEG,
        'pid_kp': PID_KP,
        'pid_ki': PID_KI,
        'pid_kd': PID_KD,
        'update_interval': HEADING_LOCK_UPDATE_INTERVAL_SEC,
        'min_turn_strength': 0.05,
        'max_turn_strength': 0.2,
        'compass_mode': OutputMode.AUTO_50HZ,
        'use_heading_wrap': True,
    }

    nav = GPSNavigationController(
        target_lat=dest_lat,
        target_lon=dest_lon,
        compass_port=compass_port,
        gps_port=gps_port,
        gps_baudrate=gps_baudrate,
        arrival_threshold=arrival_threshold_m,
        calibration_duration=2.0,
        calibration_speed=base_speed,
        update_interval=HEADING_LOCK_UPDATE_INTERVAL_SEC,
        heading_lock_config=heading_lock_config,
        use_forward_heading_calibration=True,
        bearing_recompute_interval=bearing_recompute_sec,
    )

    try:
        print('\n' + '=' * 60)
        print('  GPS 往返任务')
        print('=' * 60)
        print(f'  到达阈值: {arrival_threshold_m} m')
        print(f'  停泊时长: {dwell_sec} s (自首次到达起连续计时，不重置)')
        if bearing_recompute_sec > 0:
            print(f'  方位角重算: 每 {bearing_recompute_sec} s')
        else:
            print('  方位角重算: 每个导航周期（与默认 GPSNavigationController 一致）')
        print('=' * 60)

        if not nav.initialize():
            print('[往返任务] 初始化失败')
            return

        nav.register_callback('on_state_update', _make_nav_telemetry_handler(telemetry_interval_sec))
        print('[往返任务] 已启用导航状态实时打印（目标/当前经纬度、目标航线角、当前航向）')

        home_lat, home_lon = _snapshot_home(nav)
        print(f'[往返任务] 记录返航点(启动点): ({home_lat:.7f}, {home_lon:.7f})')

        # ---------- 航段 1：驶向目的地 ----------
        nav.navigate()
        if not _wait_arrival_with_link_watch(nav, timeout=None):
            print('[往返任务] 未能到达目的地或已停止')
            if nav._is_running:
                nav.stop()
            return

        if nav._nav_thread is not None:
            nav._nav_thread.join(timeout=30.0)

        # 停泊：停电机，从首次到达起固定倒计时
        if nav._heading_lock and nav._heading_lock._driver:
            nav._heading_lock._driver.stop()
            print('[往返任务] 已到达，停泊期内电机保持停止')

        dwell_deadline = time.time() + dwell_sec
        print(f'[往返任务] 停泊倒计时 {dwell_sec:.0f}s 开始（水面漂移不重置计时）')
        while time.time() < dwell_deadline:
            assist.on_dwell_tick(nav, elapsed_in_dwell=dwell_sec - (dwell_deadline - time.time()))
            time.sleep(0.25)

        # ---------- 航段 2：返回启动点 ----------
        print(f'[往返任务] 返航目标: ({home_lat:.7f}, {home_lon:.7f})')
        nav.set_target(home_lat, home_lon)
        nav.navigate()
        if not _wait_arrival_with_link_watch(nav, timeout=None):
            print('[往返任务] 返航未完成或已停止')
            if nav._is_running:
                nav.stop()
        else:
            if nav._nav_thread is not None:
                nav._nav_thread.join(timeout=30.0)
            if nav._heading_lock and nav._heading_lock._driver:
                nav._heading_lock._driver.stop()
            print('[往返任务] 已到达返航点')

        nav.stop()
        print('[往返任务] 结束')
    except KeyboardInterrupt:
        _shutdown_on_ctrl_c(nav)
        raise SystemExit(130) from None


def main() -> None:
    parser = argparse.ArgumentParser(
        description='GPS 往返任务（与 heading_lock 默认：前进校准 / 每周期方位角 / 停泊 / 返航）'
    )
    parser.add_argument('--dest-lat', type=float, required=True, help='目的地纬度')
    parser.add_argument('--dest-lon', type=float, required=True, help='目的地经度')
    parser.add_argument('--compass-port', type=str, default='/dev/ttyS0', help='罗盘串口')
    parser.add_argument('--gps-port', type=str, default='/dev/ttyS1', help='GPS 串口')
    parser.add_argument('--gps-baudrate', type=int, default=115200, help='GPS 波特率（与 heading_lock GPS 默认一致）')
    parser.add_argument('--threshold', type=float, default=ARRIVAL_THRESHOLD_M, help='到达阈值(米)')
    parser.add_argument('--dwell', type=float, default=DWELL_SEC, help='到达后停泊秒数')
    parser.add_argument('--bearing-interval', type=float, default=BEARING_RECOMPUTE_SEC,
                        help='方位角重算周期(秒)，0=每导航周期重算（默认，与 heading_lock 内嵌 GPS 一致）')
    parser.add_argument('--speed', type=int, default=80, help='基础速度 0-100')
    parser.add_argument(
        '--telemetry-interval',
        type=float,
        default=NAV_TELEMETRY_INTERVAL_SEC,
        help='导航状态打印最小间隔(秒)，0=每个导航周期都打印',
    )

    args = parser.parse_args()

    run_roundtrip_mission(
        args.dest_lat,
        args.dest_lon,
        compass_port=args.compass_port,
        gps_port=args.gps_port,
        gps_baudrate=args.gps_baudrate,
        arrival_threshold_m=args.threshold,
        dwell_sec=args.dwell,
        bearing_recompute_sec=args.bearing_interval,
        base_speed=args.speed,
        magnetic_assist=None,
        telemetry_interval_sec=args.telemetry_interval,
    )


if __name__ == '__main__':
    main()


#python3 Gps/gps_roundtrip_mission.py --dest-lat 纬度 --dest-lon 经度