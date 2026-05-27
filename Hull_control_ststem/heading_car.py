"""
航向锁入口别名：实现与 CLI 均在 heading_lock_control。

用法:
    python3 heading_car.py --esc ...
    python3 heading_lock_control.py --esc ...
"""
import runpy
from pathlib import Path

if __name__ == "__main__":
    runpy.run_path(
        str(Path(__file__).resolve().parent / "heading_lock_control.py"),
        run_name="__main__",
    )
