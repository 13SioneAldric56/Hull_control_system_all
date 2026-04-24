#!/usr/bin/env python3
"""
DDM350B/DDM360B 三维姿态实时可视化

三种数据并列对比：原始数据、卡尔曼滤波、消抖滤波

使用方法:
    python example_visualize.py -p /dev/ttyS0

按 Ctrl+C 退出程序
"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import matplotlib.animation as animation

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

from ddm350b import DDM350B, OutputMode


class DebounceFilter:
    """指数移动平均消抖滤波器"""

    def __init__(self, alpha: float = 0.2):
        self.alpha = alpha
        self._initialized = False
        self._roll = 0.0
        self._pitch = 0.0
        self._heading = 0.0

    def update(self, roll: float, pitch: float, heading: float):
        if not self._initialized:
            self._roll = roll
            self._pitch = pitch
            self._heading = heading
            self._initialized = True
            return roll, pitch, heading

        self._roll = self.alpha * roll + (1 - self.alpha) * self._roll
        self._pitch = self.alpha * pitch + (1 - self.alpha) * self._pitch

        diff = heading - self._heading
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        self._heading += self.alpha * diff

        return self._roll, self._pitch, self._heading


class AttitudeVisualizer:
    """三维姿态可视化器"""

    def __init__(self, max_points: int = 200):
        self.max_points = max_points

        # 数据缓冲区
        self.raw_buffer = deque(maxlen=max_points)
        self.kalman_buffer = deque(maxlen=max_points)
        self.debounce_buffer = deque(maxlen=max_points)

        # 创建图形
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(16, 10), facecolor='#1a1a2e')
        self.fig.suptitle('DDM350B 3D Attitude Real-time Visualization', fontsize=14, color='white')

        gs = GridSpec(2, 4, figure=self.fig, hspace=0.35, wspace=0.3)

        # 第一行：三个二维曲线图
        self.ax_roll = self.fig.add_subplot(gs[0, 0])
        self.ax_pitch = self.fig.add_subplot(gs[0, 1])
        self.ax_heading = self.fig.add_subplot(gs[0, 2])
        self.ax_stats = self.fig.add_subplot(gs[0, 3])

        # 第二行：三个三维姿态图
        self.ax_raw_3d = self.fig.add_subplot(gs[1, 0], projection='3d')
        self.ax_kal_3d = self.fig.add_subplot(gs[1, 1], projection='3d')
        self.ax_deb_3d = self.fig.add_subplot(gs[1, 2], projection='3d')
        self.ax_legend = self.fig.add_subplot(gs[1, 3])

        self._setup_axes()

        self.ani = None
        self.sample_count = 0
        self.last_update = 0

    def _setup_axes(self):
        """设置坐标轴样式"""
        for ax in [self.ax_roll, self.ax_pitch, self.ax_heading]:
            ax.set_facecolor('#16213e')
            ax.grid(True, alpha=0.3)
            ax.tick_params(colors='white')

        for ax in [self.ax_raw_3d, self.ax_kal_3d, self.ax_deb_3d]:
            ax.set_facecolor('#16213e')
            ax.tick_params(colors='white')
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_zticks([])

        self.ax_stats.set_facecolor('#16213e')
        self.ax_stats.axis('off')

        self.ax_legend.axis('off')
        self.ax_legend.set_facecolor('#16213e')

        # 使用 bbox 创建带边框的文本框
        legend_props = dict(boxstyle='round,pad=0.5', facecolor='#16213e', edgecolor='white', alpha=0.9)

        self.ax_legend.text(0.5, 0.95, '数据类型说明', fontsize=12, fontweight='bold',
                          color='yellow', ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.82, '黄色 - 原始数据', fontsize=10, color='yellow',
                          ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.70, '蓝色 - 卡尔曼滤波', fontsize=10, color='cornflowerblue',
                          ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.58, '绿色 - 消抖滤波', fontsize=10, color='limegreen',
                          ha='center', va='top', transform=self.ax_legend.transAxes)

        self.ax_legend.text(0.5, 0.42, '三维模型说明', fontsize=12, fontweight='bold',
                          color='cyan', ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.29, '红色箭头 - Roll (X轴)', fontsize=10, color='red',
                          ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.17, '绿色箭头 - Pitch (Y轴)', fontsize=10, color='green',
                          ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.05, '蓝色箭头 - Heading (Z轴)', fontsize=10, color='blue',
                          ha='center', va='top', transform=self.ax_legend.transAxes)

    def _rotation_matrix(self, roll, pitch, heading):
        """计算旋转矩阵"""
        r = math.radians(roll)
        p = math.radians(pitch)
        h = math.radians(heading)

        # Z轴旋转 (航向)
        cz, sz = math.cos(h), math.sin(h)
        # Y轴旋转 (俯仰)
        cy, sy = math.cos(p), math.sin(p)
        # X轴旋转 (横滚)
        cx, sx = math.cos(r), math.sin(r)

        return np.array([
            [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
            [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
            [-sy, sx*cy, cx*cy]
        ])

    def _draw_attitude_arrows(self, ax, roll, pitch, heading):
        """在3D坐标轴上绘制姿态箭头"""
        ax.cla()
        ax.set_facecolor('#16213e')
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_zlim(-1.5, 1.5)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])

        R = self._rotation_matrix(roll, pitch, heading)

        # 基准轴
        origin = np.array([0, 0, 0])
        x_axis = R @ np.array([1, 0, 0])
        y_axis = R @ np.array([0, 1, 0])
        z_axis = R @ np.array([0, 0, 1])

        # 绘制箭头
        ax.quiver(*origin, *x_axis, color='red', arrow_length_ratio=0.15, linewidth=2)
        ax.quiver(*origin, *y_axis, color='green', arrow_length_ratio=0.15, linewidth=2)
        ax.quiver(*origin, *z_axis, color='blue', arrow_length_ratio=0.15, linewidth=2)

    def add_data(self, raw, kalman, debounce):
        """添加一组数据"""
        self.raw_buffer.append(raw)
        self.kalman_buffer.append(kalman)
        self.debounce_buffer.append(debounce)
        self.sample_count += 1

    def _format_axis(self, ax, title, raw_data, kal_data, deb_data, ylim=45):
        """格式化二维曲线图"""
        ax.cla()
        ax.set_facecolor('#16213e')
        ax.set_title(title, color='white', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-ylim, ylim)
        ax.tick_params(colors='white', labelsize=8)

        if len(raw_data) < 2:
            return

        x = np.arange(len(raw_data))

        ax.plot(x, raw_data, 'y-', alpha=0.4, linewidth=1, label='原始')
        ax.plot(x, kal_data, 'b-', linewidth=1.5, label='卡尔曼')
        ax.plot(x, deb_data, 'g-', linewidth=1.5, label='消抖')

        ax.legend(loc='upper right', fontsize=7, facecolor='#16213e', edgecolor='white')

    def update(self, frame):
        """动画更新函数"""
        if len(self.raw_buffer) < 2:
            return

        raw_roll = [d[0] for d in self.raw_buffer]
        raw_pitch = [d[1] for d in self.raw_buffer]
        raw_heading = [d[2] for d in self.raw_buffer]

        kal_roll = [d[0] for d in self.kalman_buffer]
        kal_pitch = [d[1] for d in self.kalman_buffer]
        kal_heading = [d[2] for d in self.kalman_buffer]

        deb_roll = [d[0] for d in self.debounce_buffer]
        deb_pitch = [d[1] for d in self.debounce_buffer]
        deb_heading = [d[2] for d in self.debounce_buffer]

        # 更新二维曲线
        self._format_axis(self.ax_roll, '横滚角 Roll', raw_roll, kal_roll, deb_roll, ylim=180)
        self._format_axis(self.ax_pitch, '俯仰角 Pitch', raw_pitch, kal_pitch, deb_pitch, ylim=180)
        self._format_axis(self.ax_heading, '航向角 Heading', raw_heading, kal_heading, deb_heading)
        self.ax_heading.set_ylim(0, 360)

        # 更新三维姿态
        if self.sample_count >= 1:
            raw = self.raw_buffer[-1]
            kal = self.kalman_buffer[-1]
            deb = self.debounce_buffer[-1]

            self._draw_attitude_arrows(self.ax_raw_3d, *raw)
            self._draw_attitude_arrows(self.ax_kal_3d, *kal)
            self._draw_attitude_arrows(self.ax_deb_3d, *deb)

        # 绘制标签
        for ax, title in [(self.ax_raw_3d, 'Raw Data'), (self.ax_kal_3d, 'Kalman Filter'), (self.ax_deb_3d, 'Debounce')]:
            ax.set_title(title, color='white', fontsize=10, pad=2)

        # 更新统计信息
        self.ax_stats.cla()
        self.ax_stats.set_facecolor('#16213e')

        if len(raw_roll) >= 10:
            self.ax_stats.text(0.5, 0.95, f'样本数: {self.sample_count}',
                              fontsize=11, color='white', ha='center', va='top',
                              fontweight='bold', transform=self.ax_stats.transAxes)

            self.ax_stats.text(0.05, 0.78, '横滚角 Roll:', fontsize=9, color='white', va='top',
                              fontweight='bold', transform=self.ax_stats.transAxes)
            self.ax_stats.text(0.05, 0.66, f'  原始: {np.std(raw_roll):.4f} deg',
                              fontsize=9, color='yellow', va='top', transform=self.ax_stats.transAxes)
            self.ax_stats.text(0.05, 0.55, f'  卡尔曼: {np.std(kal_roll):.4f} deg',
                              fontsize=9, color='cornflowerblue', va='top', transform=self.ax_stats.transAxes)
            self.ax_stats.text(0.05, 0.44, f'  消抖: {np.std(deb_roll):.4f} deg',
                              fontsize=9, color='limegreen', va='top', transform=self.ax_stats.transAxes)

            self.ax_stats.text(0.05, 0.30, '俯仰角 Pitch:', fontsize=9, color='white', va='top',
                              fontweight='bold', transform=self.ax_stats.transAxes)
            self.ax_stats.text(0.05, 0.18, f'  原始: {np.std(raw_pitch):.4f} deg',
                              fontsize=9, color='yellow', va='top', transform=self.ax_stats.transAxes)
            self.ax_stats.text(0.05, 0.07, f'  卡尔曼: {np.std(kal_pitch):.4f} deg',
                              fontsize=9, color='cornflowerblue', va='top', transform=self.ax_stats.transAxes)
            self.ax_stats.text(0.55, 0.30, '航向角 Heading:', fontsize=9, color='white', va='top',
                              fontweight='bold', transform=self.ax_stats.transAxes)
            self.ax_stats.text(0.55, 0.18, f'  原始: {np.std(raw_heading):.4f} deg',
                              fontsize=9, color='yellow', va='top', transform=self.ax_stats.transAxes)
            self.ax_stats.text(0.55, 0.07, f'  消抖: {np.std(kal_heading):.4f} deg',
                              fontsize=9, color='limegreen', va='top', transform=self.ax_stats.transAxes)
        else:
            self.ax_stats.text(0.5, 0.5, f'样本数: {self.sample_count}\n\n等待数据...',
                              fontsize=11, color='white', ha='center', va='center',
                              transform=self.ax_stats.transAxes)

        self.fig.tight_layout()

    def start(self, compass, kalman, debounce):
        """启动可视化"""
        print("启动实时可视化...")
        print("按 Ctrl+C 或关闭图形窗口退出")
        print("-" * 40)

        def data_generator():
            while True:
                raw_data = compass.read()
                if raw_data is None:
                    yield None
                    continue

                kal_data = kalman.update(raw_data)
                deb_roll, deb_pitch, deb_heading = debounce.update(
                    raw_data.roll, raw_data.pitch, raw_data.heading
                )

                raw = (raw_data.roll, raw_data.pitch, raw_data.heading)
                kal = (kal_data.roll, kal_data.pitch, kal_data.heading)
                deb = (deb_roll, deb_pitch, deb_heading)

                self.add_data(raw, kal, deb)
                yield True

        gen = data_generator()

        def animate_wrapper(frame):
            try:
                result = next(gen)
                if result is None:
                    return []
                self.update(frame)
                return []
            except StopIteration:
                return []

        self.ani = animation.FuncAnimation(
            self.fig, animate_wrapper,
            interval=50, blit=False, cache_frame_data=False
        )

        try:
            plt.show()
        except KeyboardInterrupt:
            plt.close('all')
        finally:
            compass.disconnect()


def run_visualization(port: str = '/dev/ttyS0', alpha: float = 0.2, max_points: int = 200):
    """
    运行三维可视化

    Args:
        port: 串口名
        alpha: 消抖系数
        max_points: 最大显示点数
    """
    from compass_kalman import CompassKalmanFilter

    print("=" * 50)
    print("DDM350B 3D Attitude Real-time Visualization")
    print("=" * 50)
    print(f"串口: {port}")
    print(f"消抖系数 alpha: {alpha}")
    print("-" * 50)

    compass = DDM350B(port, timeout=2.0)
    if not compass.connect():
        print(f"错误: 无法连接到 {port}")
        return

    print(f"已连接到 {port}")
    compass.set_mode(OutputMode.AUTO_50HZ)
    print("模式: 自动输出 50Hz")
    print("-" * 50)

    kalman = CompassKalmanFilter(
        process_noise=0.001,
        measurement_noise=0.1,
        wrap_angles=[False, False, True]
    )
    debounce = DebounceFilter(alpha=alpha)

    viz = AttitudeVisualizer(max_points=max_points)
    viz.start(compass, kalman, debounce)

    print(f"\n共处理 {viz.sample_count} 个样本")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='DDM350B 三维姿态可视化')
    parser.add_argument('-p', '--port', default='/dev/ttyS0', help='串口名 (默认: /dev/ttyS0)')
    parser.add_argument('-a', '--alpha', type=float, default=0.2,
                        help='消抖系数 (默认: 0.2)')
    parser.add_argument('-n', '--points', type=int, default=200,
                        help='最大显示点数 (默认: 200)')

    args = parser.parse_args()
    run_visualization(args.port, args.alpha, args.points)
