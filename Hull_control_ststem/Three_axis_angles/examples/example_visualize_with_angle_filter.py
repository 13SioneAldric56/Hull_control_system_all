#!/usr/bin/env python3
"""
DDM350B/DDM360B 三维姿态实时可视化 - 使用 angle_filter 模块

三种数据并列对比：原始数据、卡尔曼滤波、角度滤波（angle_filter）

使用方法:
    python example_visualize_with_angle_filter.py -p COM3

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
import argparse

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

from ddm350b import DDM350B, OutputMode, CompassData
from compass_kalman import CompassKalmanFilter
from angle_filter import (
    TripleAxisFilter,
    filter_angles_ex,
    FilteredAngleData,
    reset_filters
)


class AttitudeVisualizerWithAngleFilter:
    """三维姿态可视化器 - 集成 angle_filter"""

    def __init__(self, max_points: int = 200):
        self.max_points = max_points

        # 数据缓冲区
        self.raw_buffer = deque(maxlen=max_points)       # 原始数据 (roll, pitch, heading_0_360)
        self.kalman_buffer = deque(maxlen=max_points)    # 卡尔曼滤波后 (roll, pitch, heading_0_360)
        self.angle_filter_buffer = deque(maxlen=max_points)  # angle_filter 后 (roll, pitch, heading_0_360)

        # 创建图形
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(16, 10), facecolor='#1a1a2e')
        self.fig.suptitle('DDM350B 3D Attitude - Angle Filter Visualization', fontsize=14, color='white')

        gs = GridSpec(2, 4, figure=self.fig, hspace=0.35, wspace=0.3)

        # 第一行：三个二维曲线图
        self.ax_roll = self.fig.add_subplot(gs[0, 0])
        self.ax_pitch = self.fig.add_subplot(gs[0, 1])
        self.ax_heading = self.fig.add_subplot(gs[0, 2])
        self.ax_stats = self.fig.add_subplot(gs[0, 3])

        # 第二行：三个三维姿态图
        self.ax_raw_3d = self.fig.add_subplot(gs[1, 0], projection='3d')
        self.ax_kal_3d = self.fig.add_subplot(gs[1, 1], projection='3d')
        self.ax_af_3d = self.fig.add_subplot(gs[1, 2], projection='3d')
        self.ax_legend = self.fig.add_subplot(gs[1, 3])

        self._setup_axes()

        self.ani = None
        self.sample_count = 0

    def _setup_axes(self):
        """设置坐标轴样式"""
        for ax in [self.ax_roll, self.ax_pitch, self.ax_heading]:
            ax.set_facecolor('#16213e')
            ax.grid(True, alpha=0.3)
            ax.tick_params(colors='white')

        for ax in [self.ax_raw_3d, self.ax_kal_3d, self.ax_af_3d]:
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

        # 图例说明
        legend_props = dict(boxstyle='round,pad=0.5', facecolor='#16213e', edgecolor='white', alpha=0.9)

        self.ax_legend.text(0.5, 0.95, '数据类型说明', fontsize=12, fontweight='bold',
                          color='yellow', ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.82, '黄色 - 原始数据', fontsize=10, color='yellow',
                          ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.70, '蓝色 - 卡尔曼滤波', fontsize=10, color='cornflowerblue',
                          ha='center', va='top', transform=self.ax_legend.transAxes)
        self.ax_legend.text(0.5, 0.58, '绿色 - AngleFilter', fontsize=10, color='limegreen',
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

        cz, sz = math.cos(h), math.sin(h)
        cy, sy = math.cos(p), math.sin(p)
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

        origin = np.array([0, 0, 0])
        x_axis = R @ np.array([1, 0, 0])
        y_axis = R @ np.array([0, 1, 0])
        z_axis = R @ np.array([0, 0, 1])

        ax.quiver(*origin, *x_axis, color='red', arrow_length_ratio=0.15, linewidth=2)
        ax.quiver(*origin, *y_axis, color='green', arrow_length_ratio=0.15, linewidth=2)
        ax.quiver(*origin, *z_axis, color='blue', arrow_length_ratio=0.15, linewidth=2)

    def add_data(self, raw, kalman, angle_filtered):
        """添加一组数据"""
        self.raw_buffer.append(raw)
        self.kalman_buffer.append(kalman)
        self.angle_filter_buffer.append(angle_filtered)
        self.sample_count += 1

    def _format_axis(self, ax, title, raw_data, kal_data, af_data, ylim=45):
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
        ax.plot(x, af_data, 'g-', linewidth=1.5, label='AngleFilter')

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

        af_roll = [d[0] for d in self.angle_filter_buffer]
        af_pitch = [d[1] for d in self.angle_filter_buffer]
        af_heading = [d[2] for d in self.angle_filter_buffer]

        # 更新二维曲线
        self._format_axis(self.ax_roll, '横滚角 Roll', raw_roll, kal_roll, af_roll, ylim=180)
        self._format_axis(self.ax_pitch, '俯仰角 Pitch', raw_pitch, kal_pitch, af_pitch, ylim=180)
        self._format_axis(self.ax_heading, '航向角 Heading', raw_heading, kal_heading, af_heading)
        self.ax_heading.set_ylim(0, 360)

        # 更新三维姿态
        if self.sample_count >= 1:
            raw = self.raw_buffer[-1]
            kal = self.kalman_buffer[-1]
            af = self.angle_filter_buffer[-1]

            self._draw_attitude_arrows(self.ax_raw_3d, *raw)
            self._draw_attitude_arrows(self.ax_kal_3d, *kal)
            self._draw_attitude_arrows(self.ax_af_3d, *af)

        # 绘制标签
        for ax, title in [
            (self.ax_raw_3d, 'Raw Data'),
            (self.ax_kal_3d, 'Kalman Filter'),
            (self.ax_af_3d, 'AngleFilter')
        ]:
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
            self.ax_stats.text(0.05, 0.44, f'  AngleFilter: {np.std(af_roll):.4f} deg',
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
            self.ax_stats.text(0.55, 0.07, f'  AngleFilter: {np.std(af_heading):.4f} deg',
                              fontsize=9, color='limegreen', va='top', transform=self.ax_stats.transAxes)
        else:
            self.ax_stats.text(0.5, 0.5, f'样本数: {self.sample_count}\n\n等待数据...',
                              fontsize=11, color='white', ha='center', va='center',
                              transform=self.ax_stats.transAxes)

        self.fig.tight_layout()

    def start(self, compass, kalman, angle_filter):
        """启动可视化"""
        print("启动实时可视化...")
        print("按 Ctrl+C 或关闭图形窗口退出")
        print("-" * 40)

        def data_generator():
            while True:
                raw_data = compass.read()
                if raw_data is None:
                    print("[警告] 读取数据失败，重试中...")
                    yield None
                    continue

                # 卡尔曼滤波
                kal_data = kalman.update(raw_data)

                # AngleFilter 滤波
                af_result = angle_filter.filter(
                    raw_data.roll,
                    raw_data.pitch,
                    raw_data.heading
                )
                # filter() 返回 (roll, pitch, yaw_signed)
                # yaw_signed 是 -180°~+180°，需要转回 0°~360° 显示
                af_roll, af_pitch, af_yaw_signed = af_result
                af_yaw_360 = af_yaw_signed % 360.0
                if af_yaw_360 < 0:
                    af_yaw_360 += 360.0

                raw = (raw_data.roll, raw_data.pitch, raw_data.heading)
                kal = (kal_data.roll, kal_data.pitch, kal_data.heading)
                af = (af_roll, af_pitch, af_yaw_360)

                self.add_data(raw, kal, af)
                # 调试输出（可选，取消注释以查看实时数据）
                # print(f"Raw: H={raw_data.heading:.1f}° | "
                #       f"Kalman: H={kal_data.heading:.1f}° | "
                #       f"AF: H={af_yaw_360:.1f}°")
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
            reset_filters()  # 重置滤波器状态


def run_visualization(port: str = 'COM3', yaw_coeff: float = 0.15,
                     pitch_coeff: float = 0.15, roll_coeff: float = 0.15,
                     max_points: int = 200):
    """
    运行三维可视化

    Args:
        port: 串口名
        yaw_coeff: 航向角滤波系数 (0.0~1.0)
        pitch_coeff: 俯仰角滤波系数 (0.0~1.0)
        roll_coeff: 横滚角滤波系数 (0.0~1.0)
        max_points: 最大显示点数
    """
    from compass_kalman import CompassKalmanFilter

    print("=" * 60)
    print("DDM350B 3D Attitude - AngleFilter Visualization")
    print("=" * 60)
    print(f"串口: {port}")
    print(f"滤波系数 - Yaw: {yaw_coeff}, Pitch: {pitch_coeff}, Roll: {roll_coeff}")
    print("-" * 60)

    # 连接罗盘
    compass = DDM350B(port, timeout=2.0)
    if not compass.connect():
        print(f"错误: 无法连接到 {port}")
        return

    print(f"已连接到 {port}")
    compass.set_mode(OutputMode.AUTO_50HZ)
    print("模式: 自动输出 50Hz")
    print("-" * 60)

    # 初始化滤波器
    kalman = CompassKalmanFilter(
        process_noise=0.001,
        measurement_noise=0.1,
        wrap_angles=[False, False, True]  # 仅航向角需要环形处理
    )

    angle_filter = TripleAxisFilter(
        yaw_coeff=yaw_coeff,
        pitch_coeff=pitch_coeff,
        roll_coeff=roll_coeff
    )

    # 启动可视化
    viz = AttitudeVisualizerWithAngleFilter(max_points=max_points)
    viz.start(compass, kalman, angle_filter)

    print(f"\n共处理 {viz.sample_count} 个样本")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='DDM350B 三维姿态可视化 - 使用 AngleFilter',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
    # 使用默认参数
    python example_visualize_with_angle_filter.py -p COM3

    # 调整滤波系数（更平滑）
    python example_visualize_with_angle_filter.py -p COM3 --yaw-coeff 0.1 --pitch-coeff 0.1 --roll-coeff 0.1

    # 更灵敏的响应
    python example_visualize_with_angle_filter.py -p COM3 --yaw-coeff 0.3
        """
    )
    parser.add_argument('-p', '--port', default='COM3', help='串口名 (默认: COM3)')
    parser.add_argument('--yaw-coeff', type=float, default=0.15,
                       help='航向角滤波系数 (默认: 0.15)')
    parser.add_argument('--pitch-coeff', type=float, default=0.15,
                       help='俯仰角滤波系数 (默认: 0.15)')
    parser.add_argument('--roll-coeff', type=float, default=0.15,
                       help='横滚角滤波系数 (默认: 0.15)')
    parser.add_argument('-n', '--points', type=int, default=200,
                       help='最大显示点数 (默认: 200)')

    args = parser.parse_args()
    run_visualization(args.port, args.yaw_coeff, args.pitch_coeff, args.roll_coeff, args.points)
