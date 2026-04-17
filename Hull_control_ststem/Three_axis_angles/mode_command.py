def build_set_mode_command(mode: int) -> bytes:
    """
    构建设置输出模式的命令
    
    命令格式: [帧头] [长度] [命令字高] [命令字低] [数据域] [校验和]
             68     05    00        0C         mode    checksum
    
    校验和 = (长度 + 命令字高 + 命令字低 + 数据域) & 0xFF
    
    Args:
        mode: 输出模式
            00: 问答式(出厂默认)
            01: 自动输出式5Hz
            02: 自动输出式15Hz
            03: 自动输出式25Hz
            04: 自动输出式35Hz
            05: 自动输出式50Hz
            06: 自动输出式100Hz
    
    Returns:
        完整的6字节命令
    """
    if mode not in [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06]:
        raise ValueError("模式值必须在 0x00-0x06 之间")
    
    header = 0x68        # 帧头 (不参与校验和计算)
    length = 0x05        # 长度
    cmd_high = 0x00      # 命令字高字节
    cmd_low = 0x0C       # 命令字低字节
    data_byte = mode     # 数据域 (模式)
    
    # 校验和 = (长度 + 命令字高 + 命令字低 + 数据域) & 0xFF
    checksum = (length + cmd_high + cmd_low + data_byte) & 0xFF
    
    return bytes([header, length, cmd_high, cmd_low, data_byte, checksum])


def get_mode_name(mode: int) -> str:
    """获取模式名称"""
    mode_names = {
        0x00: "问答式(默认)",
        0x01: "自动输出 5Hz",
        0x02: "自动输出 15Hz",
        0x03: "自动输出 25Hz",
        0x04: "自动输出 35Hz",
        0x05: "自动输出 50Hz",
        0x06: "自动输出 100Hz"
    }
    return mode_names.get(mode, f"未知({mode:02X})")


def verify_checksum(cmd: bytes) -> bool:
    """验证校验和是否正确"""
    if len(cmd) != 6:
        return False
    # 校验和 = (长度 + 命令字高 + 命令字低 + 数据域) & 0xFF
    expected = (cmd[1] + cmd[2] + cmd[3] + cmd[4]) & 0xFF
    return cmd[5] == expected


# 测试函数
if __name__ == "__main__":
    print("=" * 60)
    print("模式命令生成测试 (修正校验和计算)")
    print("=" * 60)
    
    test_modes = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06]
    expected_checksums = {
        0x00: 0x11,
        0x01: 0x12,
        0x02: 0x13,
        0x03: 0x14,
        0x04: 0x15,
        0x05: 0x16,
        0x06: 0x17
    }
    
    all_ok = True
    for mode in test_modes:
        cmd = build_set_mode_command(mode)
        hex_str = ' '.join(f'{b:02X}' for b in cmd)
        checksum = cmd[5]
        expected = expected_checksums[mode]
        ok = "✓" if checksum == expected else "✗"
        if checksum != expected:
            all_ok = False
        print(f"模式 {mode:02X} ({get_mode_name(mode):15s}): {hex_str}  校验和={checksum:02X} (预期={expected:02X}) {ok}")
    
    print()
    if all_ok:
        print("✓ 所有校验和计算正确！")
    else:
        print("✗ 存在校验和错误")
    
    print("\n公式验证:")
    print("  校验和 = (长度 0x05 + 命令字高 0x00 + 命令字低 0x0C + 模式) & 0xFF")
    print("  即: (5 + 0 + 12 + mode) & 0xFF = (17 + mode) & 0xFF")
    print("  例如: mode=0x00 -> 17+0=17=0x11")
    print("        mode=0x02 -> 17+2=19=0x13")
    print("        mode=0x05 -> 17+5=22=0x16")
