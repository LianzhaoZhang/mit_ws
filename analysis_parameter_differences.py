#!/usr/bin/env python3
"""
分析unitreeRobot.cpp中参数与URDF计算结果差异的原因
并提供更准确的参数计算方法
"""

def analyze_parameter_differences():
    """
    分析参数差异的原因
    """
    print("=" * 80)
    print("unitreeRobot.cpp参数与URDF计算结果差异分析")
    print("=" * 80)
    
    print("\n1. 质量差异分析:")
    print("   URDF计算: 13.1 kg")
    print("   代码中仿真: 12.0 kg") 
    print("   代码中实物: 10.5 kg")
    print("   -> 差异原因: URDF包含了所有详细组件(转子、传感器等)，")
    print("      而控制算法中使用的是简化的单刚体模型质量")
    
    print("\n2. 质心位置差异分析:")
    print("   URDF计算: (0.008, 0.001, -0.014)")
    print("   代码中仿真: (0.0, 0.0, 0.0)")
    print("   代码中实物: (0.04, 0.0, 0.0)")
    print("   -> 差异原因: 控制算法中常常假设质心在几何中心，")
    print("      实物由于电池、线缆等位置，质心会有偏移")
    
    print("\n3. 惯性矩阵差异分析:")
    print("   URDF计算: (0.0515, 0.0997, 0.0804)")
    print("   代码中参数: (0.0792, 0.2085, 0.2265)")
    print("   -> 差异原因: ")
    print("      a) URDF是基于CAD模型的理论计算")
    print("      b) 代码中的参数可能是实验标定或经验调整的结果")
    print("      c) 单刚体模型需要考虑动态特性，不只是静态惯性")
    
    print("\n4. 为什么会有这些差异？")
    print("   - URDF模型: 用于仿真，包含所有细节，追求几何精确性")
    print("   - 控制参数: 用于实际控制，经过实验优化，追求控制效果")
    print("   - 单刚体假设: 将复杂的多刚体系统简化为单个刚体")
    print("   - 参数调优: 实际参数可能经过大量实验和调试")

def calculate_adjusted_parameters():
    """
    基于分析结果，提供调整后的参数建议
    """
    print("\n" + "=" * 80)
    print("基于分析的参数调整建议")
    print("=" * 80)
    
    # 原始URDF计算结果
    urdf_mass = 13.1
    urdf_pcb = [0.008, 0.001, -0.014]
    urdf_inertia = [0.0515, 0.0997, 0.0804]
    
    # 代码中的实际参数
    code_mass_sim = 12.0
    code_mass_real = 10.5
    code_pcb_sim = [0.0, 0.0, 0.0]
    code_pcb_real = [0.04, 0.0, 0.0]
    code_inertia = [0.0792, 0.2085, 0.2265]
    
    print(f"\n建议的B1/Aliengo参数计算策略:")
    print(f"1. 质量调整:")
    print(f"   - URDF计算作为上限参考")
    print(f"   - 根据Go1的比例关系进行缩放")
    print(f"   - Go1缩放因子: 仿真 {code_mass_sim/urdf_mass:.3f}, 实物 {code_mass_real/urdf_mass:.3f}")
    
    # 假设B1的URDF计算质量为50kg(示例)
    b1_urdf_mass = 50.0  # 这个需要实际计算B1的URDF
    b1_adjusted_mass_sim = b1_urdf_mass * (code_mass_sim/urdf_mass)
    b1_adjusted_mass_real = b1_urdf_mass * (code_mass_real/urdf_mass)
    
    print(f"   - 如果B1 URDF计算质量为 {b1_urdf_mass} kg")
    print(f"   - 建议仿真质量: {b1_adjusted_mass_sim:.1f} kg")
    print(f"   - 建议实物质量: {b1_adjusted_mass_real:.1f} kg")
    
    print(f"\n2. 惯性矩阵调整:")
    print(f"   - URDF计算提供基础值")
    print(f"   - 根据机器人尺寸进行缩放")
    print(f"   - 考虑控制性能进行微调")
    
    # 惯性缩放因子
    inertia_scale_factors = [code_inertia[i]/urdf_inertia[i] for i in range(3)]
    print(f"   - Go1惯性缩放因子: {inertia_scale_factors}")
    
    print(f"\n3. 质心位置调整:")
    print(f"   - 仿真可以使用几何中心 (0,0,0)")
    print(f"   - 实物需要考虑实际装配偏差")
    print(f"   - 建议先用 (0,0,0)，根据实验结果调整")

def generate_b1_parameters():
    """
    为B1生成建议参数
    """
    print("\n" + "=" * 80)
    print("B1/Aliengo建议参数 (基于Go1比例关系)")
    print("=" * 80)
    
    # 从你的B1 URDF分析得出的假设值(需要实际运行B1计算脚本)
    # 这里使用估算值作为示例
    
    print("基于以下假设:")
    print("- B1比Go1更大更重")
    print("- 保持相似的质量分布比例")
    print("- 使用Go1的参数调整经验")
    
    # Go1参数比例
    go1_urdf_mass = 13.1
    go1_code_mass_sim = 12.0
    go1_code_mass_real = 10.5
    go1_mass_factor_sim = go1_code_mass_sim / go1_urdf_mass
    go1_mass_factor_real = go1_code_mass_real / go1_urdf_mass
    
    # 假设B1 URDF质量约为60kg (需要实际计算)
    b1_estimated_urdf_mass = 60.0
    
    b1_suggested_mass_sim = b1_estimated_urdf_mass * go1_mass_factor_sim
    b1_suggested_mass_real = b1_estimated_urdf_mass * go1_mass_factor_real
    
    print(f"\n建议的B1参数:")
    print(f"// B1Robot constructor")
    print(f"#ifdef COMPILE_WITH_REAL_ROBOT")
    print(f"    _mass = {b1_suggested_mass_real:.1f};")
    print(f"    _pcb << 0.0, 0.0, 0.0;  // 建议先使用几何中心")
    print(f"    _Ib = Vec3(0.3, 0.8, 0.9).asDiagonal();  // 基于尺寸估算，需实验调优")
    print(f"#endif  // COMPILE_WITH_REAL_ROBOT")
    print(f"")
    print(f"#ifdef COMPILE_WITH_SIMULATION") 
    print(f"    _mass = {b1_suggested_mass_sim:.1f};")
    print(f"    _pcb << 0.0, 0.0, 0.0;")
    print(f"    _Ib = Vec3(0.3, 0.8, 0.9).asDiagonal();  // 需要实际计算和调优")
    print(f"#endif  // COMPILE_WITH_SIMULATION")
    
    print(f"\n注意事项:")
    print(f"1. 这些是基于比例估算的初始值")
    print(f"2. 实际使用时需要:")
    print(f"   - 运行B1的URDF分析脚本得到准确的基础值")
    print(f"   - 根据控制效果进行参数调优")
    print(f"   - 考虑实际硬件配置差异")

if __name__ == "__main__":
    analyze_parameter_differences()
    calculate_adjusted_parameters()
    generate_b1_parameters()
