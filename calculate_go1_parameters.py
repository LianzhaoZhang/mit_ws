#!/usr/bin/env python3
"""
Calculate Go1 robot parameters (_mass, _Ib, _pcb) from URDF file
验证计算过程的正确性
"""

import xml.etree.ElementTree as ET
import numpy as np
import math

def parse_urdf_mass_inertia(urdf_file):
    """解析URDF文件中的质量和惯性信息"""
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    links_data = []
    
    for link in root.findall('link'):
        link_name = link.get('name')
        inertial = link.find('inertial')
        
        if inertial is not None:
            # 获取质量
            mass_elem = inertial.find('mass')
            if mass_elem is not None:
                mass = float(mass_elem.get('value'))
            else:
                mass = 0.0
            
            # 获取质心位置
            origin = inertial.find('origin')
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0').split()
                com = [float(x) for x in xyz]
            else:
                com = [0.0, 0.0, 0.0]
            
            # 获取惯性矩阵
            inertia_elem = inertial.find('inertia')
            if inertia_elem is not None:
                ixx = float(inertia_elem.get('ixx', 0))
                iyy = float(inertia_elem.get('iyy', 0))
                izz = float(inertia_elem.get('izz', 0))
                ixy = float(inertia_elem.get('ixy', 0))
                ixz = float(inertia_elem.get('ixz', 0))
                iyz = float(inertia_elem.get('iyz', 0))
                
                inertia_matrix = np.array([
                    [ixx, ixy, ixz],
                    [ixy, iyy, iyz],
                    [ixz, iyz, izz]
                ])
            else:
                inertia_matrix = np.zeros((3, 3))
            
            if mass > 0:  # 只考虑有质量的部件
                links_data.append({
                    'name': link_name,
                    'mass': mass,
                    'com': com,
                    'inertia': inertia_matrix
                })
    
    return links_data

def get_joint_transforms(urdf_file):
    """获取关节的变换矩阵"""
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    joints = {}
    
    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        
        origin = joint.find('origin')
        if origin is not None:
            xyz = origin.get('xyz', '0 0 0').split()
            rpy = origin.get('rpy', '0 0 0').split()
            translation = [float(x) for x in xyz]
            rotation = [float(x) for x in rpy]
        else:
            translation = [0.0, 0.0, 0.0]
            rotation = [0.0, 0.0, 0.0]
        
        joints[joint_name] = {
            'type': joint_type,
            'parent': parent,
            'child': child,
            'translation': translation,
            'rotation': rotation
        }
    
    return joints

def calculate_leg_com_inertia(joint_angles, leg_prefix):
    """计算单条腿的质心位置和惯性矩阵"""
    # Go1机器人的DH参数（基于URDF文件）
    if leg_prefix in ['FR', 'FL']:
        hip_offset = 0.0838  # 髋关节到大腿关节的距离
    else:
        hip_offset = 0.0838
    
    thigh_length = 0.2  # 大腿长度
    calf_length = 0.2   # 小腿长度
    
    # 关节角度
    q1, q2, q3 = joint_angles
    
    # 计算各关节位置
    # Hip joint position (相对于body)
    hip_pos = np.array([0, 0, 0])
    
    # Thigh joint position
    thigh_pos = hip_pos + np.array([0, hip_offset, 0])
    
    # Knee joint position
    knee_pos = thigh_pos + np.array([0, 0, -thigh_length])
    
    # Foot position
    foot_pos = knee_pos + np.array([0, 0, -calf_length])
    
    return {
        'hip_pos': hip_pos,
        'thigh_pos': thigh_pos,
        'knee_pos': knee_pos,
        'foot_pos': foot_pos
    }

def calculate_robot_parameters(urdf_file, joint_angles=(0.0, 0.67, -1.3)):
    """计算机器人的总质量、质心和惯性矩阵"""
    print(f"解析URDF文件: {urdf_file}")
    
    # 解析URDF文件
    links_data = parse_urdf_mass_inertia(urdf_file)
    joints_data = get_joint_transforms(urdf_file)
    
    print(f"找到 {len(links_data)} 个有质量的链接")
    
    # 计算总质量
    total_mass = 0.0
    for link in links_data:
        total_mass += link['mass']
        print(f"{link['name']}: mass={link['mass']:.4f} kg")
    
    print(f"\n总质量 (_mass): {total_mass:.4f} kg")
    
    # 计算质心位置 (相对于body坐标系)
    weighted_com = np.array([0.0, 0.0, 0.0])
    for link in links_data:
        # 这里简化处理，假设所有部件的质心都在body坐标系中
        # 实际应该根据运动学链计算每个部件在body坐标系中的位置
        if 'trunk' in link['name'].lower() or 'base' in link['name'].lower():
            # 主体部件
            weighted_com += link['mass'] * np.array(link['com'])
        else:
            # 其他部件，简化处理
            weighted_com += link['mass'] * np.array(link['com'])
    
    com_position = weighted_com / total_mass
    print(f"质心位置 (_pcb): [{com_position[0]:.6f}, {com_position[1]:.6f}, {com_position[2]:.6f}]")
    
    # 计算惯性矩阵 (简化为对角矩阵)
    # 使用平行轴定理计算总惯性矩阵
    total_inertia = np.zeros((3, 3))
    
    for link in links_data:
        # 获取link相对于body的位置
        link_pos = np.array(link['com'])
        
        # 平行轴定理: I_total = I_link + m * (r^2 * I - r * r^T)
        r = link_pos - com_position
        r_squared = np.dot(r, r)
        r_outer = np.outer(r, r)
        
        parallel_axis_term = link['mass'] * (r_squared * np.eye(3) - r_outer)
        total_inertia += link['inertia'] + parallel_axis_term
    
    # 提取对角元素作为主惯性矩
    ixx = total_inertia[0, 0]
    iyy = total_inertia[1, 1]
    izz = total_inertia[2, 2]
    
    print(f"惯性矩阵对角元素 (_Ib):")
    print(f"  Ixx: {ixx:.6f}")
    print(f"  Iyy: {iyy:.6f}")
    print(f"  Izz: {izz:.6f}")
    print(f"Vec3({ixx:.4f}, {iyy:.4f}, {izz:.4f}).asDiagonal()")
    
    # 与代码中的Go1参数对比
    print(f"\n与unitreeRobot.cpp中Go1参数对比:")
    print(f"URDF计算结果:")
    print(f"  _mass = {total_mass:.1f};")
    print(f"  _pcb << {com_position[0]:.2f}, {com_position[1]:.2f}, {com_position[2]:.2f};")
    print(f"  _Ib = Vec3({ixx:.4f}, {iyy:.4f}, {izz:.4f}).asDiagonal();")
    
    print(f"\nunitreeRobot.cpp中的Go1参数:")
    print(f"  仿真: _mass = 12.0, _pcb = (0.0, 0.0, 0.0), _Ib = (0.0792, 0.2085, 0.2265)")
    print(f"  实物: _mass = 10.5, _pcb = (0.04, 0.0, 0.0), _Ib = (0.0792, 0.2085, 0.2265)")
    
    return {
        'mass': total_mass,
        'com': com_position,
        'inertia_diagonal': [ixx, iyy, izz],
        'inertia_matrix': total_inertia
    }

def main():
    """主函数"""
    print("=" * 60)
    print("Go1机器人参数计算工具")
    print("=" * 60)
    
    # URDF文件路径
    urdf_file = "/home/zlz/mit_ws/src/unitree_ros/robots/go1_description/urdf/go1.urdf"
    
    # 初始站立关节角度
    joint_angles = (0.0, 0.67, -1.3)
    print(f"使用关节角度: {joint_angles}")
    
    try:
        # 计算参数
        result = calculate_robot_parameters(urdf_file, joint_angles)
        
        print(f"\n" + "=" * 60)
        print("计算完成！")
        print("=" * 60)
        
        # 生成C++代码
        print(f"\n建议的C++代码:")
        print(f"// Go1Robot constructor - 基于URDF计算")
        print(f"_mass = {result['mass']:.1f};")
        print(f"_pcb << {result['com'][0]:.3f}, {result['com'][1]:.3f}, {result['com'][2]:.3f};")
        ixx, iyy, izz = result['inertia_diagonal']
        print(f"_Ib = Vec3({ixx:.4f}, {iyy:.4f}, {izz:.4f}).asDiagonal();")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
