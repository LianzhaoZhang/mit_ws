#!/usr/bin/env python3
"""
Calculate robot parameters (_mass, _Ib, _pcb) from URDF data
Based on the B1 robot URDF file analysis
"""

import numpy as np

def calculate_robot_parameters():
    """
    Calculate the three parameters needed for unitreeRobot.cpp:
    - _mass: Total mass of all links
    - _Ib: Body inertia matrix (3x3 diagonal matrix)
    - _pcb: Position of center of mass relative to body frame
    """
    
    # Extract mass and inertia data from URDF
    # Main body (trunk) - this is the most important component
    trunk_mass = 29.45
    trunk_com = np.array([0.008987, 0.002243, 0.003013])  # CoM relative to trunk frame
    trunk_inertia = np.array([
        [0.183142146, -0.001379002, -0.027956055],
        [-0.001379002, 0.756327752, 0.000193774],
        [-0.027956055, 0.000193774, 0.783777558]
    ])
    
    # Leg components (4 legs, each with hip, thigh, calf)
    # Each leg has similar structure but different orientations
    leg_components = {
        'hip': {'mass': 2.15, 'count': 4},
        'thigh': {'mass': 4.3, 'count': 4},
        'calf': {'mass': 1.05, 'count': 4},
        'foot': {'mass': 0.05, 'count': 4},
        'hip_rotor': {'mass': 0.199, 'count': 4},
        'thigh_rotor': {'mass': 0.266, 'count': 4},
        'calf_rotor': {'mass': 0.266, 'count': 4}
    }
    
    # Sensors and accessories (negligible mass)
    accessory_mass = 5 * 0.001  # 5 camera/sensor links with 0.001 kg each
    
    # Calculate total mass
    leg_total_mass = sum([comp['mass'] * comp['count'] for comp in leg_components.values()])
    total_mass = trunk_mass + leg_total_mass + accessory_mass
    
    print("=== MASS CALCULATION ===")
    print(f"Trunk mass: {trunk_mass} kg")
    print(f"Leg components total mass: {leg_total_mass} kg")
    print(f"Accessory mass: {accessory_mass} kg")
    print(f"Total mass (_mass): {total_mass:.2f} kg")
    
    # For simplified single rigid body model, we use the trunk inertia as the base
    # Since the legs are relatively small compared to the main body and are close to the body center
    # we can approximate the body inertia using mainly the trunk inertia
    
    # Extract diagonal components (principal moments of inertia)
    Ixx = trunk_inertia[0, 0]  # Roll inertia
    Iyy = trunk_inertia[1, 1]  # Pitch inertia  
    Izz = trunk_inertia[2, 2]  # Yaw inertia
    
    print(f"\n=== INERTIA CALCULATION ===")
    print(f"Trunk inertia matrix:")
    print(trunk_inertia)
    print(f"\nPrincipal moments of inertia:")
    print(f"Ixx (roll): {Ixx:.6f}")
    print(f"Iyy (pitch): {Iyy:.6f}")
    print(f"Izz (yaw): {Izz:.6f}")
    
    # For single rigid body approximation, we mainly use trunk inertia
    # Adding some estimation for leg contribution (legs extend the body dimensions)
    # The legs will increase the inertia, especially in pitch and roll directions
    
    # Rough estimation: legs contribute about 20-30% additional inertia
    leg_inertia_factor = 1.25
    
    body_inertia_diag = np.array([
        Ixx * leg_inertia_factor,
        Iyy * leg_inertia_factor, 
        Izz * leg_inertia_factor
    ])
    
    print(f"\nEstimated body inertia diagonal (_Ib):")
    print(f"Vec3({body_inertia_diag[0]:.6f}, {body_inertia_diag[1]:.6f}, {body_inertia_diag[2]:.6f})")
    
    # Center of mass position (_pcb)
    # For a quadruped, the CoM is typically close to the geometric center of the trunk
    # The trunk CoM offset is very small, so we can use it directly or set to zero
    pcb = trunk_com  # Use trunk CoM offset
    
    print(f"\n=== CENTER OF MASS CALCULATION ===")
    print(f"Trunk CoM offset: {trunk_com}")
    print(f"Body CoM position (_pcb): Vec3({pcb[0]:.6f}, {pcb[1]:.6f}, {pcb[2]:.6f})")
    
    # Generate the code snippet for unitreeRobot.cpp
    print(f"\n=== CODE FOR UNITREEROBOT.CPP ===")
    print(f"// For B1/Aliengo Robot:")
    print(f"_mass = {total_mass:.1f};")
    print(f"_pcb << {pcb[0]:.6f}, {pcb[1]:.6f}, {pcb[2]:.6f};")
    print(f"_Ib = Vec3({body_inertia_diag[0]:.6f}, {body_inertia_diag[1]:.6f}, {body_inertia_diag[2]:.6f}).asDiagonal();")
    
    return total_mass, body_inertia_diag, pcb

if __name__ == "__main__":
    calculate_robot_parameters()
