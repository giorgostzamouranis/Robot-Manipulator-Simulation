# simulation_b_modified_coordinates.py

import numpy as np
import matplotlib.pyplot as plt
from sympy import symbols, solve

# Ορισμός Παραμέτρων Ρομποτικού Βραχίονα
l0 = 81.0  
l1 = 20.0  
l2 = 60.0  
l4 = 14.0  
l5 = 55.0  
l7 = 10.0  

# Ορισμός Αρχικών και Τελικών Σημείων
start_x, start_y, start_z = 10, 30, 100
end_x, end_y, end_z = 70, 80, 100  

# Ορισμός Παραμέτρων Χρόνου και Ταχύτητας
total_time = 300.0    
half_time = total_time / 2.0  
phase_duration = half_time * 0.25       
max_velocity_time = half_time - 2 * phase_duration 
max_vel_x = (end_x - start_x) / (half_time - phase_duration)  
max_vel_y = (end_y - start_y) / (half_time - phase_duration)  

# Δημιουργία Διαστήματος Χρόνου για Κάθε Φάση Κίνησης
num_samples = 7000  
acceleration_phase  = np.linspace(0, phase_duration, num_samples, endpoint=True)
constant_phase = np.linspace(phase_duration, phase_duration + max_velocity_time, num_samples, endpoint=True)
deceleration_phase  = np.linspace(phase_duration + max_velocity_time, half_time, num_samples, endpoint=True)

# Συνάρτηση για Υπολογισμό Πολυωνυμικών Συντελεστών
def get_polynomial_coefficients(start_pos, end_pos, start_vel, end_vel, start_t, end_t):
    a4, a3, a2, a1, a0 = symbols('a4 a3 a2 a1 a0', real=True)

    pos_start = a4*start_t**4 + a3*start_t**3 + a2*start_t**2 + a1*start_t + a0
    pos_end = a4*end_t**4 + a3*end_t**3 + a2*end_t**2 + a1*end_t + a0

    vel_start = 4*a4*start_t**3 + 3*a3*start_t**2 + 2*a2*start_t + a1
    vel_end = 4*a4*end_t**3 + 3*a3*end_t**2 + 2*a2*end_t + a1

    acc_start = 12*a4*start_t**2 + 6*a3*start_t + 2*a2  

    equations = [
        pos_start - start_pos,
        pos_end - end_pos,
        vel_start - start_vel,
        vel_end - end_vel,
        acc_start
    ]

    solution = solve(equations, (a4, a3, a2, a1, a0), dict=True)
    if not solution:
        raise ValueError("No solution found for polynomial coefficients.")

    coefficients = [float(solution[0][var]) for var in (a4, a3, a2, a1, a0)]
    return coefficients

# Υπολογισμός Συντελεστών για το Χ και Υ στην Ακίνηση
coeffs_acc_x = get_polynomial_coefficients(
    start_pos=start_x,
    end_pos=start_x + (end_x - start_x) * (phase_duration / half_time) * 0.6,  
    start_vel=0,
    end_vel=max_vel_x,
    start_t=0,
    end_t=phase_duration
)
position_acc_x = np.polyval(coeffs_acc_x, acceleration_phase)
velocity_acc_x = np.polyval(np.polyder(coeffs_acc_x), acceleration_phase)

coeffs_acc_y = get_polynomial_coefficients(
    start_pos=start_y,
    end_pos=start_y + (end_y - start_y) * (phase_duration / half_time) * 0.6,  
    start_vel=0,
    end_vel=max_vel_y,
    start_t=0,
    end_t=phase_duration
)
position_acc_y = np.polyval(coeffs_acc_y, acceleration_phase)
velocity_acc_y = np.polyval(np.polyder(coeffs_acc_y), acceleration_phase)

# Σταθερή Κίνηση Κατά τους Άξονες X και Y στη Δεύτερη Φάση
position_const_x = position_acc_x[-1] + velocity_acc_x[-1] * (constant_phase - constant_phase[0])
velocity_const_x = np.ones_like(constant_phase) * velocity_acc_x[-1]

position_const_y = position_acc_y[-1] + velocity_acc_y[-1] * (constant_phase - constant_phase[0])
velocity_const_y = np.ones_like(constant_phase) * velocity_acc_y[-1]

# Τρίτη Φάση για Αποτροπή στο Χ και Y
coeffs_dec_x = get_polynomial_coefficients(
    start_pos=position_const_x[-1],
    end_pos=end_x,
    start_vel=velocity_const_x[-1],
    end_vel=0,
    start_t=phase_duration + max_velocity_time,
    end_t=half_time
)
position_dec_x = np.polyval(coeffs_dec_x, deceleration_phase)
velocity_dec_x = np.polyval(np.polyder(coeffs_dec_x), deceleration_phase)

coeffs_dec_y = get_polynomial_coefficients(
    start_pos=position_const_y[-1],
    end_pos=end_y,
    start_vel=velocity_const_y[-1],
    end_vel=0,
    start_t=phase_duration + max_velocity_time,
    end_t=half_time
)
position_dec_y = np.polyval(coeffs_dec_y, deceleration_phase)
velocity_dec_y = np.polyval(np.polyder(coeffs_dec_y), deceleration_phase)

# Συνένωση των Φάσεων Κίνησης
pos_x_forward = np.concatenate((position_acc_x, position_const_x, position_dec_x))
vel_x_forward = np.concatenate((velocity_acc_x, velocity_const_x, velocity_dec_x))
time_forward = np.concatenate((acceleration_phase, constant_phase, deceleration_phase))

pos_y_forward = np.concatenate((position_acc_y, position_const_y, position_dec_y))
vel_y_forward = np.concatenate((velocity_acc_y, velocity_const_y, velocity_dec_y))

# Δημιουργία Επιστροφικής Κίνησης (B προς A)
pos_x_backward = pos_x_forward[::-1]
vel_x_backward = -vel_x_forward[::-1]
time_backward = time_forward + time_forward[-1]

pos_y_backward = pos_y_forward[::-1]
vel_y_backward = -vel_y_forward[::-1]

# Συνολική Κίνηση από Α προς Β και Πίσω
pos_x_total = np.concatenate((pos_x_forward, pos_x_backward))
vel_x_total = np.concatenate((vel_x_forward, vel_x_backward))
time_total_array = np.concatenate((time_forward, time_backward))

pos_y_total = np.concatenate((pos_y_forward, pos_y_backward))
vel_y_total = np.concatenate((vel_y_forward, vel_y_backward))

pos_z_total = np.ones_like(pos_x_total) * start_z  # Το Z παραμένει σταθερό

# Υπολογισμός Ταχυτήτων End-Effector
vel_x_total_calc = np.gradient(pos_x_total, time_total_array)
vel_y_total_calc = np.gradient(pos_y_total, time_total_array)
vel_z_total_calc = np.gradient(pos_z_total, time_total_array)

# Συνάρτηση Αντίστων Κινηματικών
def inverse_kinematics(desired_x, desired_y, desired_z, l0, l1, l2, l4, l5, l7):
    q1 = np.arctan2(desired_y, desired_x)

    d = np.sqrt((l5 + l7)**2 + l4**2)
    alpha = np.arctan2((l5 + l7), l4)

    if np.sin(q1) == 0:
        raise ValueError("Invalid configuration: sin(q1) is zero.")

    numerator = (desired_z - l0)**2 + (desired_y / np.sin(q1) - l1)**2 - l2**2 - d**2
    denominator = 2 * l2 * d
    arg = numerator / denominator

    arg = np.clip(arg, -1.0, 1.0)

    q3 = np.arccos(arg) - alpha

    beta = np.arctan2(d * np.sin(q3 + alpha), (l2 + d * np.cos(q3 + alpha)))

    q2 = np.arctan2((desired_y / np.sin(q1) - l1), (desired_z - l0)) - beta

    return q1, q2, q3

# Συνάρτηση Προώτων Κινηματικών
def forward_kinematics(q1, q2, q3, l0, l1, l2, l4, l5, l7):
    joints = []
    
    # Βάση
    x0, y0, z0 = 0, 0, 0
    joints.append((x0, y0, z0))
    
    # Σύνδεσμος l0
    x1, y1, z1 = 0, 0, l0
    joints.append((x1, y1, z1))
    
    # Σύνδεσμος l1
    x2 = l1 * np.cos(q1)
    y2 = l1 * np.sin(q1)
    z2 = l0
    joints.append((x2, y2, z2))
    
    # Σύνδεσμος l2
    x3 = x2 + l2 * np.cos(q1) * np.sin(q2)
    y3 = y2 + l2 * np.sin(q1) * np.sin(q2)
    z3 = z2 + l2 * np.cos(q2)
    joints.append((x3, y3, z3))
    
    # Σύνδεσμος l4
    x4 = x3 + l4 * np.cos(q1) * np.sin(q2 + q3)
    y4 = y3 + l4 * np.sin(q1) * np.sin(q2 + q3)
    z4 = z3 + l4 * np.cos(q2 + q3)
    joints.append((x4, y4, z4))
    
    # Τελικός Εκτελεστής
    xe = x4 + (l5 + l7) * np.cos(q1) * np.cos(q2 + q3)
    ye = y4 + (l5 + l7) * np.sin(q1) * np.cos(q2 + q3)
    ze = z4 - (l5 + l7) * np.sin(q2 + q3)
    joints.append((xe, ye, ze))
    
    return joints

# Υπολογισμός Αντίστων Κινηματικών για Όλο το Χρονοδιάγραμμα
q1_angles = []
q2_angles = []
q3_angles = []

for i in range(len(time_total_array)):
    try:
        q1, q2, q3 = inverse_kinematics(pos_x_total[i], pos_y_total[i], pos_z_total[i], l0, l1, l2, l4, l5, l7)
    except ValueError as e:
        print(f"Σφάλμα στην αντίστροφη κινηματική στο χρόνο {time_total_array[i]}: {e}")
        q1, q2, q3 = 0, 0, 0  # Αντικατάσταση με μηδενικές γωνίες σε περίπτωση σφάλματος
    q1_angles.append(q1)
    q2_angles.append(q2)
    q3_angles.append(q3)

q1_angles = np.array(q1_angles)
q2_angles = np.array(q2_angles)
q3_angles = np.array(q3_angles)

# Υπολογισμός Γωνιακών Ταχυτήτων
def compute_joint_velocities(q1, q2, q3, v_x, v_y, v_z):
    c1 = np.cos(q1)
    s1 = np.sin(q1)
    c2 = np.cos(q2)
    s2 = np.sin(q2)
    c23 = np.cos(q2 + q3)
    s23 = np.sin(q2 + q3)

    J = np.zeros((3, 3))

    J[0, 0] = -l1 * s1 - s1 * c23 * (l5 + l7) - l2 * s1 * s2 - l4 * s1 * s23
    J[0, 1] = c1 * l2 * c2 - c1 * s23 * (l5 + l7) + c1 * l4 * c23
    J[0, 2] = c1 * l4 * c23 - c1 * s23 * (l5 + l7)

    J[1, 0] = l1 * c1 + c1 * c23 * (l5 + l7) + l2 * c1 * s2 + l4 * c1 * s23
    J[1, 1] = s1 * l2 * c2 - s1 * s23 * (l5 + l7) + s1 * l4 * c23
    J[1, 2] = s1 * l4 * c23 - s1 * s23 * (l5 + l7)

    J[2, 0] = 0
    J[2, 1] = -l2 * s2 - c23 * (l5 + l7) - l4 * s23
    J[2, 2] = -c23 * (l5 + l7) - l4 * s23

    end_eff_velocities = np.array([v_x, v_y, v_z]).reshape(-1, 1)
    
    # Υπολογισμός Γωνιακών Ταχυτήτων με χρήση της Ψευδοαντίστροφης της Ιακωβιανής
    joint_vels = np.linalg.pinv(J) @ end_eff_velocities
    return joint_vels.flatten()

joint_velocities = np.zeros((3, len(time_total_array)))

for i in range(len(time_total_array)):
    joint_velocities[:, i] = compute_joint_velocities(
        q1_angles[i], q2_angles[i], q3_angles[i],
        vel_x_total_calc[i], vel_y_total_calc[i], vel_z_total_calc[i]
    )

q1_dot = joint_velocities[0, :]
q2_dot = joint_velocities[1, :]
q3_dot = joint_velocities[2, :]

# Γραφικές Παραστάσεις

# (1) Γωνίες Αρθρώσεων Συνατήσει του Χρόνου
plt.figure(figsize=(12, 6))
plt.plot(time_total_array, np.degrees(q1_angles), label='$\ theta_1$ (deg)', color='cyan')
plt.plot(time_total_array, np.degrees(q2_angles), label='$\ theta_2$ (deg)', color='magenta')
plt.plot(time_total_array, np.degrees(q3_angles), label='$\ theta_3$ (deg)', color='orange')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Γωνίες Αρθρώσεων (degrees)')
plt.title('Γωνίες Αρθρώσεων Συνατήσει του Χρόνου')
plt.legend()
plt.grid(True)
plt.show()

# (2) Γωνιακές Ταχύτητες Αρθρώσεων Συνατήσει του Χρόνου
plt.figure(figsize=(12, 6))
plt.plot(time_total_array, np.degrees(q1_dot), label='$\dot{\ theta}_1$ (deg/s)', color='navy')
plt.plot(time_total_array, np.degrees(q2_dot), label='$\dot{\ theta}_2$ (deg/s)', color='lime')
plt.plot(time_total_array, np.degrees(q3_dot), label='$\dot{\ theta}_3$ (deg/s)', color='gold')
plt.xlabel('Χρόνος (s)')
plt.ylabel('Γωνιακές Ταχύτητες Αρθρώσεων (degrees/s)')
plt.title('Γωνιακές Ταχύτητες Αρθρώσεων Συνατήσει του Χρόνου')
plt.legend()
plt.grid(True)
plt.show()
