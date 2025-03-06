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

# Γραφικές Παράστασης

# (1) Επιθυμητή Θέση End-Effector στον Χρόνο
plt.figure(figsize=(12, 5))
plt.plot(time_total_array, pos_x_total, label="pEx(t)", color='red')
plt.plot(time_total_array, pos_y_total, label="pEy(t)", color='orange')
plt.plot(time_total_array, pos_z_total, label="pEz(t)", color='teal')
plt.xlabel("Χρόνος (s)")
plt.ylabel("Θέση (cm)")
plt.title("Επιθυμητή Θέση End-Effector στον Χρόνο")
plt.grid(True)
plt.legend()
plt.show()

# (2) Γραμμικές Ταχύτητες End-Effector στον Χρόνο
plt.figure(figsize=(12, 5))
plt.plot(time_total_array, vel_x_total_calc, label="vx(t)", color='magenta')
plt.plot(time_total_array, vel_y_total_calc, label="vy(t)", color='cyan')
plt.plot(time_total_array, vel_z_total_calc, label="vz(t)", color='lime')
plt.axhline(0, color='black', linestyle='--', linewidth=0.8)
plt.xlabel("Χρόνος (s)")
plt.ylabel("Ταχύτητα (cm/s)")
plt.title("Γραμμικές Ταχύτητες End-Effector στον Χρόνο")
plt.grid(True)
plt.legend()
plt.show()
