# Robotics-I Project (2024-25)

This repository contains a simulation project for an industrial robotic manipulator – specifically, a KUKA-style robot with six rotational degrees of freedom. The project implements both the kinematic analysis and the simulation of a robotic arm executing a periodic linear trajectory between two points. It covers the following aspects:

- **Forward and Inverse Kinematics:**  
  Calculation of joint angles and joint velocities based on the desired end-effector position and motion.  
- **Trajectory Planning:**  
  Generation of smooth (continuous velocity) trajectories between two defined positions.  
- **Kinematic Simulation and Visualization:**  
  Plotting of end-effector positions, linear velocities, joint angles, and angular velocities over time; also includes a 3D animation of the robot’s motion.

## Project Structure

- **`robo_7a.py`**  
  Implements the basic simulation with trajectory generation and plots for the desired end-effector position and linear velocities.

- **`robo_7b.py`**  
  Expands on the simulation by computing the inverse and forward kinematics. It plots the joint angles and angular velocities over time.

- **`robo_7c.py`**  
  Provides a 3D animation of the robot manipulator as it follows the planned trajectory, showing intermediate configurations of the kinematic chain.

- **`Robotics-I_Project_2024-25_v1.pdf`**  
  Contains the project requirements and theoretical background. (See below for an English translation of this document.)

## How to Run the Project

1. **Dependencies:**  
   Make sure you have the following Python packages installed:
   - `numpy`
   - `matplotlib`
   - `sympy`

   You can install them via pip:
   ```bash
   pip install numpy matplotlib sympy
2. **Running the Simulations:**  
   - For plotting the end-effector trajectory and linear velocities, run:
     ```bash
     python robo_7a.py
     ```
   - For joint angles and angular velocities analysis, run:
     ```bash
     python robo_7b.py
     ```
   - For the 3D animation of the robotic arm motion, run:
     ```bash
     python robo_7c.py
     ```
## Project Background and Requirements

This project was developed as part of the "Robotics I: Analysis, Control, Laboratory" course for the academic year 2024-25. The main objectives include:

- Applying the Denavit-Hartenberg (D-H) method for establishing coordinate frames.
- Deriving both forward and inverse kinematic models (geometric and differential).
- Designing a smooth trajectory for the robot’s end-effector between two defined points on a horizontal plane.
- Simulating the kinematics and visualizing the results with plots and animations.

For a detailed description of the requirements, please refer to the requirements.pdf.


