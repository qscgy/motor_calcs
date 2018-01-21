import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint


def feet(inches):
    return inches/12


def ft_lbf(n_m):
    return n_m*0.7375621

# Tuples are (free speed (rps), stall torque (ft-lb), stall current (A))
MOTORS = {
    '775pro': (18730/60, ft_lbf(0.71), 134)
}

# Inputs
# ------

# Motor
motor_name = '775pro'
n_motors = 2

# Gear ratio
gr = 0.05

# Pulley diameter
pulley_dia = feet(2.256)   # ft

# Voltage to motor
voltage = 10.5      # V

# Efficiencies
velocity_eff = 0.9
torque_eff = 0.9
static_friction = 10    # lbf

# Elevator parameters
elevator = np.array([[11., feet(23.25)], [5., feet(25.375)], [5.5, feet(26.875)]])  # [weight (lbm), run (ft)]

# Integration parameters
dt = 0.001
start_height = 0.    # feet

# Intermediate calculations
# -------------------------

# Get useful numbers
motor = MOTORS[motor_name]
pulley_rad = pulley_dia/2

# Accumulators
cum_masses = np.cumsum(elevator[:, 0])
cum_heights = np.cumsum(elevator[:, 1])
max_height = cum_heights[-1]

# Torques
friction_torque = static_friction*pulley_rad
torques = (pulley_rad*cum_masses)+(np.ones(3)*friction_torque)  # Frictionless torques plus friction torque
stall_torque_fl = motor[1]*n_motors/gr   # frictionless stall torque

# Velocities
free_speed_fl = np.pi*pulley_dia*gr*motor[0]  # frictionless free speed (fps)
kv = 12/(free_speed_fl*velocity_eff)

# Accelerations
v_int = torques/(stall_torque_fl*torque_eff)*12  # v_intercept
max_accel = np.reciprocal(cum_masses*pulley_rad/(stall_torque_fl*torque_eff))   # t = rma -> a = t/rm
ka = np.reciprocal(max_accel/12)
resistance = 12/(motor[2]*n_motors)

# Euler's method simulation
# -------------------------
pos = [start_height]
t = [0.]
stage = 0
velocity = [0.]
v_const = v_int[0]
v_vel = [kv * velocity[-0]]
v_accel = [voltage - v_const - v_vel[0]]
accel = [v_accel[-1]*ka[stage]]
current = [(voltage-v_vel[0])/resistance]
print('t={0}, pos={1}, vel={2}, accel={3}'.format(t[-1], pos[-1], velocity[-1], accel[-1]))

while pos[-1] <= max_height:
    for i in range(stage, len(cum_heights)):
        if pos[-1] > cum_heights[i]:
            stage = i+1

    v_const = v_int[stage]
    t.append(np.round(t[-1] + dt, 9))
    pos.append(pos[-1]+velocity[-1]*dt)
    velocity.append(velocity[-1]+accel[-1]*dt)
    v_vel.append(velocity[-1]*kv)
    v_accel.append(voltage - v_vel[-1] - v_const)
    accel.append(ka[stage]*v_accel[-1])
    current.append(voltage-v_vel[-1])
    print('t={0}, pos={1}, vel={2}, accel={3}, v_accel={4}, v_const={5}'.format(t[-1], pos[-1], velocity[-1], accel[-1], v_accel[-1], v_const))


