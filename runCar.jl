
using JuMP
using HiGHS
import ParametricOptInterface as POI
include("carParams.jl")


time_step = 0.5
N = 15

## system Definition
#v̇ = (car.torqueMax *Ratio*R_d *r) -v/(r * gearbox_ratio * R_d * 3)
#ṡ = v

rhocp = Model(() -> POI.Optimizer(HiGHS.Optimizer()))

@variable(rhocp, first_gear[1:N], Bin)                       # [first_gear(k), first_gear(k+1), ..., first_gear(k+N-1)]
@variable(rhocp, second_gear[1:N], Bin)
@variable(rhocp, third_gear[1:N], Bin)
@variable(rhocp, fourth_gear[1:N], Bin)
@variable(rhocp, fifth_gear[1:N], Bin)
@variable(rhocp, over_rev[1:N], Bin)
@variable(rhocp, 0 <= v[1:N+1] <= 1000)  # velocity in m/s, bounded [0, 100]
@variable(rhocp, -10 <= z[1:N] <= 10)   # velocity change in m/s per time step
@variable(rhocp, v_cur in MOI.Parameter(0))  
@constraint(rhocp, v[1] == v_cur)    


m = car.weight_p
R_d = car.differential_ratio
r = car.rear_wheel_radius
R_gb = car.gearbox_ratios

for i in 1:N                                        # i corresponds to time step k+i-1
    
    @constraint(rhocp, first_gear[i] +second_gear[i] + third_gear[i] == 1)
    
    
    @constraint(rhocp, first_gear[i] -->  {z[i] == (car.torqueMax + 3*(470-v[i] * R_d * R_gb[1] )/r) * R_gb[1] *R_d * r /m * time_step})
    @constraint(rhocp, second_gear[i] --> {z[i] == (car.torqueMax + 3*(470-v[i] * R_d * R_gb[2] )/r) * R_gb[2] *R_d * r /m * time_step})
    @constraint(rhocp, third_gear[i] -->  {z[i] == (car.torqueMax + 3*(470-v[i] * R_d * R_gb[3] )/r) * R_gb[3] *R_d * r /m * time_step})
    #@constraint(rhocp, fourth_gear[i] --> {z[i] == (car.torqueMax + 3*(470-v[i] * R_d * R_gb[4] )/r) * R_gb[4] *R_d * r /m * time_step})
    
    @constraint(rhocp, v[i+1] == v[i] + z[i])
end

# Objective: maximize final velocity
@objective(rhocp, Max, v[N+1])

optimize!(rhocp)

# Extract results
velocity = value.(v)
time_pts = 0:time_step:(N*time_step)
time_steps = time_step:time_step:(N*time_step)

# Extract gear selections
gear_1 = value.(first_gear)
gear_2 = value.(second_gear)
gear_3 = value.(third_gear)
# gear_4 = value.(fourth_gear)

# Determine active gear at each time step
active_gear = zeros(Int, N)
for i in 1:N
    if gear_1[i] > 0.5
        active_gear[i] = 1
    elseif gear_2[i] > 0.5
        active_gear[i] = 2
    elseif gear_3[i] > 0.5
        active_gear[i] = 3
    # elseif gear_4[i] > 0.5
    #     active_gear[i] = 4
    end
end

# Calculate motor RPM for each time step
r_m = car.rear_wheel_radius
motor_rpm = zeros(N+1)
motor_torque = zeros(N+1)
for i in 1:(N+1)
    if i == 1
        gear_idx = 1  # Initial gear
    else
        gear_idx = active_gear[i-1]
    end
    # Motor RPM = (v / r) * differential_ratio * gearbox_ratio * (60 / 2π)
    motor_rpm[i] = (velocity[i] / r_m) * R_d * R_gb[gear_idx] * (60 / (2 * π))
    # Motor torque calculation from the constraint equation
    motor_torque[i] = car.torqueMax + 3*(470 - velocity[i] * R_d * R_gb[gear_idx] / r)
end

# Plot
using Plots
p1 = plot(time_pts, velocity, 
     xlabel="Time (s)", 
     ylabel="Velocity (m/s)",
     title="Car Velocity vs Time",
     linewidth=2,
     marker=:circle,
     legend=false)

p2 = plot(time_steps, active_gear,
     xlabel="Time (s)",
     ylabel="Gear",
     title="Gear Selection vs Time",
     linewidth=2,
     marker=:square,
     legend=false,
     yticks=1:5)

p3 = plot(time_pts, motor_rpm,
     xlabel="Time (s)",
     ylabel="Motor RPM",
     title="Motor Speed vs Time",
     linewidth=2,
     marker=:circle,
     legend=false)

p4 = plot(time_pts, motor_torque,
     xlabel="Time (s)",
     ylabel="Motor Torque (Nm)",
     title="Motor Torque vs Time",
     linewidth=2,
     marker=:circle,
     legend=false)

plot(p1, p2, p3, p4, layout=(4,1), size=(800, 1200))
