
using JuMP
using HiGHS
import ParametricOptInterface as POI
include("carParams.jl")


time_step = 0.1
N = 80


rhocp = Model(() -> POI.Optimizer(HiGHS.Optimizer()))

@variable(rhocp, first_gear[1:N], Bin)                       # [first_gear(k), first_gear(k+1), ..., first_gear(k+N-1)]
@variable(rhocp, second_gear[1:N], Bin)
@variable(rhocp, third_gear[1:N], Bin)
@variable(rhocp, fourth_gear[1:N], Bin)
@variable(rhocp, fifth_gear[1:N], Bin)

@variable(rhocp, 0 <= v[1:N+1] <= 60)  # velocity in m/s, bounded [0, 100] 
@variable(rhocp, 0 <= z[1:N] <= 3)   
@variable(rhocp, v_cur in MOI.Parameter(0))  
@variable(rhocp,-2 <= u[1:N] <= 500)
@constraint(rhocp, v[1] == v_cur)    


m = car.weight_p
R_d = car.differential_ratio
r = car.rear_wheel_radius
R_gb = car.gearbox_ratios

for i in 1:N                                        # i corresponds to time step k+i-1
    # XOR to force exactly one gear ratio to be active at all times
    @constraint(rhocp, first_gear[i] + second_gear[i] + third_gear[i] + fourth_gear[i] + fifth_gear[i] == 1)
    #@constraint(rhocp, ==0)
    
    #traction force from motor to wheels
    @constraint(rhocp, first_gear[i]  -->  {z[i] == (u[i] * R_gb[1] *R_d)/(m * r) * time_step})
    @constraint(rhocp, second_gear[i] -->  {z[i] == (u[i] * R_gb[2] *R_d)/(m * r) * time_step})
    @constraint(rhocp, third_gear[i]  -->  {z[i] == (u[i] * R_gb[3] *R_d)/(m * r) * time_step})
    @constraint(rhocp, fourth_gear[i] -->  {z[i] == (u[i] * R_gb[4] *R_d)/(m * r) * time_step})
    @constraint(rhocp, fifth_gear[i]  -->  {z[i] == (u[i] * R_gb[5] *R_d)/(m * r) * time_step})

    #ramp up of torque
    @constraint(rhocp, first_gear[i]  -->  {u[i] <= 52/235 * v[i] * R_d * R_gb[1] / r + 400})
    @constraint(rhocp, second_gear[i] -->  {u[i] <= 52/235 * v[i] * R_d * R_gb[2] / r + 400})
    @constraint(rhocp, third_gear[i]  -->  {u[i] <= 52/235 * v[i] * R_d * R_gb[3] / r + 400})
    @constraint(rhocp, fourth_gear[i] -->  {u[i] <= 52/235 * v[i] * R_d * R_gb[4] / r + 400})
    @constraint(rhocp, fifth_gear[i]  -->  {u[i] <= 52/235 * v[i] * R_d * R_gb[5] / r + 400})


    #peak torque to over rev transition
    @constraint(rhocp, first_gear[i]  -->  {u[i] <= 500 - 0.5458 * (v[i] * R_d * R_gb[1] / r - 471.2389)})
    @constraint(rhocp, second_gear[i] -->  {u[i] <= 500 - 0.5458 * (v[i] * R_d * R_gb[2] / r - 471.2389)})
    @constraint(rhocp, third_gear[i]  -->  {u[i] <= 500 - 0.5458 * (v[i] * R_d * R_gb[3] / r - 471.2389)})
    @constraint(rhocp, fourth_gear[i] -->  {u[i] <= 500 - 0.5458 * (v[i] * R_d * R_gb[4] / r - 471.2389)})
    @constraint(rhocp, fifth_gear[i]  -->  {u[i] <= 500 - 0.5458 * (v[i] * R_d * R_gb[5] / r - 471.2389)})

    #over rev
    @constraint(rhocp, first_gear[i]  -->  {u[i]/500 <= (420 - 26.7324 * (v[i] * R_d * R_gb[1] / r - 617.7605))/500})
    @constraint(rhocp, second_gear[i] -->  {u[i]/500 <= (420 - 26.7324 * (v[i] * R_d * R_gb[2] / r - 617.7605))/500})
    @constraint(rhocp, third_gear[i]  -->  {u[i]/500 <= (420 - 26.7324 * (v[i] * R_d * R_gb[3] / r - 617.7605))/500})
    @constraint(rhocp, fourth_gear[i] -->  {u[i]/500 <= (420 - 26.7324 * (v[i] * R_d * R_gb[4] / r - 617.7605))/500})
    @constraint(rhocp, fifth_gear[i]  -->  {u[i]/500 <= (420 - 26.7324 * (v[i] * R_d * R_gb[5] / r - 617.7605))/500})
    
    @constraint(rhocp, v[i+1] == v[i] + z[i]  )
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
gear_4 = value.(fourth_gear)
gear_5 = value.(fifth_gear)
u_vals = value.(u)

# Determine active gear at each time step
active_gear = zeros(Int, N)
for i in 1:N
    if gear_1[i] > 0.5
        active_gear[i] = 1
    elseif gear_2[i] > 0.5
        active_gear[i] = 2
    elseif gear_3[i] > 0.5
        active_gear[i] = 3
    elseif gear_4[i] > 0.5
         active_gear[i] = 4
    elseif gear_5[i] > 0.5
         active_gear[i] = 5
    end
end

# Calculate motor RPM for each time step
r_m = car.rear_wheel_radius
motor_rpm = zeros(N)
for i in 1:N
    gear_idx = active_gear[i]
    # Motor RPM = (v / r) * differential_ratio * gearbox_ratio * (60 / 2π)
    motor_rpm[i] = (velocity[i] / r_m) * R_d * R_gb[gear_idx] * (60 / (2 * π))
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

p3 = plot(time_steps, u_vals,
     xlabel="Time (s)",
     ylabel="Control Input u (Nm)",
     title="Control Input vs Time",
     linewidth=2,
     marker=:circle,
     legend=false)

p4 = plot(time_steps, motor_rpm,
     xlabel="Time (s)",
     ylabel="Motor RPM",
     title="Motor RPM vs Time",
     linewidth=2,
     marker=:circle,
     legend=false)

p5 = scatter(motor_rpm, u_vals,
     xlabel="Motor RPM",
     ylabel="Torque (Nm)",
     title="Torque vs RPM",
     marker=:circle,
     legend=false,
     markersize=4)


plot(p1, p2, p3, p4, p5, layout=(5,1), size=(800, 1500))
