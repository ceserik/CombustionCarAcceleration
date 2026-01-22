a₁ = -1/5                   # Definition of the hybrid system that switches between two linear modes 
a₂ = 1                    # x(k+1) = a₁*x(k) + b₁*u(k) and x(k+1) = a₂*x(k) + b₂*u(k)
a₃ = -2

b₁ = 1                    # depending on whether x(k) < 0 or x(k) >= 0, respectively.
b₂ = 1
b₃ = 0.5

x₀ = -3.0                   # Initial state.
N_full = 200                # Length of the simulation discrete-time horizon.
h = 0.1                     # Sampling period. Just for creating the reference signal. 
t = 0:h:N_full*h            # Vector of continuous-time instants. Just for creating the reference signal.
x_ref_full = sin.(t)        # [x_ref(0), x_ref(1), ..., x_ref(N_full-1), x_ref(N_full)]

x_min, x_max = -10.0, 10.0  # Lower and upper bounds on the state and the control input.
u_min, u_max = -10.0, 100    # Unfortunately, these are not used in the formulation below and must be entered as literals there.

using JuMP
using HiGHS
#using Gurobi
import ParametricOptInterface as POI

rhocp = Model(() -> POI.Optimizer(HiGHS.Optimizer()))    # Defining the receding-horizon (RH) optimal control problem (OCP).
set_silent(rhocp)

N = 20                                              # Prediction horizon for the MPC controller.
ϵ = 1e-6                                            # Small positive constant for strict inequalities.
@variable(rhocp, first_gear[1:N], Bin)                       # [first_gear(k), first_gear(k+1), ..., first_gear(k+N-1)]
@variable(rhocp, second_gear[1:N], Bin)
@variable(rhocp, third_gear[1:N], Bin)
@variable(rhocp, fourth_gear[1:N], Bin)


@variable(rhocp, -10.0 <= z[i=1:N] <= 10.0)         # [z(k), z(k+1), ..., z(k+N-1)]
@variable(rhocp, -10.0 <= x[i=1:N+1] <= 10.0)       # [x(k), x(k+1), ..., x(k+N-1), x(k+N)] 
@variable(rhocp, u_min<= u[i=1:N] <= u_max)           # [u(k), u(k+1), ..., u(k+N-1)]
@variable(rhocp, t)                                 # Aux variable for formulating the abs value in the objective.
@variable(rhocp, x_cur in MOI.Parameter(0))         # Current state x(k)        
@variable(rhocp, x_ref[1:N+1] in MOI.Parameter(0))  # [x_ref(k), x_ref(k+1), ..., x_ref(k+N)]
@constraint(rhocp, x[1] == x_cur)                   # State at the beginning of the prediction horizon.
for i in 1:N                                        # i corresponds to time step k+i-1
    # Exactly one gear must be active at each time step
    @constraint(rhocp, first_gear[i] + second_gear[i] + third_gear[i] + fourth_gear[i] == 1)
    
    @constraint(rhocp, first_gear[i] --> {z[i] == a₁ * x[i] + b₁ * u[i]})
    @constraint(rhocp, second_gear[i] --> {z[i] == a₂ * x[i] + b₂ * u[i]})
    @constraint(rhocp, third_gear[i] --> {z[i] == a₃ * x[i] + b₃ * u[i]})
    @constraint(rhocp, fourth_gear[i] --> {z[i] == a₂ * x[i] + b₂ * u[i]})
    
    @constraint(rhocp, x[i+1] == z[i])
end
@constraint(rhocp, [t; (x-x_ref)] in MOI.NormOneCone(1 + length(x)))
@objective(rhocp, Min, t)
@objective(rhocp, Min, sum((x[i]-x_ref[i])^2 for i in 1:N+1))

function mpc_tracking(rhocp, xₖ, x_ref_full, k, N)          # Function that solves the RH OCP 
    set_parameter_value(x_cur,xₖ)                           #   for the current state x_cur = x(k),
    for i in 1:(N+1)
        set_parameter_value(x_ref[i], x_ref_full[k+i-1])    #   and the full reference trajectory, from which the relevant portion is extracted.
    end
    optimize!(rhocp)
    return value.(u), value.(first_gear), value.(second_gear), value.(third_gear), value.(fourth_gear)
end

xopt_full = [x₀,]
uopt_full = Float64[]
first_gearopt_full = Float64[]
second_gearopt_full = Float64[]
third_gearopt_full = Float64[]
fourth_gearopt_full = Float64[]

for k in 1:(N_full-N)                                           # Receding-horizon simulation loop.  
    xₖ = xopt_full[end]                                         # Current state.  
    u_pred, g1, g2, g3, g4 = mpc_tracking(rhocp, xₖ, x_ref_full, k, N)  # Optimal control and gear sequences
    uₖ = u_pred[1]                                              # Apply only the first control input.
    # Update state according to the selected gear (from optimizer)
    x⁺ = g1[1] == 1 ? (a₁ * xₖ + b₁ * uₖ) :
         g2[1] == 1 ? (a₂ * xₖ + b₂ * uₖ) :
         g3[1] == 1 ? (a₃ * xₖ + b₃ * uₖ) :
                      (a₂ * xₖ + b₂ * uₖ)
    push!(xopt_full, x⁺)
    push!(uopt_full, uₖ)
    push!(first_gearopt_full, g1[1])
    push!(second_gearopt_full, g2[1])
    push!(third_gearopt_full, g3[1])
    push!(fourth_gearopt_full, g4[1])
end


using Plots

# Create active gear indicator (1, 2, 3, or 4)
active_gear = [first_gearopt_full[i]*1 + second_gearopt_full[i]*2 + third_gearopt_full[i]*3 + fourth_gearopt_full[i]*4 for i in 1:length(first_gearopt_full)]

p1 = plot(0:h:(N_full-N)*h, xopt_full, xlims=(-0.1, N_full*h*1.01), label="x(k)", xlabel="k", ylabel="x(k)", markershape=:circle, markersize=2, linetype=:steppost)
plot!(p1, 0:h:N_full*h, x_ref_full, label="x_ref(k)")
p2 = plot(0:h:(N_full-N-1)*h, uopt_full, xlims=(-0.1, N_full*h*1.01), label="u(k)", xlabel="k", ylabel="u(k)", markershape=:circle, markersize=2, linetype=:steppost)    
p3 = plot(0:h:(N_full-N-1)*h, active_gear, xlims=(-0.1, N_full*h*1.01), label="Active Gear", xlabel="k", ylabel="Gear", markershape=:circle, markersize=3, linetype=:steppost, ylims=(0.5, 4.5), yticks=1:4)
plot(p1, p2, p3, layout=(3,1), size=(600,800))