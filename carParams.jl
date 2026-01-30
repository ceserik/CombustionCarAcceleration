

Base.@kwdef struct Car
    # Weight without pilot [kg]
    weight_0::Float64
    
    # Weight with pilot [kg]
    weight_p::Float64
    
    # Weight on wheels [kg]
    weight_fl::Float64
    weight_fr::Float64
    weight_rl::Float64
    weight_rr::Float64
    
    # Wheel radius [mm]
    front_wheel_radius::Float64
    rear_wheel_radius::Float64
    
    # Gearbox ratios
    gearbox_ratios::Vector{Float64}
    differential_ratio::Float64

    #Motor
    torqueMax::Float64

end

# Create car instance with default parameters
car = Car(
    weight_0 = 1350,
    weight_p = 1420,
    weight_fl = 352,
    weight_fr = 353,
    weight_rl = 358,
    weight_rr = 357,
    front_wheel_radius = 317.85/1000,
    rear_wheel_radius = 321.35/1000,
    gearbox_ratios = [4.23, 2.52, 1.66, 1.22, 1],
    differential_ratio = 3.15,
    torqueMax = 500,
    
)