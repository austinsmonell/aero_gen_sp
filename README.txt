Austin Monell
09/15/2023

AeroGen: Single Kite with Fixed Tether Length Configuration

Overview
This repository contains a non-linear simulation of an Airborne Wind Turbine (AWT) system implemented using MATLAB and Simulink. The simulation is designed to model the dynamics of a Ground-Gen AWES (Airborne Wind Energy System). It features a 6-degree-of-freedom rigid wing aircraft with constant stability derivatives, connected to an elastic tether. The aircraft is equipped with elastic stabilizers in the lateral and longitudinal axes to control its attitude relative to the tether axis. Think of these stabilizers as analogous to the bridle lines of a traditional kite. The simulation is initialized in a steady-state cycle and can be controlled either using control surfaces or by adjusting the tether's relative attitude.

Simulation Components
1. Aircraft Model
The core component of the simulation is a 6-degree-of-freedom rigid wing aircraft model. It includes:

Position and Orientation: Tracking of the aircraft's position and orientation in 3D space with respect to a global coordinate system.

Linear and Angular Velocities: Description of the aircraft's translational and rotational motion through linear and angular velocities.

2. Elastic Tether
The aircraft is tethered to the ground through an elastic tether. Key characteristics of the tether include:

No Modeled Mass or Drag: The tether is simplified in the simulation, assuming it has no mass or aerodynamic drag, allowing focus on the aircraft's behavior.
3. Elastic Stabilizers
To control the aircraft's attitude relative to the tether axis, elastic stabilizers are integrated into the simulation. These stabilizers function similarly to bridle lines on a traditional kite and are responsible for maintaining the aircraft in a specific attitude relative to the tether axis.

Simulation Initialization
The simulation begins in a steady-state cycle to ensure the system starts in a stable configuration before control actions are applied.

Control Mechanisms
Control of the AWT system can be achieved through two primary mechanisms:

1. Control Surfaces
The aircraft is equipped with control surfaces that allow for adjustments in pitch, roll, and yaw. These control surfaces influence the aerodynamic forces and moments acting on the aircraft, enabling dynamic control.

2. Tether Relative Attitude Adjustment
Alternatively, the simulation permits adjustments to the attitude of the aircraft relative to the tether axis. This adjustment influences the angle at which the aircraft is tethered to the ground, thereby impacting its aerodynamic behavior.

How to Use
1. Clone or download this repository to your local machine.
2. Open the MATLAB/Simulink project file to access the simulation model.
   Important: Ensure you have MATLAB 2021b or a compatible version installed on your system.
3. To initialize the model, run the run_sim.m script. This script sets up the necessary initial conditions and system parameters for the simulation.
4. After initializing the model, you can choose one of the following methods to begin simulating the dynamic model:
	a. Using Simulink:
		Open the Simulink model within the project.
		Click the "Run" button in Simulink to start the simulation.
	b. Using the run_sim_states script:
		Run the run_sim_states.m script to execute the simulation with dynamic state updates.
		This script handles the integration of control inputs and updates the simulation state over time use the ODEXXX function.
		Monitor and analyze the simulation outputs to evaluate the behavior of the Airborne Wind Turbine (AWT) system.

Output
The simulation provides various outputs for analysis, including:

1. Aircraft position and orientation.
2. Linear and angular velocities.
These outputs enable the evaluation of system performance and can be used for further analysis and optimization.

Conclusion
This repository contains a comprehensive non-linear simulation of an Airborne Wind Turbine (AWT) system using MATLAB and Simulink. The simulation represents a 6-degree-of-freedom rigid wing aircraft connected to an elastic tether with elastic stabilizers. It allows for control via both control surfaces and adjustments to the tether's relative attitude, making it a valuable tool for studying and optimizing AWT systems. Feel free to explore, modify, and adapt the simulation for your specific research or engineering needs.




