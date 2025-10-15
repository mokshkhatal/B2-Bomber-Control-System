# B2-Bomber-Control-System
Intuitive Dynamics Understading:
  The B2 Bomber aircraft has a flying wing design with no vertical stabalizers and tricky yet beautiful control system. It has 8 control surfaces namely: 
  Split Drag Rudders, Outboard Elevons, Mid Elevons and Inboard Elevons. 
  Looking into the dynamics, the pitching moment is controlled by inboard and mid elevons, rolling moment by outboard elevons and yaw by the split drag rudders.

Workflow of This System:
  The system works by first generating a mathematical model understanding the dynamics of the aircraft. 
  Then trimming the model for a straight level flight further linearizing it around the same trim point.
  All of the above operations are performed by generating MATLAB scripts and integrating them in Simulink.
  This system is tested by simulating in Unreal Engine 5 using Simulink blocksets from Aerospace Toolbox.

The Program Files:
  The B2_6DOF_model.m file:
    This is a 6 DOF non-linear mathematical aircraft model for implemeting control system on a B2 Bomber. It has all the dynamics, mathematics necessary for applying the control system.
    It accepts two inputs: (X)current state of the system(12x1) and (U)control input(5x1). And gives one output: (XDOT)time derivative of the state vector(17x1).
    Many of the coefficients, constants are approximate values derived from similiar aircraft designs but are reliable to use for this purpose. For precise values wind tunnel testing and CFD is required.
  
  The cost_straight_level_6dof_october.m file:
    This is a mathematical function that measures the difference between a model's expected values and actual values, required for trimming the aircraft model for a straight level flight.
    It has the trim point defined which is set of values for X and U matrices for maintaining a straight level flight.
    It also has bounds set for limiting variations in control inputs and states of the model.
  
  The trim_6dof_straight_level_october.m file:
    This is a function which returns us the magical trim values for maintaining the flight straight and level. The trim point is a set of values of our control input and state vector which correspond
    to straight level flight of the aircraft. By taking the expected values from above cost function it calculates the trim values of X and U whose output are the expected values for straight level flight.
  
  The B2_6DOF_model_implicit.m file:
    Its just two lines of code for defining an implicit function of this model for linearizing.
  
  The ImplicitLinmod.m file:
    Just like the cost function which defines expected values for trimming, this function defines expected result for the linear system which will be generated. A linear system is basically an approximation 
    of the non-linear system around an operating point(the operating point is the trim point in this case). It's simply Y = A(X) + B(U).
  
  The LinearizeSymmetricDifference.m file:
    Like the earlier trim function which calculates the values according to cost function, this function calculates the coefficients A and B for the linear model. Now this linear model can be used as a proxy
    for our original non-linear model while strictly adhering to range of the operating point.

Simulation and Testing:
  All these scripts are integrated in Simulink to make a custom block of the non-linear as well as linear model of the aircraft. The block is given inputs from a joystick whose block is available in the
  Aerospace Toolbox. Outputs from the linear/non-linear blocks and joystick are displayed in the scope.
  For better visualization, this process is simulated in Unreal Engine 5 using the aided blocks from Aerospace blockset for customizing aircraft model and simualation environment.

Approximations and Boundaries of Usage for This Version:
  This version is a basic replication attempt of the control system solely for educational purpose so it has certain limitations due to unavailability of wind tunnel testing and CFD data.
  Many assumptions are also made for simplicity of the project whcich do not significantly affect its resemblance from real life. Key assumptions and limitations:
  > The air density is taken as a constant mean value for range 0-10000 ft.
  > Time taken by control surfaces to move is considered 0.
  > Mid elevons and inboard elevons are considered to be single inborad elevons for simplicity.
  > 4 Engines are replaced by 2 engines while keeping the total thrust same for simplicity.
  > Split drag rudders also produce a rolling moment which needs to be countered by elevons.
  > Coefficients of relation between moment and control surface delfection are derived from CFD data for flying wing design not specifically B2 bomber airframe.
  > The skeletal mesh of the aircraft used for simulation does not include animations for control surface delfelections to keep it simple.
  > Takeoff/Landing system is completely ignored as the flight simulation is operated for a straight level trim point.

Further Scopes of Improvement:
  The system is currently trimmed and linearized of straight level flight which can later be appended fo more operating points providing more maneuvers and takeoff/landing.
  More accurate CFD can be perfomed for precise coefficents and constants required in the mathematical modelling part.
  PID and GLAS can be implemented for fine performance in real world applications.
  More control authority can be gained by differtially controlling every control surface for better stability.
  
