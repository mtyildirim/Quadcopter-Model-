function x_dot = Quadcopter_Dynamic_Model_And_Equations_Of_Motion(x,u)

x1=x(1);        % U
x2=x(2);        % V
x3=x(3);        % W
x4=x(4);        % P
x5=x(5);        % Q                 States
x6=x(6);        % R
x7=x(7);        % Phi
x8=x(8);        % Theta
x9=x(9);        % Psi

%% Initialise Parameters

u1=u(1);        % Engine Rotational Speed 1
u2=u(2);        % Engine Rotational Speed 2
u3=u(3);        % Engine Rotational Speed 3
u4=u(4);        % Engine Rotational Speed 4

g = 9.81;        % Gravity 
M =    ;         % Drone Mass , Kg
l =    ;         % Arm Lenght , M 

Ixx =     ;      % Drone X Axis Inertia
Iyy =     ;      % Drone Y Axis Inertia
Izz =     ;      % Drone Z Axis Inertia

Kt =       ;     % Thrust Coefficient
Kq =       ;     % Torque Coefficient

%%

Total_Thrust = Kt*(u1^2+u2^2+u3^2+u4^2);            % Total Thrust
Torque_roll  = Kt*l*((u2^2+u3^2)-(u1^2+u4^2));      % Roll Axis Torque
Torque_pitch = Kt*l*((u1^2+u2^2)-(u3^2+u4^2));      % Pitch Axis Torque
Torque_yaw   = Kq*((u1^2+u3^2)-(u2^2+u4^2));        % Yaw Axis Torque

P_Dot = (Torque_roll/Ixx);                          % X Axis Angular Acceleration
Q_Dot = (Torque_pitch/Iyy);                         % Y Axis Angular Acceleration
R_Dot = (Torque_yaw/Izz);                           % Z Axis Angular Acceleration

Wbe = [x4;x5;x6];

Ax = -g*x8;                                         % X Axis Linear Acceleration
Ay =  g*x7;                                         % Y Axis Linear Acceleration
Az =  g-Total_Thrust/M;                             % Z Axis Linear Acceleration

A = [Ax;Ay;Az];

C1 = [cos(x9) sin(x9) 0 ; -sin(x9) cos(x9) 0 ; 0 0 1];
C2 = [cos(x8) 0 -sin(x8);0 1 0 ; sin(x8) 0 cos(x8)];                        % Direction Cosine Matrix
C3 = [1 0 0 ; 0 cos(x7) sin(x7); 0 -sin(x7) cos(x7)];
Cw_b = C1*C2*C3;
DCM = transpose(Cw_b);

U_Dot = Ax + (x6*x2-x2*x5)*g*cos(x8);               % X Axis Velocity(Body)
V_Dot = Ay + (x4*x3-x1*x6)*sin(x7)*cos(x8);         % Y Axis Velocity(Body)
W_Dot = Az + (x5*x1-x4*x2)*cos(x7)*cos(x8);         % Z Axis Velocity(Body)

x1to3dot = [U_Dot;V_Dot;W_Dot];
x4to6dot = [P_Dot;Q_Dot;R_Dot];                     % X_dot 
x7to9dot = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);0 cos(x7) -sin(x7);0 sin(x7)*cos(x8) cos(x7)*cos(x8)]*Wbe;
x_dot    = [x1to3dot;x4to6dot;x7to9dot];






