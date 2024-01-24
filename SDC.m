function SD = SDC(Aircraft_Data,CSV,ATF,ATM)
% Help
% State Derivatives Calculator (SDC)
% This function outputs a vector comprising the state derivatives as:
% SD = [u_dot;v_dot;w_dot;p_dot;q_dot;r_dot;phi_dot;theta_dot;psi_dot;xE_dot;yE_dot;zE_dot]

%% Definitions
% BA is a vector comprising Body Accelerations; u_dot, v_dot and w_dot
% BRA is a vector comprising Body Rotational Accelerations; p_dot, q_dot, and r_dot
% ERR is a vector comprising Euler Rotation Rates; phi_dot, theta_dot and psi_dot
% IV is a vector comprising Inertial Velocities; xE_dot, yE_dot and zE_dot

%% Current State
 BV = CSV(1:3);   % Body Velocities
BRR = CSV(4:6);   % Body Rotation Rates
 ER = CSV(7:9);   % Euler Rotations

%% Coordinate Transformation Matrix
% For transformation from body to inertial (Earth) axes
  T = [cos(ER(2))*cos(ER(3))   (sin(ER(1))*sin(ER(2))*cos(ER(3)) -cos(ER(1))*sin(ER(3)))   (cos(ER(1))*sin(ER(2))*cos(ER(3)) +sin(ER(1))*sin(ER(3)))
       cos(ER(2))*sin(ER(3))   (sin(ER(1))*sin(ER(2))*sin(ER(3)) +cos(ER(1))*cos(ER(3)))   (cos(ER(1))*sin(ER(2))*sin(ER(3)) -sin(ER(1))*cos(ER(3)))
                 -sin(ER(2))                                       sin(ER(1))*cos(ER(2))                                       cos(ER(1))*cos(ER(2))];

%% Kinetics
 GF = T.'*[0;0;Aircraft_Data.m*Aircraft_Data.g];   % Gravitational Forces
  F = ATF +GF;   % Total Forces
  M = ATM;   % Total Moments
 BA = F/Aircraft_Data.m -cross(BRR,BV);
BRA = Aircraft_Data.I\(M -cross(BRR,Aircraft_Data.I*BRR));

%% Kinematics
  J = [1   sin(ER(1))*tan(ER(2))   cos(ER(1))*tan(ER(2))
       0              cos(ER(1))             -sin(ER(1))
       0   sin(ER(1))/cos(ER(2))   cos(ER(1))/cos(ER(2))];
ERR = J*BRR;
 IV = T*BV;

%% State Derivatives
 SD = [BA;BRA;ERR;IV];

end