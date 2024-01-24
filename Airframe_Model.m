function varargout = Airframe_Model(varargin)
% Help
% This function calculates the change in Aerodynamic and Thrust Forces and
% Moments.
%
% This function can be used with:
%    1) one input only (Aircraft_Data) to output [MI_SCDer], a matrix
%       comprising the stability and control derivatives of the airplane
%       multiplied by the airplane's mass and appropriate moments of
%       inertia
%    2) three inputs (Aircraft_Data,CSV,CSD) to output the change in
%       aerodynamic and thrust forces and moments [Delta_ATF,Delta_ATM],
%       calculated according to the following equations:
%       Delta_X = Aircraft_Data.m*(X_u*Delta_u +X_w*Delta_w +X_de*Delta_de
%                    +X_dT*Delta_dT)
%       Delta_Y = Aircraft_Data.m*(Y_v*Delta_v +Y_p*Delta_p +Y_r*Delta_r
%                    +Y_dr*Delta_dr +Y_beta*Delta_beta +Y_da*Delta_da)
%       Delta_Z = Aircraft_Data.m*(Z_u*Delta_u +Z_w*Delta_w +Z_q*Delta_q
%                    +Z_wdot*Delta_wdot +Z_de*Delta_de +Z_dT*Delta_dT)
%       Delta_L = Ixx*(L_v*Delta_v +L_p*Delta_p +L_r*Delta_r +L_dr*Delta_dr
%                    +L_da*Delta_da +L_beta*Delta_beta)
%       Delta_M = Iyy*(M_u*Delta_u +M_w*Delta_w +M_q*Delta_q
%                    +M_wdot*Delta_wdot +M_de*Delta_de +M_dT*Delta_dT)
%       Delta_N = Izz*(N_v*Delta_v +N_p*Delta_p +N_r*Delta_r +N_dr*Delta_dr
%                    +N_da*Delta_da +N_beta*Delta_beta)

%% Aircraft Data
Aircraft_Data = varargin{1};

%% Longitudinal Derivatives
% Stability
 X_u = Aircraft_Data.Long_Der(1);    Z_u = Aircraft_Data.Long_Der(2);    M_u = Aircraft_Data.Long_Der(3);
 X_w = Aircraft_Data.Long_Der(4);    Z_w = Aircraft_Data.Long_Der(5);    M_w = Aircraft_Data.Long_Der(6);
                Z_wdot = Aircraft_Data.Long_Der(7);   M_wdot = Aircraft_Data.Long_Der(9);
                   Z_q = Aircraft_Data.Long_Der(8);      M_q = Aircraft_Data.Long_Der(10);

% Control
X_de = Aircraft_Data.Long_Der(11);  Z_de = Aircraft_Data.Long_Der(12);  M_de = Aircraft_Data.Long_Der(13);
X_dT = Aircraft_Data.Long_Der(14);  Z_dT = Aircraft_Data.Long_Der(15);  M_dT = Aircraft_Data.Long_Der(16);

%% Lateral Derivatives
% Stability
   L_v = 0;                            N_v = 0;                            Y_v = Aircraft_Data.Lat_Der(1);
   L_p = Aircraft_Data.Lat_Der(3);     N_p = Aircraft_Data.Lat_Der(4);     Y_p = 0;
   L_r = Aircraft_Data.Lat_Der(5);     N_r = Aircraft_Data.Lat_Der(6);     Y_r = 0;
L_beta = Aircraft_Data.Lat_Der(7);  N_beta = Aircraft_Data.Lat_Der(8);  Y_beta = Aircraft_Data.Lat_Der(2);

% Control
  L_da = Aircraft_Data.Lat_Der(9);   N_da = Aircraft_Data.Lat_Der(10);  Y_da = Aircraft_Data.Lat_Der(11);
  L_dr = Aircraft_Data.Lat_Der(12);  N_dr = Aircraft_Data.Lat_Der(13);  Y_dr = Aircraft_Data.Lat_Der(14);

%% Stability and Control Derivatives matrix
SCDer = [X_u    0  X_w    0    0    0       0       0     0  X_de     0  X_dT
           0  Y_v    0  Y_p    0  Y_r  Y_beta       0  Y_da     0  Y_dr     0
         Z_u    0  Z_w    0  Z_q    0       0  Z_wdot     0  Z_de     0  Z_dT
           0  L_v    0  L_p    0  L_r  L_beta       0  L_da     0  L_dr     0
         M_u    0  M_w    0  M_q    0       0  M_wdot     0  M_de     0  M_dT
           0  N_v    0  N_p    0  N_r  N_beta       0  N_da     0  N_dr     0];

if nargin==1
    varargout{1} = SCDer;
    
elseif nargin==3
    %% Change in States
    CSV = varargin{2};
    
    Delta_u = CSV(1)-Aircraft_Data.ISV(1);
    Delta_v = CSV(2)-Aircraft_Data.ISV(2);
    Delta_w = CSV(3)-Aircraft_Data.ISV(3);
    Delta_p = CSV(4)-Aircraft_Data.ISV(4);
    Delta_q = CSV(5)-Aircraft_Data.ISV(5);
    Delta_r = CSV(6)-Aircraft_Data.ISV(6);

    Delta_beta = asin(CSV(2)/sqrt(CSV(1)^2+CSV(2)^2+CSV(3)^2)) -asin(Aircraft_Data.ISV(2)/Aircraft_Data.V_T0);

    %% Change in State Derivatives
    CSD = varargin{3};
    
    Delta_wdot = CSD(3)-Aircraft_Data.ISD(3);

    %% Control Action
    Delta_da = Aircraft_Data.CA(1);
    Delta_de = Aircraft_Data.CA(2);
    Delta_dr = Aircraft_Data.CA(3);
    Delta_dT = Aircraft_Data.CA(4);

    %% Change in Aerodynamic and Thrust Forces and Moments
    
    Delta_SV_SD_CA = [Delta_u  Delta_v  Delta_w  Delta_p  Delta_q  Delta_r  Delta_beta  Delta_wdot  Delta_da  Delta_de  Delta_dr  Delta_dT]';

    % Moments of inertia
    Ixx = Aircraft_Data.I(1,1);
    Iyy = Aircraft_Data.I(2,2);
    Izz = Aircraft_Data.I(3,3);

    Delta_ATFM_dash = SCDer*Delta_SV_SD_CA;
    Delta_ATF = Aircraft_Data.m*Delta_ATFM_dash(1:3);
    Delta_ATM = [Ixx; Iyy; Izz].*Delta_ATFM_dash(4:6);
    
    varargout{1} = Delta_ATF;
    varargout{2} = Delta_ATM;
    
else
    error('Only one or three inputs are allowed, check the help section for details')
end
end