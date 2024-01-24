function NL = NLAEOM_Model(Aircraft_Data)
% This function outputs the state variables of the Non-linear Airplane
% Equations of Motion (NLAEOM) calculated between the initial and final
% times with the defined step size.

%% Initialization

SV = NaN(Aircraft_Data.n_max,12);   % State Variables at each time step
SD = NaN(Aircraft_Data.n_max,12);   % State Derivatives at each time step

%% Reference Condition

SV(1,:) = Aircraft_Data.ISV(1:12);   % Storing the Initial State Variables in the State Variables matrix
SD(1,:) = Aircraft_Data.ISD;   % Storing the Initial State Derivatives in the State Derivatives matrix

%% Solver

for n = 2:Aircraft_Data.n_max
                     CSV = SV(n-1,:).';   % Current State Variables
                     CSD = SD(n-1,:).';   % Current State Derivatives
   [Delta_ATF,Delta_ATM] = Airframe_Model(Aircraft_Data,CSV,CSD);   % Calculating the change in the Aerodynamic & Thrust Forces & Moments
                     ATF = Delta_ATF +Aircraft_Data.IGF;   % Aerodynamic & Thrust Forces
                     ATM = Delta_ATM;   % Aerodynamic & Thrust Moments
                     NSD = SDC(Aircraft_Data,CSV,ATF,ATM);   % Next State Derivatives
                     NSV = NSVC_RK4(Aircraft_Data,CSV,ATF,ATM).';   % Next State Variables
                 SD(n,:) = NSD;   % Storing the Next State Derivatives in the State Derivatives matrix
                 SV(n,:) = NSV;   % Storing the Next State Variables in the State Variables matrix
end

%% Results

    NL.u = SV(:,1);
    NL.v = SV(:,2);
    NL.w = SV(:,3);
    NL.p = SV(:,4);
    NL.q = SV(:,5);
    NL.r = SV(:,6);
  NL.phi = SV(:,7);
NL.theta = SV(:,8);
  NL.psi = SV(:,9);
   NL.xE = SV(:,10);
   NL.yE = SV(:,11);
   NL.zE = SV(:,12);

  NL.beta = asin(NL.v./sqrt(NL.u.^2+NL.v.^2+NL.w.^2));
 NL.alpha = atan2(NL.w,NL.u);
 
end