function Lat_Der = LDC(Aircraft_Data)
% Help
% Lateral Derivatives Calculator (LDC)
% This function calculates the undashed and unstarred, stability and
% control, lateral derivatives using the relations provided in NASA CR-2144
% Appendix A.

%% Moments of Inertia
Ixx = Aircraft_Data.I(1,1);
Izz = Aircraft_Data.I(3,3);
Ixz = -Aircraft_Data.I(1,3);

%% Lateral Stability Derivatives
% Dashed
   LN_p_dash = Aircraft_Data.Lat_Der_raw(5:6);
   LN_r_dash = Aircraft_Data.Lat_Der_raw(7:8);
LN_Beta_dash = Aircraft_Data.Lat_Der_raw(3:4);

% Undashed
      G = 1/(1-Ixz^2/Ixx/Izz);
      A = [      1   Ixz/Ixx
           Ixz/Izz         1];

   LN_p = A\(LN_p_dash/G); %  (in 1/sec)
   LN_r = A\(LN_r_dash/G); %  (in 1/sec)
LN_Beta = A\(LN_Beta_dash/G); %  (in 1/sec^2)

%% Lateral Control Derivatives
% Dashed and starred
LN_da_dash = Aircraft_Data.Lat_Der_raw(11:12);
LN_dr_dash = Aircraft_Data.Lat_Der_raw(13:14);
 Y_da_star = Aircraft_Data.Lat_Der_raw(9);
 Y_dr_star = Aircraft_Data.Lat_Der_raw(10);

% Undashed and unstarred
LN_da = A\(LN_da_dash/G); %  (in 1/sec^2)
LN_dr = A\(LN_dr_dash/G); %  (in 1/sec^2)
 Y_da = Y_da_star*Aircraft_Data.V_T0; % (in ft/sec^2)
 Y_dr = Y_dr_star*Aircraft_Data.V_T0; % (in ft/sec^2)

%% Lateral Derivatives
 SD_Lat = [LN_p; LN_r; LN_Beta];
 CD_Lat = [LN_da; Y_da; LN_dr; Y_dr];
Lat_Der = [Aircraft_Data.Lat_Der_raw(1:2); SD_Lat; CD_Lat];

end
