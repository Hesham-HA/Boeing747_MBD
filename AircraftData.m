function Aircraft_Data = AircraftData(Data_From_Excel)
% This function extracts the aircraft data from the excel file and stores
% it into the struct named 'Aircraft_Data'.

%% Time

     Aircraft_Data.t_i = 0;   % Initial time
     Aircraft_Data.t_f = Data_From_Excel(2);   % Final time
      Aircraft_Data.dt = Data_From_Excel(1);   % Step size
   Aircraft_Data.n_max = (Aircraft_Data.t_f -Aircraft_Data.t_i)/Aircraft_Data.dt +1;   % Number of sampling points in time
Aircraft_Data.time_vec = Aircraft_Data.t_i:Aircraft_Data.dt:Aircraft_Data.t_f;

%% Initial State Variables

 Aircraft_Data.ISV = [Data_From_Excel(4:15); Data_From_Excel(18); Data_From_Excel(20)];
Aircraft_Data.V_T0 = sqrt(Aircraft_Data.ISV(1)^2+Aircraft_Data.ISV(2)^2+Aircraft_Data.ISV(3)^2);

%% Initial State Derivatives

Aircraft_Data.ISD = zeros(12,1);

%% Mass, Gravity and Moments of Inertia

Aircraft_Data.m = Data_From_Excel(51);
Aircraft_Data.g = Data_From_Excel(52);
Aircraft_Data.I = [ Data_From_Excel(53)                    0  -Data_From_Excel(56)
                                      0  Data_From_Excel(54)                     0
                   -Data_From_Excel(56)                    0   Data_From_Excel(55)];

%% Longitudinal Stability and Control Derivatives

Aircraft_Data.Long_Der = Data_From_Excel(21:36);

%% Lateral Stability and Control Derivatives

Aircraft_Data.Lat_Der_raw = Data_From_Excel(37:50);
    Aircraft_Data.Lat_Der = LDC(Aircraft_Data);

%% Initial Gravity Force

Aircraft_Data.IGF = Aircraft_Data.m*Aircraft_Data.g*[sin(Aircraft_Data.ISV(8)); -cos(Aircraft_Data.ISV(8))*sin(Aircraft_Data.ISV(7)); -cos(Aircraft_Data.ISV(8))*cos(Aircraft_Data.ISV(7))];

end