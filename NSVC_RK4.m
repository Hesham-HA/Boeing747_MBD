function NSV = NSVC_RK4(Aircraft_Data,CSV,ATF,ATM)
% Next State Variables Calculator using 4th order Runge Kutta (NSVC_RK4)
%
% This function outputs the Next State Variables (NSV, state after time
% delta_t), which is a vector comprising 12 variables that fully describe
% the state of a 6-DOF (Degrees Of Freedom) rigid body under certain
% Aerodynamic & Thrust Forces & Moments represented by the vectors ATF and
% ATM respectively, using the Current State Variables (CSV).
% The next state variables is calculated from the current state variables
% using the 4th order Runge-Kutta (RK4) numerical integration method.

k_1 = Aircraft_Data.dt*SDC(Aircraft_Data,CSV,ATF,ATM);
k_2 = Aircraft_Data.dt*SDC(Aircraft_Data,CSV+k_1/2,ATF,ATM);
k_3 = Aircraft_Data.dt*SDC(Aircraft_Data,CSV+k_2/2,ATF,ATM);
k_4 = Aircraft_Data.dt*SDC(Aircraft_Data,CSV+k_3,ATF,ATM);

% NSV = [u;v;w;p;q;r;phi;theta;psi;x;y;z]
NSV = CSV +(k_1 +2*k_2 +2*k_3 +k_4)/6;

% Tweaking phi for periodicity
if NSV(7)<-pi
    NSV(7) = NSV(7)+2*pi;
elseif NSV(7)>pi
    NSV(7) = NSV(7)-2*pi;
end

% Tweaking theta for periodicity
if NSV(8)<-pi/2
    NSV(8) = NSV(8)+pi;
elseif NSV(8)>pi/2
    NSV(8) = NSV(8)-pi;
end

% Tweaking psi for periodicity
if NSV(9)<-pi
    NSV(9) = NSV(9)+2*pi;
elseif NSV(9)>pi
    NSV(9) = NSV(9)-2*pi;
end

end