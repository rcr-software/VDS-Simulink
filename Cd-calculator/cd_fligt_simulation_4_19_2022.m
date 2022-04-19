%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               RCR Flight Simulator                               %
%                                                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clear all memory and plots
%%clear all; 
close all;
dataStore = zeros(10020,9); % Data storange array
%states of the rocket
 Translation_Accel_True = 0;
 Translation_Vel_True = 0;
 Translation_Pos_True = 0;

% Create IMU and Preasure simmulated sensor parameters
ABS_Orientation_Val=zeros(1,3);%Yaw, Pitch, and Roll
Accel_Sensor_Vals = zeros(1,3);%Accelerometer data read from the Vector Nav
Gyro_Sensor_Vals = zeros(1,3);%Gyroscope data read from the Vector Nav
Mag_Sensor_Vals = zeros(1,3);%Magnetometer data read from the Vector Nav
Gyro_Drift_True = 1.0;
Gyro_Cal_Bias_True = 0.01;
Gyro_Sigma_Noise = .2;
Accel_Sigma_Noise = .1;
Mag_Sigma_Noise = .1;
BMP280_Sigma_Noise = .3;

% Create Time, Thrust, and Variable mass arrays from the AeroTech N2220 
Time = 0.0;     % Initial sim time 
Burn_Time = 6.59; % seconds
Delta_T = 0.004; % Time step, 250Hz
Thrust_Phase_Itterations = round(Burn_Time / Delta_T) ;
Sample_Time_Stamp =    [ 0.01 0.15 0.29 0.43 0.57 0.71 0.85 0.99 1.13 1.27 1.41 1.55 1.69 1.83 1.97 2.11 2.25 2.39 2.53 2.67 2.81 2.95 3.09 3.23 3.37 3.51 3.65 3.79 3.93 4.07 4.21 4.35 4.49 4.63 4.77 4.91 5.05 5.19 5.33 5.47 5.61 5.75 5.89 6.03 6.17 6.31 6.45 Burn_Time ];
Thrust_Curve =         [ 3859.684 3894.684 3928.489 3961.067 3992.385 4022.414 4051.122 4078.482 4104.466 4129.049 4152.204 4173.909 4194.142 4212.882 4230.11 4245.808 4259.96 4272.552 4283.57 4293.004 4300.842 4307.079 4311.706 4314.718 4316.113 4315.889 4314.047 4310.589 4305.517 4298.837 4290.556 4280.684 4269.228 4256.202 4241.619 4225.495 4207.844 4188.686 4168.041 4145.928 4122.371 4097.393 4071.019 4043.276 4014.192 3983.795 3952.115 3919.185  ];
Propelant_Mass_Curve = [14 13.70213 13.40426 13.10639 12.80852 12.51065 12.21278 11.91491 11.61704 11.31917 11.0213 10.72343 10.42556 10.12769 9.82982 9.53195 9.23408 8.93621 8.63834 8.34047 8.0426 7.74473 7.44686 7.14899 6.85112 6.55325 6.25538 5.95751 5.65964 5.36177 5.0639 4.76603 4.46816 4.17029 3.87242 3.57455 3.27668 2.97881 2.68094 2.38307 2.0852 1.78733 1.48946 1.19159 0.89372 0.59585 0.29798 0.00011  ];
Ajusted_Time_Stamp  = linspace(0,5.219,Thrust_Phase_Itterations);
Ajusted_Mass_Curve = interp1(Sample_Time_Stamp,Propelant_Mass_Curve,Ajusted_Time_Stamp,'spline');
Ajusted_Thrust_Curve = interp1(Sample_Time_Stamp,Thrust_Curve,Ajusted_Time_Stamp,'spline');
Coast_Phase_Itterations = 10000;
Total_Itterations = Coast_Phase_Itterations + Thrust_Phase_Itterations;

%Forces
G_Force   = 0;
Thrust    = 0;
Drag      = 0;
Net_Force = 0;
turbulenceNoise = .01;

% Create the vehicles parameters
Total_Vehicle_Mass = 54;%kg
XSectional_Area = 0.0324 ; %Total area of the rocket (m^2)
Density0 = 1.225;
Drag_C = .75 ; 
Target_Apogee = 1000;
 
% __  __       _         _                        %
%|  \/  |     (_)       | |                       %
%| \  / | __ _ _ _ __   | |     ___   ___  _ __   %
%| |\/| |/ _` | | '_ \  | |    / _ \ / _ \| '_ \  %
%| |  | | (_| | | | | | | |___| (_) | (_) | |_) | %
%|_|  |_|\__,_|_|_| |_| |______\___/ \___/| .__/  %
%										 | |      %
%										 |_|      %
for i = 1:Thrust_Phase_Itterations%Engine burn phase  
 %Forces
 G_Force = (Total_Vehicle_Mass + Ajusted_Mass_Curve(1,i)) *9.8* -1;
 Thrust = Ajusted_Thrust_Curve(1,i);
 Drag = -1 * .5 * Drag_C * Density0 * XSectional_Area * Translation_Vel_True * Translation_Vel_True;
 Net_Force =  Thrust + G_Force + Drag;
 %Translational Motion
 Translation_Accel_True = Net_Force / (Total_Vehicle_Mass + Ajusted_Mass_Curve(1,i));
 Translation_Vel_True = Translation_Vel_True + (Translation_Accel_True * Delta_T);
 Translation_Pos_True = Translation_Pos_True + (Translation_Vel_True * Delta_T);


 % Store data
 dataStore(i,:) = [Time Net_Force Drag G_Force Thrust Translation_Accel_True Translation_Vel_True Translation_Pos_True  Density0];
  
 % Reset values for next iteration
 Time = Time + Delta_T;
 Density0 = 1.225*(288.15/(288.15-0.0065*Translation_Pos_True))^ ...
     (1+((9.8*.02896)/(8.3144*-1*0.0065)));
end     

%coasting phase
for i = 1:Coast_Phase_Itterations
 G_Force = Total_Vehicle_Mass *9.8* -1;
 Drag = -1 * .5 * Drag_C * Density0 * XSectional_Area * Translation_Vel_True ^ 2;
 Net_Force =  G_Force + Drag;
    
  %Translational Motion
 Translation_Accel_True = Net_Force / Total_Vehicle_Mass;
 Translation_Vel_True = Translation_Vel_True + (Translation_Accel_True * Delta_T);
 Translation_Pos_True = Translation_Pos_True + (Translation_Vel_True * Delta_T);  
  % Reset values for next iteration
 Time = Time + Delta_T;
 
 dataStore(i+Thrust_Phase_Itterations,:) = [Time Net_Force Drag G_Force 0 Translation_Accel_True Translation_Vel_True  Translation_Pos_True  Density0];

end
    %deadzone

      
                    % Plot results &
set(gcf, 'Position',  [0, 0, 1000, 800]);

%Propelant Mass Curve
subplot(3,5,1);
plot(Sample_Time_Stamp,Propelant_Mass_Curve, 'ro')%data points
hold
plot(Ajusted_Time_Stamp,Ajusted_Mass_Curve,'r-')%interpolated Mass Values
grid on
axis([0 7 0 14])
xlabel('Time (sec)')
ylabel('Mass (Kg)')

%Weight curve
subplot(3,5,2);
plot(dataStore(1:Total_Itterations,1),dataStore(1:Total_Itterations,4),'r-')
hold
grid on
axis([0 30 -700 0])
xlabel('Time (sec)')
ylabel('Weight (N)')

%Thrust curve
subplot(3,5,3);
plot(Sample_Time_Stamp,Thrust_Curve, 'ro')%data points
hold
plot(Ajusted_Time_Stamp,Ajusted_Thrust_Curve,'r-')%interpolated Thrust Values
grid on
axis([0 7 0 6000])
xlabel('Time (sec)')
ylabel('Thrust (N)')

%Drag curve
subplot(3,5,4);
plot(dataStore(1:Total_Itterations,1),dataStore(1:Total_Itterations,3),'r-');
hold
grid on
axis([0 30 -2000 0])
xlabel('Time (sec)')
ylabel('Drag (N)')

%Net Force Curve
subplot(3,5,5);
plot(dataStore(1:Total_Itterations,1),dataStore(1:Total_Itterations,2),'r-');
hold
grid on
axis([0 30 -3000 4000])
xlabel('Time (sec)')
ylabel('Net Force (N)')

%Net Acceleration Curve
subplot(3,5,6);
plot(dataStore(1:Total_Itterations,1),dataStore(1:Total_Itterations,6),'b-');
grid on
axis([0 30 -100 100])
xlabel('Time (sec)')
ylabel('Net Acceleration (m/s^2)')

%Net Velocity Curve
subplot(3,5,7);
plot(dataStore(1:Total_Itterations,1),dataStore(1:Total_Itterations,7),'b-');
grid on
axis([0 30 0 300])
xlabel('Time (sec)')
ylabel('Net Velocity (m/s)')

%Net Position Curve
subplot(3,5,8);
plot(dataStore(1:Total_Itterations,1),dataStore(1:Total_Itterations,8),'b-');
grid on
axis([0 30 0 4000])
xlabel('Time (sec)')
ylabel('Altitude m')

%Air Density Curve
subplot(3,5,11);
plot(dataStore(1:Total_Itterations,1),dataStore(1:Total_Itterations,9),'g-');
grid on
axis([0 30 0 2])
xlabel('Time (sec)')
ylabel('Air Density (kg/m^3)')

pause(.00001);
%FinalEx;
%
%%
%making the experimental velocity and acceleration curve
aReal=dataStore(Thrust_Phase_Itterations:Total_Itterations,6);
vReal=dataStore(Thrust_Phase_Itterations:Total_Itterations,7);
pReal=dataStore(Thrust_Phase_Itterations:Total_Itterations,9);
cReal=cd(aReal,vReal,pReal);

%making the cd vs acelleration curve
aGraph=linspace(0,25,100);
vGraph=linspace(150,300,100);
[AGraph,VGraph] = meshgrid(aGraph,vGraph);
pGraph = zeros(100,1) + 1.1225;
cGraph=cd(AGraph,VGraph,1.1225);

hold on
plot3(aReal,vReal,cReal)
figure(3)
clf;
surf(aGraph,vGraph,cGraph)
xlabel('acceleration (m/s/s)')
ylabel('velocity (m/s)')
zlabel('drag coeficient')
xlim([0 25]) 
ylim([150 300]) 
zlim([0 2])
shading interp

function cdCurve = cd(a,v,p)
    m = 54;%kg
    area = 0.0324 ; %Total area of the rocket (m^2)
    cdCurve=((a+9.81)*2*m)./(area*(v.^2).*p);
end