%% Top-Level Simulation Script
% Load parameters and run scripts needed for the simulation, then run the
% simulation.
% Date created: 3/4/17
% Last updated: 3/4/17

%% Clear and close
clear
close all

%% Run startup
cd ~/RollingSpiderEdu/MIT_MatlabToolbox/trunk/matlab/
startup

%% Load values
% Load linearized drone model
load('controllers/controller_fullstate/LQR/linearizeDrone_hover')
% load('RSdata_224_30')
load('testFlight_0.6AltRef.mat')

%% Run scripts
mdl_quadrotor
parameters_estimationcontrol
LQRControl

%% Run full simulation
ts = 30;
% sim('sim_quadrotor.slx')

%% Run flight analyzer
FlightAnalyzer
close all

%% Run compensator simulation
sim('sim_SoftwareIntheLoop_Compensator_R2015b.slx')

%% Plot results
plotAltitudeWithSim
cd ./Lab4
figName = ['problem7_Q' num2str(estimParams.alt.kf.Q) '_R' num2str(estimParams.alt.kf.R) '.fig'];
savefig(figName)
figName = ['problem7_Q' num2str(estimParams.alt.kf.Q) '_R' num2str(estimParams.alt.kf.R) '.png'];
print(figName,'-dpng')