%% Top-Level Simulation Script
% Load parameters and run scripts needed for the simulation, then run the
% simulation.
% Date created: 3/4/17
% Last updated: 3/19/17

%% Clear and close
clear
close all

%% Load test data
% load('RSdata__2017_03_21__10_04_25.mat')
load('RSdata__2017_03_21__10_31_38.mat')

%% Run startup
cd ~/RollingSpiderEdu/MIT_MatlabToolbox/trunk/matlab/
startup

%% Load values
% Load linearized drone model
load('controllers/controller_fullstate/LQR/linearizeDrone_hover')
% load('RSdata_224_30')
% load('RSdata__upAndDown.mat')

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
figName = ['problem8_tested_Q' num2str(estimParams.alt.kf.Q) '_Rs' num2str(estimParams.alt.kf.R(1,1)) '_Rp' num2str(estimParams.alt.kf.R(2,2)) '.fig'];
% figName = ['problem7_tested_Q' num2str(estimParams.alt.kf.Q) '_R' num2str(estimParams.alt.kf.R) '.fig'];
savefig(figName)
figName = ['problem8_tested_Q' num2str(estimParams.alt.kf.Q) '_Rs' num2str(estimParams.alt.kf.R(1,1)) '_Rp' num2str(estimParams.alt.kf.R(2,2)) '.png'];
% figName = ['problem7_tested_Q' num2str(estimParams.alt.kf.Q) '_R' num2str(estimParams.alt.kf.R) '.png'];
print(figName,'-dpng')