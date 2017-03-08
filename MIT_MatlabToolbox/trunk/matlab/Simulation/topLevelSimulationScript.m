%% Top-Level Simulation Script
% Load parameters and run scripts needed for the simulation, then run the
% simulation.
% Date created: 3/4/17
% Last updated: 3/4/17

%% Clear and close

clear
close all

%% Run startup

cd /home/student/RollingSpiderEdu/MIT_MatlabToolbox/trunk/matlab/
startup

%% Load values

% Load linearized drone model
load('/home/student/RollingSpiderEdu/MIT_MatlabToolbox/trunk/matlab/Simulation/controllers/controller_fullstate/LQR/linearizeDrone_hover')

%% Run scripts

mdl_quadrotor
parameters_estimationcontrol
LQRControl

%% Run simulation

ts = 30;
sim('sim_quadrotor.slx')

%% Run flight analyzer

FlightAnalyzer