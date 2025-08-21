clear all;
close all;
clc;

%%

% load robot URDF
robot = importrobot('frankaEmikaPanda.urdf');

% PD params

KP = [25,70,10,10,10,10,10];

KD = [15,50,10,10,10,10,10];

% trajectory gen

syms t real;

freq = 2; 
A = 1/2; 

traj = A * sin(t * freq);

traj = matlabFunction([traj;diff(traj);diff(diff(traj))]);

