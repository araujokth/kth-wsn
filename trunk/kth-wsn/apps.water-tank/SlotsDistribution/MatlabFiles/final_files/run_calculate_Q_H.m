% This function tests the controller design for sensor selection and
% control stabilization
% written by : Jim Weimer
clc
close all
clear all

%========================================
% User defined inputs
N = 10; % Deminsion of system
m = 0; % initial state mean
sigma_x = 2; % initial state covariance
yd = 0; % desired set point
y_max = 10; % error bounds
alpha = .1; % probability of error
W = 5; % scheduling window
sigma_w = .01;  % process noise
sigma_v = .1;   % measurement noise
lambda = .9; % channel reliability
fy = 100; % sensor selection weighting
fu = 1; % control input weighting
%=========================================

% Define a random system
A = rand(N);
sysd.A = A'*A;
sysd.B = rand(N);
sysd.C = rand(N);
sysd.D = zeros(N,1);
sysd.E = eye(N);

% Define cost constraints (assume no quadratic weighting)
cost.num = zeros(W,1);
for j = 1:W
    for i = 1:N
        tempQx = zeros(1,j*N);
        tempQx(1,(j-1)*N+i) = 1;
        tempb = 0;
        cost.Qx{j,i} = tempQx;
        cost.Fx{j,i} = .5;
        cost.b{j,i} = tempb;
        cost.c{j,i} = y_max;
        cost.alpha{j,i} = alpha;
    end
    cost.num(j) = N;
end
cost.Fu = eye(N*W)*fu;
cost.Fy = eye(N*W)*fy;

% Define the initial conditions
init.m = ones(N,1)*m;
init.P = diag(ones(N,1)*sigma_x);
init.W = diag(ones(N,1)*sigma_w);
init.V = diag(ones(N,1)*sigma_v);
init.Lambda = diag(ones(N,1)*lambda);

tic
trans = Transition_Matrices(sysd,W,'static');
toc
% determine control and sensor schedules

[q,u] = LQG_WSN(trans,cost,init);
toc