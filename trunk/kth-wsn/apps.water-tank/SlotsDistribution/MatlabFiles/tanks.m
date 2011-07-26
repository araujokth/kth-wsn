% Pump 1

clc
%clear all
close all


offset = 1;
ending= 8;
%xi = [0 .5 3 5 7 9 12 15.5 19]
%xi = [.25 1 2 4 7 9 12 16 18]
%xi = [0.5 1 2 3 5.5 7.5 11 14 17];
%xi = [0 0.5 2 3.5 5.5 8.5 12 16 19];
%xi = [0.5 1 2 4 7 9 12 15.5 19]
%xi = [0 0.5 2 3.5 5.5 8 12 14 19];
%xi = [0 0.5 1.5 3 6 8 12 14.5 18];
xi = [0.5 1 2.5 4 7 9 13 16 21];
xi= [10.5 8 7.5 8.5 8.5 9 8.5 10.5]

u = 6;


x = xi(offset:end)
%=====================================================================
% Upper Tank:
    % Km = 4.6    ;       % m^3/s*V   Detta �r det v�rde som anges i manualen.
     a1 = 10000*pi*(0.0047625/2)^2; % m^2        Detta ar det mista av de tre h?len, "Small".
     A1 = 10000*pi*(0.04445/2)^2; % m^2        Detta �r det v�rde som anges i manualen.
     H1 = 0.25;              % m
init_h1 = 0;                 % m


% Lower Tank:
     a2 = 10000*pi*(0.0047625/2)^2; % m^2
     A2 = 10000*pi*(0.04445/2)^2; % m^2        Detta �r det v�rde som anges i manualen.
     H2 = 0.25;              % m
init_h2 = 0;                 % m
% General
     g=980;
     k1=1;
     k2=1;
    
     % Here we define the point of linearization
     L20=10;
     L10=L20*(a2/a1)^2;
     %Km=5*4.6e-6;
%**************************************
% Definition of the system for control design
% more details in
% http://mechanical.poly.edu/faculty/vkapila/ME325/WaterTank/Water_Tank_Manual.pdf

% The states are x=[L1 z1 L2 z2] and u=Vp 
% L1 - level in tank 1
% L2 - level in tank 2
% z1 - integral state of level 1
% z2 - integral state of level 2

o1=0.635;
o2=0.4763;

%beta1=(o1/(o1+o2))*Km/A1; %out 1
alfa=(-a2/A2)*sqrt(g/(2*L20));

beta = (o1/(o1+o2))/A1;
A = beta*u';
b = -alfa*x';

Km = inv(A'*A)*A'*b

Km = (4)*(-alfa*u');


