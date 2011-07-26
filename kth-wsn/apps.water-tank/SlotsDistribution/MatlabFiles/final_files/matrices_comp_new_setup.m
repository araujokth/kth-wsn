                                                                     
function [A_new,B_new,C_new,D_new]=matrices_comp_new_setup(nA,nB,h)
                            
                                                                     
                                             
%clc
%clear all
%function [A_new,B_new,C_new,D_new,E,Ed]=matrices_comp(nA,nB,h)
%**************************************

% Calibration
%Km_cal = [3.81 3.73 3.30 3.71 3.77 3.55 3.52 4.00];
%Km_cal = [3.3427 3.2735 2.7972 3.0570 3.2735 3.0050 3.0137 3.3947]; % linearization between 4..7
%Km_cal = [3.81 3.4 3 3.4 3.2 3.55 3.52 4.00];
%  Km_cal = [
%     2.9
%     2.5
%     2.7279
%     3.0916
%     3.0916
%     3.2735
%     3.0916
%     3.8190
%     ];
% Km_cal = [
%     3
%     2.2
%     2.4
%     2.8575
%     2.6
%     3.0195
%     2.5
%     3.8
%     ];
Km_cal = ones(8,1)*4.6;


% Upper Tank:
     Km = 4.6    ;       % m^3/s*V   Detta är det värde som anges i manualen.
     a1 = 10000*pi*(0.0047625/2)^2; % m^2        Detta ar det mista av de tre h?len, "Small".
     A1 = 10000*pi*(0.04445/2)^2; % m^2        Detta är det värde som anges i manualen.
     H1 = 0.25;              % m
init_h1 = 0;                 % m


% Lower Tank:
     a2 = 10000*pi*(0.0047625/2)^2; % m^2
     A2 = 10000*pi*(0.04445/2)^2; % m^2        Detta är det värde som anges i manualen.
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
% http://mechani%beta1=(o1/(o1+o2))*Km/A1; % out 1
%beta3=(o2/(o1+o2))*Km/A1; % out 2cal.poly.edu/faculty/vkapila/ME325/WaterTank/Water_Tank_Manual.pdf

% The states are x=[L1 z1 L2 z2] and u=Vp 
% L1 - level in tank 1
% L2 - level in tank 2
% z1 - integral state of level 1
% z2 - integral state of level 2

o1=0.635;
o2=0.4763;
alfa1=-1*(a1/A1)*sqrt(g/(2*L10));

beta1=(o1/(o1+o2))/A1; % out 1
beta3=(o2/(o1+o2))/A1; % out 2

%beta1=(o1/(o1+o2))*Km/A1; % out 1
%beta3=(o2/(o1+o2))*Km/A1; % out 2
alfa2=(-a2/A2)*sqrt(g/(2*L20));
beta2=(a1/A2)*sqrt(g/(2*L10));

% _a is for the first coupled tank system
A_block_a=[alfa1 0; beta2 alfa2];
B_block_a=[beta1]; % to itself
B_block_b=[beta3]; % to the next one

% full A,B,C

%% Without integral control
% select size: 10 processes = 20 states and 10 inputs
nA=16;
nB=8;
A=zeros(nA,nA);
B=zeros(nA,nB);
C=eye(nA);

k=1;
i=1;
while i<nA
    A(i:i+1,k:k+1)=A_block_a;
    k=k+2;
    i=i+2;
end

k=1;
while k<nB
    B(2*k+1,k)=beta3*Km_cal(k); % pump k feeds upper tank k+1 with small hole
    B(2*k+2,k+1)=beta1*Km_cal(k+1); % pump k+1 feeds lower tank k+1 with big hole
    k=k+1;
end
B(1,nB)=beta3*Km_cal(nB); % last pump loops to first top tank with small hole
B(2,1)=beta1*Km_cal(1); % first pump feeds first lower tank with big hole
B;
% k=1;
% i=1;
% j=1;
% while j<=nB
%     B(i,k)=B_block_b;
%     i=i+2;
%     B(i,k)=B_block_a;
%     k=k+1;
%     i=i+2;
%     j=j+1;
% end
% 
% B
% D=zeros(nA,nB);
% 
% 
% %% With integral control
[a,b]=size(B);
k=2; % we need to do integral control of 
% x2,x4,x6 etc etc
nAnew=nA+8; % 10 since we want to track 10 lower tanks out of the 20 tanks
A_new=zeros(nAnew,nAnew);
A_new(1:nA,1:nA)=A;
for i=1:nB
    A_new(nA+i,k)=1;
    k=k+2;
end
% 
% 
% 
B_new=zeros(nAnew,b); % first column for original B and second column for the integrator!
B_new(1:nA,1:b)=B;

%B_new(nA:nAnew,b)=-1*ones(nAnew-nA,1);

C_new=[eye(nA) zeros(nA,nAnew-nA)];

D_new=zeros(nA,nB);
% 
% E=[B;zeros(nAnew-nA,nB)];
% sys=ss(A_new,E,C_new,zeros(nA,nB));
% sysd=c2d(sys,h);
% Ed=zeros(nAnew,nB); % disturbance has the same size of the inputs u (nB)
% 
% k=1;
% i=3;
% j=1;
% while j<=nB
%     Ed(i,k)=B_block_a;
%     i=i+2;
%     Ed(i,k)=B_block_a;
%     k=k+1;
%     i=i+2;
%     j=j+1;
% end
% 
% A_new(2,nA-1)=beta2;
% end
Ai=A_new;
Bi=B_new;
Ci=C_new;
Di=D_new;