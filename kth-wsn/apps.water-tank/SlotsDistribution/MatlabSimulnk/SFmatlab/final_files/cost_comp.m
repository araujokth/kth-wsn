function [data, ]cost_comp()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% DEFINE CONSTANTS %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
data.h=3; % sampling period
data.simTIME=80; % N seconds: total sim time
data.sizeWINDOW=1; % steps of data that we implement (DO NOT CHANGE!)
data.sizeHORIZON=2;  % Size for the horizon window (DO NOT CHANGE!)
data.nSensors=20;
data.nTracking=10;
data.nActuators=10; % used to be 5

data.Rw=.5; % process noise variance
data.Wm=1; % mean of the process noise
data.Rv=0.5; % measurement noise variance
data.lossP=1; % select loss probability
data.P=10;   % inital state variance
% SELECT THE INITIAL STATE 
X=zeros(data.nSensors,1)
Xref=10; % set point selection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Xlog=zeros(size(X,1),data.simTIME);
Ylog=zeros(data.nSensors,data.simTIME);
Ulog=zeros(data.nActuators,data.simTIME);
Wlog=zeros(data.nActuators,data.simTIME);
Vlog=zeros(data.nSensors,data.simTIME);
Plog=cell(data.simTIME);
mlog=zeros(size(X,1),data.simTIME);
LAMBDAlog=zeros(data.nSensors,data.simTIME);
Qlog=zeros(data.nSensors,data.simTIME);
ErrorLog = zeros(data.simTIME,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% call for A,B,C,D, E(matrix for noise, continuous), Ed(matrix for noise,
% discrete)
%[A,B,C,D,E,Ed]=matrices_comp(data.nSensors,data.nActuators,data.h);
[A,B,C,D]=matrices_comp_new(data.nSensors,data.nActuators,data.h);

A=A(1:data.nSensors,1:data.nSensors);
B=B(1:data.nSensors,1:data.nActuators);
C=C(1:data.nSensors,1:data.nSensors);
D=D(1:data.nSensors,1:data.nActuators);

% sampling at h
sys=ss(A,B,C,D);
sysd=c2d(sys,data.h);
sys_out.A = sysd.a; % need to fix the error checking in transition_matrices
sys_out.B = sysd.b; % so that this step isn't needed, making the code more 
sys_out.C = sysd.c; % matlab friendly
sys_out.E = sysd.b; 


sys_trans.A = cell(data.sizeHORIZON);
sys_trans.B = cell(data.sizeHORIZON);
sys_trans.C = cell(data.sizeHORIZON);
sys_trans.E = cell(data.sizeHORIZON);    
for j = 1:data.sizeHORIZON
    sys_trans.A{j} = sys_out.A;
    sys_trans.B{j} = sys_out.B;   
    sys_trans.C{j} = sys_out.C;       
    sys_trans.E{j} = sys_out.E;
end
%% Run Algorithm


%======================================================
% DEFINE THE COST FUNCTIONS
% 1) Assume we want the lower tanks to be bounded between 5 and 15 centimeters
% 2) Assume we
% Define cost constraints (assume no quadratic weighting)
num_states = data.nSensors;
cost.num = zeros(data.sizeHORIZON,1);
cost.Fu = eye(data.sizeHORIZON*(data.nActuators)) ...
   - kron(diag(ones(data.sizeHORIZON-1,1),1),-eye(data.nActuators));% ... % ...% diagonal weights
   % + .5*kron(diag(ones(data.sizeHORIZON-1,1),-1),-eye(data.nActuators));% diagonal weights
%cost.Fu = 10*cost.Fu;

%cost.Fy = eye(data.sizeHORIZON*data.nSensors);
cost.Fy = 100*eye(data.nSensors);

%m = ones(size(X))*Xref;
m = X;
P = eye(size(A))*data.P;

init.W = eye(data.nActuators)*data.Rw;
init.Wm = ones(data.nActuators,1)*data.Wm;
init.V = eye(data.nSensors)*data.Rv;
init.Lambda = eye(data.nSensors)*data.lossP;
init.h = data.h;
init.m = cell(data.sizeHORIZON,1);
init.P = cell(data.sizeHORIZON,1);
for i = 1:data.sizeHORIZON
    init.m{i} = m;
    init.P{i} = P;
end
init.M = data.sizeHORIZON;

fprintf('\nSimulation starting ... please wait\n');

%======================================================================
% Above this comment needs to be in the initialization function
% Below needs to be inside the timing loop
% YOU MUST INCLUDE EVERYTHING ... NOTHING CAN BE LEFT OUT.
%======================================================================

k=1;
flag = 1;
while k<=data.simTIME 

    %% get u and r from Jim
    

    % OPTIMAL CONSTRAINED CONTROLLER (sensor selection)
    tic
    cost.num = zeros(data.sizeHORIZON,1);
    cost.Fu = eye(data.sizeHORIZON*(data.nActuators)) ...
        - kron(diag(ones(data.sizeHORIZON-1,1),1),-eye(data.nActuators));
    cost.Fy = 100*eye(data.nSensors);
    for j = 1:data.sizeHORIZON
        for i = 1:1:10 % only the even states up to 20
            tempQx = zeros(1,data.sizeHORIZON*num_states);
            tempQx(1,(j-1)*num_states+2*i) = 1;
            cost.Qx{j,i} = tempQx;
            cost.Fx{j,i} = 1;
            cost.b{j,i} = 10;
            %cost.b{j,i} = 10*(1 - .5*cos(2*pi/40*(k+j-2)));
            cost.c{j,i} = 25;
            cost.alpha{j,i} = .1;
        end
        cost.num(j) = data.nTracking;
    end

    trans = Transition_Matrices(sys_trans);
    [Qsel,U,Error] = LQG_WSN_CDC(trans,cost,init);
    toc
    %cost.Fy = cost.Fy + kron(eye(data.sizeHORIZON),diag(Qsel(:,1)));
    cost.Fy = cost.Fy + diag(Qsel(:,1));
    ErrorLog(k) = Error;

%     %=================================================
%     %OPTIMAL LQG CONTROLLER (selecting all sensors)
%     Qu=eye(data.nSensors+data.nTracking);
%     Ru=0.01*eye(data.nActuators);
%     [K,S,e] = lqrd(A,E,Qu,Ru,data.h) ;
%     U=-K*X; % actuator matrix
%     U = [U ; Xref];
%     Qsel=ones(data.nSensors,data.sizeHORIZON ); % sensor selection matrix
%     %==================================================


    
    %% Algorithm
    % calc Q and H    
    [Q,H,LAMBDA,Ql]=comp_Q_H(Qsel,data);
 
    W=sqrt(data.Rw)*randn(data.nActuators,data.sizeWINDOW) ...
        + data.Wm; % process noise
    V=sqrt(data.Rv)*randn(data.nSensors,data.sizeWINDOW); % measurement noise
    
    % run algorithm for state space calculation
    % we should run this over each N steps, and get new values from Jim in
    % order to implement receding horizon
    for j=1:data.sizeWINDOW
        Y=sysd.C*X+V(:,j);
        
        Xlog(:,k)=X;
        Ylog(:,k)=Y;
        Ulog(:,k)=U(:,j);
        Wlog(:,k)=W(:,j);
        Vlog(:,k)=V(:,j);
        Qlog(:,k)=Ql(:,j);
        Plog{k}=P;
        mlog(:,k)=m;
        LAMBDAlog(:,k)=LAMBDA(:,j);

        % increment system
        X=sysd.A*X+sysd.B*U(:,j)+sysd.B*W(:,j);
        
        % Kalman filter (optimal)
        S = sysd.A*P*sysd.A' + sysd.B*init.W*sysd.B';
        C = H{1,j}*Q{1,j};
        r = C*Ylog(:,k);
        P = S - S*sysd.C'*C'/(C*(sysd.C*S*sysd.C' + init.V)*C')*C*sysd.C*S;
        m = sysd.A*m+sysd.B*U(:,j)+sysd.B*init.Wm;
        m = m + S*sysd.C'*C'/(C*(sysd.C*S*sysd.C' + init.V)*C')*...
            (r - C*sysd.C*m);
        
        % Kalman filter (time delayed) 
        for i = 1:data.sizeHORIZON-1
            init.m{i} = init.m{i+1};
            init.P{i} = init.P{i+1};
        end
        init.m{data.sizeHORIZON} = m;
        init.P{data.sizeHORIZON} = P;
        
        fprintf('Time %d : size(Q,1) = %d, size(H,1) = %d\n', k, ...
        size(Q{1,j},1), size(H{1,j},1));
        k=k+1;
    end    
end

fprintf('\nSimulation complete\n');

%% TODO: 5
% we want to plot the number of sensors vs erros
% energy cost vs alfa

Xlog;
save test

print_MED;

% find the control states
