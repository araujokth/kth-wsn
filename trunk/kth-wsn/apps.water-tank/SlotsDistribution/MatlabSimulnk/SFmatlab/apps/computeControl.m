function [ u ] = computeControl(tankLevel1, tankLevel2)
global WTAPP;


switch WTAPP.controller
    case 1
        %controller = 'null';
        u = zeros(WTAPP.NUMBER_WT, 1);
    case 2
        %controller = 'constant';
        u = ones(WTAPP.NUMBER_WT, 1)*WTAPP.constant;
    case 3
        %controller = 'jose';
        u = joseController();
    case 4
        %controller = 'jim';
        u = jimController();
    otherwise
        u = zeros(WTAPP.NUMBER_WT, 1);
end

end

function u = joseController()
    global WTAPP;
    tankLevel1=WTAPP.tankLevel1;
    tankLevel2=WTAPP.tankLevel2;
        x_state_wt = [];
      for i=1:length(tankLevel1)
          disp('integrating....')
        WTAPP.integrals(i) =  (WTAPP.integrals(i) + ...
            WTAPP.SAMPLING_INTERVAL * (tankLevel2(i) - WTAPP.REF_LEVEL));
 end
 
for i=1:length(tankLevel1)
    x_state_wt = [x_state_wt double(tankLevel1(i)) double(tankLevel2(i))];
end

    x_state = [x_state_wt WTAPP.integrals'];
    x_state = reshape(x_state, length(tankLevel2)*2 + length(WTAPP.integrals),1,1);
    u = -double(WTAPP.K_jose)*double(x_state);
end

function u = jimController()
global WTCONTROL;
global WTAPP;

    tankLevel1=WTAPP.tankLevel1;
    tankLevel2=WTAPP.tankLevel2;
    
cost = WTCONTROL.cost;
data = WTCONTROL.data;
init = WTCONTROL.init;
ErrorLog = WTCONTROL.ErrorLog;
sysd = WTCONTROL.sysd;
Xlog = WTCONTROL.Xlog;
Ylog = WTCONTROL.Ylog;
Ulog = WTCONTROL.Ulog;
Hlog = WTCONTROL.Hlog;
Wlog = WTCONTROL.Wlog;
Vlog = WTCONTROL.Vlog;
Qlog = WTCONTROL.Qlog;
Plog = WTCONTROL.Plog;
P = WTCONTROL.P;
mlog = WTCONTROL.mlog;
m = WTCONTROL.m;
k = WTCONTROL.k;
LAMBDAlog = WTCONTROL.LAMBDAlog;
sys_trans = WTCONTROL.sys_trans;
       
if (k <= data.simTIME)

    % Recall previous sensor selection matrix and control input
    if (k > 1)
        U = Ulog(:,k-1);
        Qsel = Qlog(:,k-1);
        Q = zeros(sum(Qsel),data.nSensors);
        j = 0;
        for i = 1:data.nSensors
            if (Qsel(i) == 1)
                j = j + 1;
                Q(j,i) = 1;
            end
        end
    else
        Q = eye(data.nSensors);
    end
    
    
    % Perform previous step prediction (Kalman Filter)
    if (k > data.calibrate)
        S = sysd.A*P*sysd.A' + sysd.B*init.W*sysd.B';
        m = sysd.A*m + sysd.B*U + sysd.B*init.Wm;
    end
    

    % Sensor Measurements Arrive
    %Y = zeros(length(tankLevel1)
   double(tankLevel1)
    double(tankLevel2)
    Y = [];
    for i=1:length(tankLevel1)
        Y = [Y tankLevel1(i) tankLevel2(i)];
    end
    Y = Y';
    Ylog(:,k)= Y;
    
    % Adjust once calibrated
    if (k > data.calibrate)
        %Y = Y - data.offset
    end
    
    
    
    % Channel loss Model at time k
    Lambda = ones(length(Y),1);
    for i = 1:length(Y)
        if ( Y(i) >= WTAPP.MISSING_VALUE )
            Lambda(i) = 0;
        end
    end
    LAMBDAlog(:,k)=Lambda;   
    %diag(Lambda)*Y
    
    
    % Calculate channel selection matrix
    QTQ = diag(Q'*Q);
    HTH = diag(Q*diag(Lambda)*Q');
    H = zeros(sum(HTH),sum(QTQ));
    j = 0;
    for i = 1:sum(QTQ)
        if (HTH(i) == 1)
            j = j + 1;
            H(j,i) = 1;
        end
    end

    
    
    % Perform Innovation Update (Kalman Filter) 
    if (k > data.calibrate)
        H = H*Q;
        r = H*Y; % received measurements
        P = S - S*sysd.C'*H'/(H*(sysd.C*S*sysd.C' + init.V)*H')*H*sysd.C*S;
        m = m + S*sysd.C'*H'/(H*(sysd.C*S*sysd.C' + init.V)*H')*...
            (r - H*sysd.C*m);
        Plog{k}=P;
        mlog(:,k)=m;
    end

    
    % Kalman filter (time delayed for sensor selection) 
    for i = 1:data.sizeHORIZON-1
        init.m{i} = init.m{i+1};
        init.P{i} = init.P{i+1};
    end
    init.m{data.sizeHORIZON} = m;
    init.P{data.sizeHORIZON} = P;
    

    % Controller performance specifications
    num_states = data.nSensors;
    cost.num = zeros(data.sizeHORIZON,1);
    cost.Fu = eye(data.sizeHORIZON*(data.nActuators)) ...
        - kron(diag(ones(data.sizeHORIZON-1,1),1),-eye(data.nActuators));
    cost.Fy = 100*eye(data.nSensors);
    for j = 1:data.sizeHORIZON
        for i = 1:1:data.nActuators % only the even states up to 20
            tempQx = zeros(1,data.sizeHORIZON*num_states);
            tempQx(1,(j-1)*num_states+2*i) = 1;
            cost.Qx{j,i} = tempQx;
            cost.Fx{j,i} = 1;
            %cost.b{j,i} = 15;
           
            
            if (k < 100)
                cost.b{j,i} = 15;
            else
                cost.b{j,i} = 15 - 2.5*(1 - cos(2*pi/60*(k-150))); % ramp up
            end
            

            cost.c{j,i} = 25;
            cost.alpha{j,i} = .1;
        end
        cost.num(j) = data.nTracking;
    end
    
    % determine the mean offset for scaling the observations
    if (k < data.calibrate)
        data.offset = data.offset + Y;
    elseif (k == data.calibrate)
        data.offset = (data.offset + Y)/data.calibrate;
    end
    
    
    % Calculate Transistion Matrix
    trans = Transition_Matrices(sys_trans);

    % Calculate Control Sequence and Sensor Selection Matrix
    if (k > data.calibrate)
        [Qsel,U,Error] = LQG_WSN_CDC(trans,cost,init);
    else
        Qsel = ones(data.nSensors,data.sizeHORIZON);
        U = zeros(data.nActuators,1);
        Error = 0;
    end
    Ulog(:,k) = U(:,1);
    Qlog(:,k) = Qsel(:,1);
    ErrorLog(k) = Error;
    
    
    % Update Selection cost for future
    cost.Fy = cost.Fy + diag(Qsel(:,1));

    
    % Calculate Q
    [Q]=comp_Q_H(Qsel,data);
    Q = Q{1};
 
    
    % Display Sensor Selection Information
    fprintf('Time %d : size(Q,1) = %d, size(H,1) = %d\n', k, ...
    size(Q,1), size(H,1));
 

    WTCONTROL.cost = cost; 
    WTCONTROL.data = data;
    WTCONTROL.init = init;
    WTCONTROL.ErrorLog = ErrorLog;
    WTCONTROL.sysd = sysd;
    WTCONTROL.Xlog = Xlog;
    WTCONTROL.Ylog = Ylog;
    WTCONTROL.Ulog = Ulog;
    WTCONTROL.Hlog = Hlog;
    WTCONTROL.Wlog = Wlog;
    WTCONTROL.Vlog = Vlog;
    WTCONTROL.Qlog = Qlog;
    WTCONTROL.Plog = Plog;
    WTCONTROL.P = P;
    WTCONTROL.mlog = mlog;
    WTCONTROL.LAMBDAlog = LAMBDAlog;
    WTCONTROL.m = m;

       
    u = U(:,1);
else
    u = zeros(data.nActuators, 1);
end


% Update time
k = k + 1;
WTCONTROL.k = k;

end
