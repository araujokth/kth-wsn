function [q_out,u_out,alpha_out] = LQG_WSN_CDC(x,y,z)
% x is a structure containing transition matrices:
% Tx : transition matrix for x (system state)
% Ty : measurement matrix 
% Tu : transition matrix for u (control inputs)
% Tw : transition matrix for state process noise
% window : the scheduling window size
%
% y is a structure containing cost function data:
% b : cell array of desired set points, rows are the time
% Fx : cell array of quadratic weightings, rows are the same time
% Qx : cell array of selection matrices to specify subsets of the state
%       to constrain. This can be used to eliminate the zero eigenvalues
%       of Fx when constraining only a subset of the state. This is an
%       optional parameter in the structure. If not specified, Qx is
%       assumed to be a cell array of identity matrices.
% Fu : matrix that represents the weighting of the controller cost
% Fy : matrix that represents the sensor selection cost
% c : vector of safety constraints corresponding to the b and F cell arrays
% alpha : cell array of maximum probabilities of errors
% num : cell array of how many contraints are impossed at each time
% Gamma : positive definite matrix weighting the cost of selecting sensors
% -- need to add Gamma to error checking
%
% z is a structure containing
% m : the mean of the initial state
% P : the covariance of the initial state
% Wx : the covariance of the state process noise
% Wu : covariance of the control input noise
% V : the covariance of the measurement noise
% Lambda : the network channel reliability for each sensor
% Wm : mean of the process noise

warning('off')
% dim = LQG_WSN_error_check(x,y,z);
% Nx = dim.Nx;
% M = dim.M;
% Nu = dim.Nu;
% Ny = dim.Ny;
% Nw = dim.Nw;

Nx = 16;
M = z.M;
Nu = 8;
Ny = 16;
Nw = 8;


%====================================================================
% constants for the optimization problem
%m = z.m; % not constant initial condition, varies depending on window
%P = z.P; % not constant initial condition, varies depending on window
V = z.V;
W = z.W;
Ru = sqrtm(y.Fu);
Rq = sqrtm(y.Fy);
Lambda1 = z.Lambda;
Lambda2 = kron(diag(Lambda1)',diag(Lambda1));
Re = zeros(M); % This is only used as a saftey clause.
for i = 1:M
    Re(i,i) = 10e10*(.9)^(i-1);
end
C0_log = cell(M,max(y.num));
C1_log = cell(M,max(y.num));
D0_log = cell(M,max(y.num));
D1_log = cell(M,max(y.num));
%====================================================================

%===============================================================
% Calculate the minimization constants, worst-case (NO INNOVATION)
for i = 1:M-1
    m = z.m{i};
    P = z.P{i};
    Ak = x.Tx{i}; % x transition matrix at time i
    Ek = x.Tw{i}; % w transition matrix at time i
    %Ck = x.Ty(1:Ny*i,1:Nx*i); % y transition matrix at time i (NOT NEEDED)
    
    S = Ak*P*Ak' + Ek*kron(eye(i),W)*Ek'; 
    
    for j = 1:y.num(i)
        % determine constraint parameters
        c = y.c{i,j};
        alpha = y.alpha{i,j};
        g = y.b{i,j};
        F = y.Fx{i,j};

        % Pull off the element selection matrix
        Q = y.Qx{i,j};
        Q = Q(:,1:Nx*i);

        % sensor selection variables           
        C0 = -g + Q*(Ak*m + Ek*kron(ones(i,1),z.Wm));
        C1 = trace(F*Q*S*Q') - c*alpha;
        
        C0_log{i,j} = C0; % control contraint constant
        C1_log{i,j} = C1; % scalar sensor selection constraint
    end
end
%===============================================================

%===============================================================
% Calculate the minimization constants for selection
i = M;
m = z.m{i};
P = z.P{i};
Ak = x.Tx{i}; % x transition matrix at time i
Ek = x.Tw{i}; % w transition matrix at time i

% y transition matrix at time i using only sensors at time i-M 
Ck = x.Ty{i};
Ck = Ck(1:Ny,1:Nx*i);

S = Ak*P*Ak' + Ek*kron(eye(M),W)*Ek';
Si = Ck*S*Ck' + V;
Si_inv = eye(size(Si))/Si;   

% G1 = kron(eye(i),Lambda1);
% G2 = kron(ones(i),Lambda2);
% G2 = (ones(size(G2)) - eye(size(G2))).*G2 + G1;

G1 = Lambda1;
G2 = Lambda2;
G2 = (ones(size(G2)) - eye(size(G2))).*G2 + G1;


for j = 1:y.num(i)
    % determine constraint parameters
    c = y.c{i,j};
    alpha = y.alpha{i,j};
    g = y.b{i,j};
    F = y.Fx{i,j};

    % Pull off the element selection matrix
    Q = y.Qx{i,j};
    Q = Q(:,1:Nx*i);

    % sensor selection variables           
    C0 = -g + Q*(Ak*m + Ek*kron(ones(i,1),z.Wm));
    C1 = trace(F*Q*S*Q') - c*alpha;
    
    D0 = -2*diag(G1*Si_inv*Ck*S*Q'*F*Q*S*Ck');
    D1 = ((Si_inv*Ck*S*Q')*F*(Q*S'*Ck'*Si_inv')).*G2.*Si;

    C0_log{i,j} = C0; % control contraint constant
    C1_log{i,j} = C1; % scalar sensor selection constraint
    D0_log{i,j} = D0; % vector sensor selection constraint
    D1_log{i,j} = D1; % quadratic sensor selection constraint
end
%===============================================================


%===============================================================
% Determine optimal control sequence
cvx_begin
    cvx_quiet(true)
    variables q(Ny) u(Nu*M) e(M) 

    minimize norm(Ru*u) + norm(Rq*q) + sum(Re*e)
    %minimize norm(Rq*q) + 10e10*e
%    minimize norm(Re*e)

    subject to 
        u <= 12;
        u >= 0; % can't have a negative control value

        q <= 1;
        q >= 0;

        e >= 0;
        % the worst case constraints
        for i = 2:M-1
            Bk = x.Tu{i}; % transistion matrix for u

            for j = 1:y.num(i)
                C0 = C0_log{i,j};
                C1 = C1_log{i,j};

                F = y.Fx{i,j};
                Q = y.Qx{i,j};
                Q = Q(:,1:Nx*i);

                % build cvx constraint
                quad_form(Q*Bk*u(1:i*Nu) + C0,F) + C1 <= e(i);
            end
        end
        
        % add in the sensor selection contraint
        i = M;
        Bk = x.Tu{i}; % transistion matrix for u
        for j = 1:y.num(i)
            C0 = C0_log{i,j};
            C1 = C1_log{i,j};
            D0 = D0_log{i,j};
            D1 = D1_log{i,j};

            F = y.Fx{i,j};
            Q = y.Qx{i,j};
            Q = Q(:,1:Nx*i);

            % build cvx constraint
            quad_form(Q*Bk*u(1:i*Nu) + C0,F) ...
                + quad_form(q,D1,D0,0) + C1 <= e(i);
%             quad_form(Q*Bk*u(1:i*Nu) + C0,F) ...
%                 + quad_form(q(1:i*Ny),D1,D0,0) + C1 <= e;
        end
        
cvx_end
%=====================================================================
 

%=====================================================================
%Step 2
[~,qSort] = sort(q,'descend');
qHold = q;
q = zeros(size(q));
k = 0;
i = M;
zHold = qHold;
for j = 1:y.num(i)
    D0 = D0_log{i,j};
    D1 = D1_log{i,j};
    val = (zHold'*D1*zHold + D0'*zHold);
    %z = q(1:i*Ny);
    z = q;
    while (z'*D1*z + D0'*z > val)
        k = k + 1;
        q(qSort(k)) = 1;
        %z = q(1:i*Ny);
        z = q;
    end
end

%=====================================================================


warning('on')

q_out = zeros(Ny,M);
u_out = zeros(Nu,M);
alpha_out = e(2);

q_out(:,1) = q;
for i = 1:M
    u_out(:,i) = u(1+(i-1)*Nu:i*Nu);
end

% for i = 1:M
%     q_out(:,i) = q(1+(i-1)*Ny:i*Ny);
%     u_out(:,i) = u(1+(i-1)*Nu:i*Nu);
% end



