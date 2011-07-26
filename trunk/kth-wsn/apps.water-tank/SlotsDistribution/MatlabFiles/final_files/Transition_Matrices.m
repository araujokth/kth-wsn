function y = Transition_Matrices(x) 
% Calculates the transition matrices for estimation and control
% by : Jim Weimer
%
% x is a structure containing
% A : row concatenation of A matrices
% B : row concatenation of B matrices
% C : row concatenation of C matrices
% E : row concatenation of E matrices
%
% W is the window size to build the transition matrices
%
% flag is a string to specify whether the system is static or dynamic
%   -- static : a single (A,B,C) is given, and are to be be repeated W
%               times
%   -- dynamic : W (A,B,C) are given for the entire sampling window

%=================================================================
% ERROR CHECKING
if (nargin ~= 1)
    error('Tranistion_Matrices requires 3 input arguments')
end

if (~isfield(x,'A'))
    error('System does not contain an A matrix')
elseif (~isfield(x,'B'))
    error('System does not contain an B matrix')
elseif (~isfield(x,'C'))
    error('System does not contain an C matrix')
elseif (~isfield(x,'E'))
    error('System does not contain an E matrix')
end
    
if (~iscell(x.A))
    error('A matrix must be specified as a cell')
elseif (~iscell(x.B))
    error('B matrix must be specified as a cell')
elseif (~iscell(x.C))
    error('C matrix must be specified as a cell')
elseif (~iscell(x.E))
    error('E matrix must be specified as a cell')
end
%===================================================================

%===================================================================
% VARIABLE DEFINITIONS
N = size(x.A{1},1); % number of states
M = size(x.B{1},2); % number of control inputs
J = size(x.C{1},1); % number of sensors
K = size(x.E{1},2); % number of process noise inputs
W = size(x.A,1); % window size
%===================================================================

%====================================================================
% Build transition storage matrices
y.Tx = cell(W,1);  % transition matrix for x
y.Tw = cell(W,1);  % transition matrix for w
y.Tu = cell(W,1);  % transition matrix for u
y.Ty = cell(W,1);  % transition matrix for y
%===================================================================


%===================================================================
% Build Transition Matrices
for i = 1:W
    % Initialize storage matrices
    Tx = zeros(N*i,N);
    Tw = zeros(N*i,K*i);
    Tu = zeros(N*i,M*i);
    Ty = zeros(J*i,N*i);
    
    % Determine transistion matrix for x seperate (different structure)
    Tx(1:N,:) = x.A{1};
    for j = 1:i-1
        Tx(1+j*N:(j+1)*N,:) = x.A{j+1}*Tx(1+(j-1)*N:j*N,:);
    end
    
    % Determine all other transition matrices
    for j = 0:i-1
        Delta = zeros(N*i,N); % a temporary calculation matrix (see paper)
        Delta(1+j*N:(j+1)*N,:) = eye(N);
        for k = j+1:i-1
            Delta(1+k*N:(k+1)*N,:) = x.A{k}*Delta(1+(k-1)*N:k*N,:);
        end
        
        % Transition matrices for w, u, y at time j each
        Tw(:,1+j*K:(j+1)*K) = Delta*x.E{j+1};
        Tu(:,1+j*M:(j+1)*M) = Delta*x.B{j+1};
        Ty(1+j*J:(j+1)*J,1+j*N:(j+1)*N) = x.C{j+1};
    end

    y.Tx{i} = Tx;  % transition matrix for x at time i
    y.Tw{i} = Tw;  % transition matrix for w at time i
    y.Tu{i} = Tu;  % transition matrix for u at time i
    y.Ty{i} = Ty;  % transition matrix for y at time i
end
%===================================================================