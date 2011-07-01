function [J, Ju, J1, J2, JRMS] = ...
    getControlCost(x, theta, alpha, beta, u_x, u_theta, t, r_1,r_2)

xa = zeros(4,1); %% sum of all states
u = zeros(2,1); %% sum of control outputs
x1 = zeros(2,1); %% sum of x and theta
x2 = zeros(2,1); %% sum of alhpa and beta

% 
% xa(1) = sum(x);
% xa(2) = sum(theta);
% xa(3) = sum(alpha);
% xa(4) = sum(beta);
% 
% u(1) = sum(u_x);
% u(2) = sum(u_theta);
% 
% x1(1) = sum(x);
% x1(2) = sum(theta);
% 
% x2(1) = sum(alhpa);
% x2(2) = sum(beta);

J_v = zeros(5,1);

%% sum for all the samples
for i=2:length(x)
    %% get the states vectors
    xa = [x(i)-r_1(i) theta(i)-r_2(i) alpha(i) beta(i)];
    %% compute the norm
    xa = norm(xa)^2;
    %% rectangle approximation
    J_v(1) = J_v(1) + xa*(t(i)-t(i-1));
    
    %% RMS
    J_v(5) = J_v(5) + sqrt(xa);
    
    %% get the states vectors
    u = [u_x(i) u_theta(i)];
    %% compute the norm
    u = norm(u)^2;
    %% rectangle approximation
    J_v(2) = J_v(2) + u*(t(i)-t(i-1));
    
    %% get the states vectors
    x1 = [x(i)-r_1(i) theta(i)-r_2(i)];
    %% compute the norm
    x1 = norm(x1)^2;
    %% rectangle approximation
    J_v(3) = J_v(3) + x1*(t(i)-t(i-1));
    
        %% get the states vectors
    x2 = [alpha(i) beta(i)];
    %% compute the norm
    x2 = norm(x2)^2;
    %% rectangle approximation
    J_v(4) = J_v(4) + x2* (t(i)-t(i-1) ) ;
    

end

%% sqrt
J= J_v(1);
Ju = J_v(2);
J1 = J_v(3);
J2 = J_v(4);
JRMS = sqrt(1/(length(x))*J_v(5));
% J = sqrt(J_v(1));
% Ju = sqrt(J_v(2));
% J1 = sqrt(J_v(3));
% J2 = sqrt(J_v(4));


