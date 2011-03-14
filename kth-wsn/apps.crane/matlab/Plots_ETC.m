function [totalPackets, successPackets, lossPackets, reliability,  ...
    J, Ju, J1, J2] = ...
    Plots_ETC(file)

close all

% Initializations
events_interval = []; t = []; alpha =[]; beta = []; theta = [];
u_theta = []; u_x = []; u_z = []; x = []; z = []; ref1 = []; ref2 = [];
timestamp = [];

MAT_PATH = 'mat';
OUTPUTS_PATH = 'outputs';
THRESHOLD = 100;
if nargin < 1
    file = '../Outputs';
end

% Change the file name to 'file'.m
d = dir(file); % get the date of the file
if nargin < 1
m_file = sprintf('%s/data_%s.m',OUTPUTS_PATH , ...
    datestr(d.date, 'yymmdd_HHMMSS'));
copyfile(file, m_file);
run(m_file);
else
    run(file);
end
% run the .m file

disp(datestr(d.date));

%% Save the data in a different .mat
path = sprintf('%s/data_%s', MAT_PATH, datestr(clock, 'yymmdd_HHMMSS'));
save(path);

% %%DEBUG
% figure;plot(diff(t));
% figure; plot(diff(timestamp)*15.259E-6);

%% find the startPoint of the experiment
%% if we have started to save data when we select the ED controller
startPoint = find(events_interval > 100, 1, 'first'); % threshold changed from 500 to 100 , was giving error (Faisal)
if ~isempty(startPoint)
    sp = startPoint;
    events_interval2 = events_interval(startPoint:end);
    startPoint2 = find(events_interval2 < 100, 1, 'first'); % threshold changed from 500 to 100 
    if ~isempty(startPoint2)
        startPoint = startPoint2 + sp;
    end
else
    startPoint = 1;
end

events_interval = events_interval(startPoint:end);
t = t(startPoint:end) - t(startPoint);
alpha = alpha(startPoint:end);
beta = beta(startPoint:end);
theta = theta(startPoint:end);
u_theta = u_theta(startPoint:end);
u_x = u_x(startPoint:end);
u_z = u_z(startPoint:end);
x = x(startPoint:end);
z = z(startPoint:end);
ref1 = ref1(startPoint:end);
ref2 = ref2(startPoint:end);
pckTotal = pckTotal(startPoint:end);
pckSuccess = pckSuccess(startPoint:end);
timestamp = timestamp(startPoint:end);


%%
% Show the plots
%


%% Trolley Response
figure(5)
subplot(3,1,1)
hLine = plot(t,u_x);
xlabel('Time')
ylabel('Trolley Control, u_1')
removedIndex = find(abs(u_x) < THRESHOLD);
yNew = u_x(removedIndex);
xNew = t(removedIndex);
set(hLine, 'XData', xNew, 'YData', yNew);

subplot(3,1,2)
%plot(t,x,'b',t,30*ones(1,length(theta)),'r');
hLine = plot(t,x,'b',t,ref1,'r');
xlabel('Time')
ylabel('Trolley Position,X_w [cm]')
removedIndex = find(abs(ref1) < THRESHOLD);
yNew = ref1(removedIndex);
xNew = t(removedIndex);
set(hLine(2,:), 'XData', xNew, 'YData', yNew);

removedIndex = find(abs(yNew) ~= 0);
yNew = yNew(removedIndex);
xNew = xNew(removedIndex);
set(hLine(2,:), 'XData', xNew, 'YData', yNew);

subplot(3,1,3)
hLine = plot(t,alpha);
xlabel('Time')
ylabel('Payload Angle,\alpha [rad]')
removedIndex = find(abs(alpha) < THRESHOLD);
yNew = alpha(removedIndex);
xNew = t(removedIndex);
set(hLine, 'XData', xNew, 'YData', yNew);

% Arm Response
figure(6)
subplot(3,1,1)
hLine = plot(t,u_theta);
xlabel('Time')
ylabel('Arm Control Signal, u_2')
removedIndex = find(abs(u_theta) < THRESHOLD);
yNew = u_theta(removedIndex);
xNew = t(removedIndex);
set(hLine, 'XData', xNew, 'YData', yNew);

subplot(3,1,2)
%plot(t,theta,'b',t,pi*0.5*ones(1,length(theta)),'r');
hLine = plot(t,theta,'b',t,ref2,'r');
xlabel('Time')
ylabel('Arm Position,\theta[rad]')
removedIndex = find(abs(ref2) < THRESHOLD);
yNew = ref2(removedIndex);
xNew = t(removedIndex);
set(hLine(2,:), 'XData', xNew, 'YData', yNew);

removedIndex = find(abs(yNew) ~= 0);
yNew = yNew(removedIndex);
xNew = xNew(removedIndex);
set(hLine(2,:), 'XData', xNew, 'YData', yNew);

subplot(3,1,3)
hLine = plot(t,beta);
xlabel('Time')
ylabel('Payload Angle,\beta[rad]')
removedIndex = find(abs(beta) < THRESHOLD);
yNew = beta(removedIndex);
xNew = t(removedIndex);
set(hLine, 'XData', xNew, 'YData', yNew);

%% Sample interval
disp(sprintf('Min event interval: %0.2f ms',min(events_interval)));
disp(sprintf('Max event interval: %0.2f ms',max(events_interval)));
% Instants of packets sent
figure(8);
for i=2:length(events_interval)
    if(events_interval(i-1) == events_interval(i))
        events_interval(i-1)= 0;
    else
        events_interval(i-1) = 1;
    end
end
events_interval(end) = 0;

%%
% Synch events with timestamp
figure;
hold on;
idx = find(events_interval == 1);
stem(timestamp(idx), events_interval(idx), '+');
hold off;

%%


%% Control cost
[J, Ju, J1, J2] = getControlCost(x, theta, alpha, beta, u_x, u_theta, t);


disp('----------------------------------------');
end

