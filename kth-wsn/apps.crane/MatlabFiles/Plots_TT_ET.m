% This functions extract the information from the input files
% @param TT_files - cell with two strings:
%                     TT_Files{1} - labView data for the periodic experiment
%                     TT_Files{2} - serial forwarder data for the periodic
%                                   experiment
%                     ET_Files{1} - labView data for the event-triggered
%                                   experiment
%                     ET_Files{2} - serial forwarder data for the
%                                    event-trigerred experiment
% @return empty
%
% @examples
% i. Plots_TT_ET({'outputs/data_201102102230';'sf_output/tt_201102102230'},
%              {'outputs/data_201102102226'; 'sf_output/et_201102102226'})
%    This runs the experiment without losses    
% ii. Plots_TT_ET( {'outputs/data_201102102334'; 'sf_output/tt_n0_8_201102102334'},
%                  {'outputs/data_201102102341';'sf_output/et_n0_8_201102102341'})
%    This runs the experiment with high losses and the motes with 0
%    retransmissions and 0 CSMAbackoffs
% iii. Plots_TT_ET({'outputs/data_201102142209';'sf_output/tt_n1_10_201102142209'}, 
%                  {'outputs/data_201102141950'; 'sf_output/et_n2_10_201102141950'})
%     This runs the experiment with high losses and the motes with 2
%     retransmissions and 4 CSMA backoffs


function [xET, thetaET, alphaET, betaET, u_xET, u_thetaET, tET, ref1ET, ref2ET, ...
    xTT, thetaTT, alphaTT, betaTT, u_xTT, u_thetaTT, tTT, ref1TT, ref2TT ...
    ] = Plots_TT_ET(TT_files, ET_files)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Load the previous values for the axes
if exist('prev_values.mat', 'file')
    load('prev_values.mat')
else
    %% default limits
     MIN_TROLLEY= 0.5;  
     MAX_TROLLEY= 0;  

     MAX_ARM= 0;
     MIN_ARM=pi;
     
     MIN_PAYLOAD_ALPHA = 0.8;
     MAX_PAYLOAD_ALPHA = -0.8;

     MIN_PAYLOAD_BETA = 0.8;
     MAX_PAYLOAD_BETA = -0.8;
     
     MIN_U_X = 1;
     MAX_U_X = -1;
     
     MIN_U_THETA = 1;
     MAX_U_THETA = -1;
end
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Constants definitions

THRESHOLD = 100;
THRESHOLD_TIMESTAMP = 120;

COST_COMP_LIMIT = 50;
THRESHOLD_INTER_TIME = 5;
START_REF1 = 25;
SAMPLE_INTERVAL = 0.03;

close all;

% Constants for visualitzation
COLOR= ['r';'m';'b';'g';'b'];   % line color->(2) TT ; (3) ET ; (4) errors
LINE_TYPE = {'-';'--';'-.'};    % line types-> (2) TT ; (3) ET ; (4) errors
DOTS_TYPE = ['-';'x';'o'; '+']; % dots types-> (2) TT ; (3) ET ; (4) errors
FONT_SIZE = 30;                 % default font size
INCREASED_SIZE_ZOOM = 6;
MARKET_SIZE = 8;                % marker size
LINE_WIDTH = 2;                 % thickness of the line
TIME_MAX = 80; % limit of the xaxes

%scale factor
SCALE_X_THETA = 1E2;
SCALE_U = 5;

%offset periodic
OFFSET_PERIODIC = 1E-2;

SLIDING_WINDOW=50;
SMOOTH_SPAN = 50;

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Load the data

%% Data for the periodic control
[events_intervalTT, tTT, alphaTT, betaTT, thetaTT, u_thetaTT ...
    u_xTT, u_zTT, xTT, zTT, ref1TT, ref2TT, timestampTT ] = ...
    loadLabViewDataFile(TT_files{1});

[totalPacketsTT, successPacketsTT,  ...
 idxPckTT, idxErrorsSfTT, idxErrorsWirelessTT, packetsTT, timestampTTsf, delayTTsf] ...
    = getInstants(TT_files{2});

%% Data for the event triggered control
[events_intervalET, tET, alphaET, betaET, thetaET, u_thetaET ...
    u_xET, u_zET, xET, zET, ref1ET, ref2ET,timestampET ] = ...
    loadLabViewDataFile(ET_files{1});

[totalPacketsET, successPacketsET,  ...
    idxPckET, idxErrorsSfET, idxErrorsWirelessET, packetsET, timestampETsf, delayETsf] ...
    = getInstants(ET_files{2});
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% synch the startingPoint and final point from labView
% count the number of 25 that both have
count(1) = length(find(ref1TT == START_REF1));
count(2) = length(find(ref1ET == START_REF1));

counter = min(count);

sp(1) = find(ref1TT == START_REF1, 1, 'first');
sp(2) = find(ref1ET == START_REF1, 1, 'first');
for i=1:length(count)
    if (count(i) > counter)
        sp(i) = sp(i) + (count(i) - counter);
    end
end

% sp2(1) = find(ref1TT == START_REF1, 1, 'first');
% sp2(2) = find(ref1ET == START_REF1, 1, 'first');
% 
% sp(1) = max(sp2(1), sp(1));
% sp(2) = max(sp2(2), sp(2));

% get the shortest simulation
simLength = min(length(ref1TT)-sp(1), length(ref1ET)-sp(2));
[events_intervalTT, tTT, alphaTT, betaTT, thetaTT, u_thetaTT ...
    u_xTT, u_zTT, xTT, zTT, ref1TT, ref2TT, timestampTT ] = ...
    syncSimulation (sp(1), simLength, events_intervalTT, tTT, alphaTT, betaTT, ...
    thetaTT, u_thetaTT, u_xTT, u_zTT, xTT, zTT, ref1TT, ref2TT, timestampTT);

[events_intervalET, tET, alphaET, betaET, thetaET, u_thetaET ...
    u_xET, u_zET, xET, zET, ref1ET, ref2ET, timestampET ] = ...
    syncSimulation (sp(2), simLength, events_intervalET, tET, alphaET, betaET, ...
    thetaET, u_thetaET, u_xET, u_zET, xET, zET, ref1ET, ref2ET, timestampET);

%%debug: are they synch?
hold on;
plot(ref1TT, 'r');
plot(ref1ET, 'b');
hold off;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Change units and scale u
% x[cm] -> x[m] 
xET = xET / SCALE_X_THETA;
xTT = xTT / SCALE_X_THETA;
ref1ET = ref1ET / SCALE_X_THETA;
ref1TT = ref1TT / SCALE_X_THETA;
u_xET = u_xET / SCALE_U;
u_xTT = u_xTT / SCALE_U ;
u_thetaET = u_thetaET / SCALE_U;
u_thetaTT = u_thetaTT / SCALE_U;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%synch the vectors from SF and labView

% synch the file from the SF
%figure; hold on; plot(timestampTT,'r'); plot(timestampTTsf,'b'); hold off;
%figure; hold on; plot(timestampET,'r'); plot(timestampETsf,'b'); hold off;

% Uncomment to check if the timing is synch
%figure; hold on; plot(tET,'r'); plot(tTT,'b'); hold off;

% relative timing
tTT = tTT - tTT(1);
tET = tET - tET(1);
timestampTT = timestampTT - timestampTT(1);
timestampET = timestampET - timestampET(1);

% check which is the last timestamp that they are sharing
endTime(1) = min(max(timestampTT), max(timestampET)); % For labView
endTime(2) = min(max(timestampTTsf), max(timestampETsf)); % For SF
endTime = min(endTime);

simLength(1,1) = find(tTT <= endTime,  1, 'last');
simLength(1,2) = find(tET <= endTime, 1, 'last');
simLength(2,1) = find(timestampTTsf <= endTime, 1, 'last');
simLength(2,2) = find(timestampETsf <= endTime, 1, 'last');

% cut again the LabView files
[events_intervalTT, tTT, alphaTT, betaTT, thetaTT, u_thetaTT ...
    u_xTT, u_zTT, xTT, zTT, ref1TT, ref2TT, timestampTT ] = ...
    syncSimulation (1, simLength(1,1), events_intervalTT, tTT, alphaTT, betaTT, ...
    thetaTT, u_thetaTT, u_xTT, u_zTT, xTT, zTT, ref1TT, ref2TT, timestampTT);

[events_intervalET, tET, alphaET, betaET, thetaET, u_thetaET ...
    u_xET, u_zET, xET, zET, ref1ET, ref2ET, timestampET ] = ...
    syncSimulation (1, simLength(1,2), events_intervalET, tET, alphaET, betaET, ...
    thetaET, u_thetaET, u_xET, u_zET, xET, zET, ref1ET, ref2ET, timestampET);

[timestampTTsf, totalPacketsTT, successPacketsTT] = ...
    syncSimulationSf(1, simLength(2,1), timestampTTsf, totalPacketsTT, ...
    successPacketsTT);
[timestampETsf, totalPacketsET, successPacketsET] = ...
    syncSimulationSf(1, simLength(2,2), timestampETsf, totalPacketsET, ...
    successPacketsET);

totalPacketsETsum  = sum(totalPacketsET);
totalPacketsTTsum= sum(totalPacketsTT);
successPacketsETsum = sum(successPacketsET);
successPacketsTTsum = sum(successPacketsTT);

idxPckTT = idxPckTT(find(idxPckTT <= simLength(2,1) ));
idxPckET = idxPckET(find(idxPckET <= simLength(2, 2) ));
idxErrorsSfTT = idxErrorsSfTT(find(idxErrorsSfTT <= simLength(2, 1) ));
idxErrorsSfET = idxErrorsSfET(find(idxErrorsSfET <= simLength(2, 2) ));
idxErrorsWirelessTT = idxErrorsWirelessTT(find(idxErrorsWirelessTT <= simLength(2, 1) ));
idxErrorsWirelessET = idxErrorsWirelessET(find(idxErrorsWirelessET <= simLength(2,2) ));


timestampTTsf = timestampTTsf - timestampTTsf(1);
timestampETsf = timestampETsf - timestampETsf(1);

% xLimitET = [0 tET(end)];
% xLimitTT = [0 tTT(end)];
xLimitET = [0 TIME_MAX];
xLimitTT = xLimitET;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Set the plot limits and the axes Ticks
minExperiment = min(min(xET), min(xTT));
maxExperiment = max(max(xET), max(xTT));
     MIN_TROLLEY= min(minExperiment, MIN_TROLLEY);  
     MAX_TROLLEY= max(maxExperiment, MAX_TROLLEY);
     
         
minExperiment = min(min(u_xET), min(u_xTT));
maxExperiment = max(max(u_xET), max(u_xTT));
     MIN_U_X = min(minExperiment, MIN_U_X);
     MAX_U_X = max(maxExperiment, MAX_U_X);
     
     MIN_TROLLEY_GLOBAL = round(100*min(MIN_TROLLEY, MIN_U_X))/100;
     MAX_TROLLEY_GLOBAL = round(100*max(MAX_TROLLEY, MAX_U_X))/100;

minExperiment = min(min(thetaET), min(thetaTT));
maxExperiment = max(max(thetaET), max(thetaTT));
     MIN_ARM= min(minExperiment, MIN_ARM);
     MAX_ARM= max(maxExperiment, MAX_ARM);
     
minExperiment = min(min(u_thetaET), min(u_thetaTT));
maxExperiment = max(max(u_thetaET), max(u_thetaTT));
     MIN_U_THETA = min(minExperiment, MIN_U_THETA);
     MAX_U_THETA = max(maxExperiment, MAX_U_THETA);
     
    MIN_ARM_GLOBAL = round(100*min(MIN_ARM, MIN_U_THETA))/100;
     MAX_ARM_GLOBAL = round(100*max(MAX_ARM, MAX_U_THETA))/100;
     
minExperiment = min(min(alphaET), min(alphaTT));
maxExperiment = max(max(alphaET), max(alphaTT));
     MIN_PAYLOAD_ALPHA = round(100*min(minExperiment, MIN_PAYLOAD_ALPHA))/100;
     MAX_PAYLOAD_ALPHA = round(100*max(maxExperiment, MAX_PAYLOAD_ALPHA))/100;

minExperiment = min(min(betaET), min(betaTT));
maxExperiment = max(max(betaET), max(betaTT));
     MIN_PAYLOAD_BETA = round(100*min(minExperiment, MIN_PAYLOAD_BETA))/100;
     MAX_PAYLOAD_BETA = round(100*max(maxExperiment, MAX_PAYLOAD_BETA))/100;
     
     MIN_INTER_EVENT_TIME = 0;
     MAX_INTER_EVENT_TIME = 1.5;
     MIN_INTER_EVENT_TIME_ZOOM = 0;
     MAX_INTER_EVENT_TIME_ZOOM = 0.7;
     
     % save the new values
     save('prev_values.mat' , 'MIN_TROLLEY', 'MAX_TROLLEY', 'MIN_U_X',  'MAX_U_X', ...
         'MIN_ARM', 'MAX_ARM', 'MIN_U_THETA', 'MAX_U_THETA', ...
         'MIN_PAYLOAD_ALPHA', 'MAX_PAYLOAD_ALPHA', 'MIN_PAYLOAD_BETA', 'MAX_PAYLOAD_BETA');

STEPS_Y_TROLLEY = 4;
STEPS_Y_TROLLEY = round(100*(abs(MAX_TROLLEY_GLOBAL- MIN_TROLLEY_GLOBAL)/STEPS_Y_TROLLEY))/100;

STEPS_Y_ARM = 4;
STEPS_Y_ARM = round(100*abs(MAX_ARM_GLOBAL- MIN_ARM_GLOBAL)/STEPS_Y_ARM)/100;
STEPS_Y_PAYLOAD_ALPHA = 4;
STEPS_Y_PAYLOAD_ALPHA = round(100*(abs(MAX_PAYLOAD_ALPHA - MIN_PAYLOAD_ALPHA)/STEPS_Y_PAYLOAD_ALPHA))/100;
STEPS_Y_PAYLOAD_BETA = 4;
STEPS_Y_PAYLOAD_BETA = round(100*(abs(MAX_PAYLOAD_BETA - MIN_PAYLOAD_BETA)/STEPS_Y_PAYLOAD_BETA))/100;

STEPS_Y_INTER = 5;
STEPS_Y_INTER = round(100*abs(MAX_INTER_EVENT_TIME- MIN_INTER_EVENT_TIME)/STEPS_Y_INTER)/100;

STEPS_Y_INTER_ZOOM = 5;
STEPS_Y_INTER_ZOOM = round(100*abs(MAX_INTER_EVENT_TIME_ZOOM- MIN_INTER_EVENT_TIME_ZOOM)/STEPS_Y_INTER_ZOOM)/100;

STEPS_X_INTER = 10;
STEPS_X_INTER_ZOOM = 2;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Plots Throlley position

hf = figure;
set(hf, 'Name', 'Trolley Position and Control Law');

hf = subplot(2,1,1);
hLine = plot(tTT,ref1TT,[COLOR(1) LINE_TYPE{1}], ...
             tTT, xTT, [COLOR(2) LINE_TYPE{2}] , ...
             tET, xET, [COLOR(3) LINE_TYPE{3}]);
removePoints(hLine(1,:), tTT, ref1TT, find(abs(ref1TT) < THRESHOLD));
setLineProperties(hLine, LINE_WIDTH);
set(hf, 'FontSize', FONT_SIZE);


%% Plot the control law
hold on;
hLine = plot(tTT,u_xTT,[COLOR(4) LINE_TYPE{2}], ...
             tET,u_xET,[COLOR(5) LINE_TYPE{3}]);
         setLineProperties(hLine, LINE_WIDTH);

% subplot configuration
xlabel('Time (s)','FontSize', FONT_SIZE);
ylabel('x_w [m] and u_1 [V]','FontSize', FONT_SIZE),
legend('r_1', 'x_{wp}', 'x_{we}', ...
    ['u_{1p} /' num2str(SCALE_U)] , ['u_{1e} /' num2str(SCALE_U)],...
    'FontSize', FONT_SIZE, 'Orientation', 'horizontal'),

xlim(xLimitTT),
yLimitTT = [MIN_TROLLEY_GLOBAL MAX_TROLLEY_GLOBAL];
ylim(yLimitTT);
set(gca, 'YTick', [MIN_TROLLEY_GLOBAL:STEPS_Y_TROLLEY:MAX_TROLLEY_GLOBAL]),
set(hf, 'FontSize', FONT_SIZE);

%% Plot the Payload alhpa angle
hf = subplot(2,1,2);
hLine = plot(tTT,alphaTT,[COLOR(2) LINE_TYPE{2}], ...
             tET,alphaET,[COLOR(3) LINE_TYPE{3}]);
setLineProperties(hLine, LINE_WIDTH);
         
% subplot configuration
xlabel('Time (s)','FontSize', FONT_SIZE);
ylabel('Payload angle, \alpha[rad]','FontSize', FONT_SIZE);
legend('\alpha_{p}', '\alpha_{e}','FontSize', FONT_SIZE,...
    'Orientation','horizontal');
xlim(xLimitET),
yLimitTT = [MIN_PAYLOAD_ALPHA MAX_PAYLOAD_ALPHA];
ylim(yLimitTT);
set(gca, 'YTick',[MIN_PAYLOAD_ALPHA:STEPS_Y_PAYLOAD_ALPHA:MAX_PAYLOAD_ALPHA]),
set(hf, 'FontSize', FONT_SIZE);
hold off;
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the Arm position
hf = figure;
set(hf, 'Name', 'Arm Position and Control Law');
hf = subplot(2,1,1);
set(hf, 'FontSize', FONT_SIZE);
hLine = plot(tTT,ref2TT,[COLOR(1) LINE_TYPE{1}], ...
             tTT, thetaTT,[COLOR(2)  LINE_TYPE{2}], ...
             tET,thetaET,[COLOR(3) LINE_TYPE{3}]);
removePoints(hLine(1,:),  tTT, ref2TT, find(abs(ref2TT) < THRESHOLD));
setLineProperties(hLine, LINE_WIDTH);

%% Plot the control law
hold on;
hLine = plot(tTT,u_thetaTT,[COLOR(4) LINE_TYPE{2}], ...
             tET,u_thetaET,[COLOR(5) LINE_TYPE{3}]);
setLineProperties(hLine, LINE_WIDTH);
hold off;

% subplot configuration
xlabel('Time (s)','FontSize', FONT_SIZE);
ylabel('\theta[rad] and u_2 [V]','FontSize', FONT_SIZE);
legend('r_2', '\theta_{p}', '\theta_{e}', ...
        ['u_{2p} /' num2str(SCALE_U)] , ['u_{2e} /' num2str(SCALE_U)],... 
        'FontSize', FONT_SIZE, ...
    'Orientation','horizontal');
xlim(xLimitET),
yLimitTT = [MIN_ARM_GLOBAL MAX_ARM_GLOBAL];
ylim(yLimitTT);
set(gca, 'YTick', [MIN_ARM_GLOBAL:STEPS_Y_ARM:MAX_ARM_GLOBAL]),

set(hf, 'FontSize', FONT_SIZE);

%% Plot the Payload beta angle
hf = subplot(2,1,2);
title( 'Payload \beta angle');
hLine = plot(tTT, betaTT,[COLOR(2) LINE_TYPE{2}], ...
             tET, betaET,[COLOR(3)  LINE_TYPE{3}]);
setLineProperties(hLine, LINE_WIDTH);

xlabel('Time (s)','FontSize', FONT_SIZE),
ylabel('Payload angle, \beta[rad]','FontSize', FONT_SIZE),
legend('\beta_{p}', '\beta_{e}','FontSize', FONT_SIZE, ...
        'FontSize', FONT_SIZE, ...
    'Orientation','horizontal'),
xlim(xLimitET),
yLimitTT = [MIN_PAYLOAD_BETA MAX_PAYLOAD_BETA];
ylim(yLimitTT);
set(gca, 'YTick',[MIN_PAYLOAD_BETA:STEPS_Y_PAYLOAD_BETA:MAX_PAYLOAD_BETA]),

set(hf, 'FontSize', FONT_SIZE);

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plots Inter-event time
% Events for both TT and ET from the Linux SF

hf = figure(4); grid on;
set(hf, 'Name', 'Events for TT and ET');

diffTT =  diff(timestampTTsf);
stampTT = timestampTTsf(2:end);

diffET =  diff(timestampETsf);
stampET = timestampETsf(2:end);


% modify periodic
if mean(diffTT) <= SAMPLE_INTERVAL
    diffTT = diffTT + OFFSET_PERIODIC;
    SAMPLE_INTERVAL = OFFSET_PERIODIC + SAMPLE_INTERVAL;
    successPacketsTT = successPacketsTT * (SAMPLE_INTERVAL - OFFSET_PERIODIC) / SAMPLE_INTERVAL;
    totalPacketsTT = totalPacketsTT * (SAMPLE_INTERVAL - OFFSET_PERIODIC) / SAMPLE_INTERVAL;
end

% delete SF errors
errorSFTT1 = find(abs(diffTT) < THRESHOLD_INTER_TIME); %errors on the SF
errorSFTT2 = find(abs(stampTT) < THRESHOLD_TIMESTAMP); %errors on the SF
positiveDiffTT = find(diffTT > 0);
positiveTimestampTT = find(stampTT > 0);

errorSFET1 = find(abs(diffET) < THRESHOLD_INTER_TIME); %errors on the SF
errorSFET2 = find(abs(stampET) < THRESHOLD_TIMESTAMP); %errors on the SF
positiveDiffET = find(diffET > 0);
positiveTimestampET = find(stampET > 0);

errorSFET = intersect( errorSFET1,errorSFET2);
errorSFET = intersect( errorSFET, positiveDiffET);
errorSFET = intersect( errorSFET, positiveTimestampET);

errorSFTT = intersect( errorSFTT1,errorSFTT2);
errorSFTT = intersect( errorSFTT, positiveDiffTT);
errorSFTT = intersect( errorSFTT, positiveTimestampTT); 

diffTT = diffTT(errorSFTT );
stampTT = stampTT(errorSFTT);
diffET = diffET(errorSFET);
stampET = stampET(errorSFET);

hLine = plot(stampTT, diffTT, [COLOR(2) DOTS_TYPE(2)], ...
          stampET, diffET, [COLOR(3) DOTS_TYPE(3)]);
      setLineProperties(hLine, LINE_WIDTH);
      set(hLine, 'MarkerSize', MARKET_SIZE);

% plot configuration
ylabel('Inter-event time t_{k+1} - t_{k} (s)', 'FontSize', FONT_SIZE, 'Color', 'black');
xlabel('Time (s)',  'FontSize', FONT_SIZE); 
lgd = legend('Periodic', 'Event-triggered');
set(lgd, 'FontSize', FONT_SIZE);
set(gca, 'FontSize', FONT_SIZE);
xlim(xLimitTT);

yLimitTT = [MIN_INTER_EVENT_TIME MAX_INTER_EVENT_TIME];
ylim(yLimitTT);
set(gca, 'YTick',[MIN_INTER_EVENT_TIME:STEPS_Y_INTER:MAX_INTER_EVENT_TIME]),
set(gca, 'XTick', [0:STEPS_X_INTER:ceil(stampET(end))]); 

hold off;

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Reliability

reliabilityET = getReliability(successPacketsET, totalPacketsET);
reliabilityTT = getReliability(successPacketsTT, totalPacketsTT);

hf = figure;
set(hf, 'Name', 'Reliability');

hLine = plot(stampET, reliabilityET(1:length(stampET)), [COLOR(2) LINE_TYPE{2}], ...
             stampTT, reliabilityTT(1:length(stampTT)), [COLOR(3)  LINE_TYPE{3}]);
setLineProperties(hLine, LINE_WIDTH);

xlabel('Time (s)','FontSize', FONT_SIZE),
ylabel('Reliability', 'FontSize', FONT_SIZE),
set(gca, 'FontSize', FONT_SIZE),
lgd = legend('Periodic', 'Event-triggered', ...
    'Orientation','horizontal');
set(lgd, 'FontSize', FONT_SIZE);
xlim(xLimitET),
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plots Inter-event time with zoom in

xLimitZoom = [tTT(find(tTT >= 35, 1, 'first')) tTT(find(tTT >= 46, 1, 'first'))];

% Events for both TT and ET from the Linux SF
hf = figure(5);
set(hf, 'Name', 'Zoom - Events for TT and ET');

hLine = plot(stampTT, diffTT, [COLOR(2) DOTS_TYPE(2)], ...
          stampET, diffET, [COLOR(3) DOTS_TYPE(3)]);
      setLineProperties(hLine, LINE_WIDTH);

ylabel('Inter-event time t_{k+1} - t_{k} (s)', 'FontSize', FONT_SIZE, 'Color', 'black');
xlabel('Time (s)',  'FontSize', FONT_SIZE+6); 
set(gca, 'FontSize', FONT_SIZE+INCREASED_SIZE_ZOOM);
set(hLine, 'MarkerSize', MARKET_SIZE);

yLimitTT = [MIN_INTER_EVENT_TIME_ZOOM MAX_INTER_EVENT_TIME_ZOOM];
ylim(yLimitTT);
set(gca, 'YTick',[MIN_INTER_EVENT_TIME_ZOOM:STEPS_Y_INTER_ZOOM:MAX_INTER_EVENT_TIME_ZOOM]),

xlim(xLimitZoom),
set(gca, 'XTick',[ceil(min(xLimitZoom)):STEPS_X_INTER_ZOOM:ceil(stampET(end))]); 
hold off;

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% To debug, and check the time limits
% disp('...........');
% disp(sprintf('ET t(end): %0.2f', tET(end)));
% disp(sprintf('TT t(end): %0.2f', tTT(end)));
% disp(sprintf('SF TT t(end): %0.2f', stampTT(end)));
% disp(sprintf('SF ET t(end): %0.2f', stampET(end)));
% disp('...........');
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control cost computation
costEndET = find(tET >= COST_COMP_LIMIT, 1, 'first');

[events_intervalET, tET, alphaET, betaET, thetaET, u_thetaET ...
    u_xET, u_zET, xET, zET, ref1ET, ref2ET, timestampET ] = ...
    syncSimulation (1, costEndET, events_intervalET, tET, alphaET, betaET, ...
    thetaET, u_thetaET, u_xET, u_zET, xET, zET, ref1ET, ref2ET, timestampET);

costEndTT = find(tTT >= COST_COMP_LIMIT, 1, 'first');
[events_intervalTT, tTT, alphaTT, betaTT, thetaTT, u_thetaTT ...
    u_xTT, u_zTT, xTT, zTT, ref1TT, ref2TT, timestampTT ] = ...
    syncSimulation (1, costEndTT, events_intervalTT, tTT, alphaTT, betaTT, ...
    thetaTT, u_thetaTT, u_xTT, u_zTT, xTT, zTT, ref1TT, ref2TT, timestampTT);

[JET, JuET, J1ET, J2ET, JRMSET] = ...
    getControlCost(xET, thetaET, alphaET, betaET, u_xET, u_thetaET, tET, ref1ET, ref2ET);

[JTT, JuTT, J1TT, J2TT, JRMSTT] = ...
    getControlCost(xTT, thetaTT, alphaTT, betaTT, u_xTT, u_thetaTT, tTT, ref1TT, ref2TT);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Show results
disp('Periodic control');
disp(sprintf('\tTotal Packets: %0.0f', totalPacketsTTsum ));
disp(sprintf('\tSuccess Packets: %0.0f', successPacketsTTsum ));
disp(sprintf('\tLoss Packets: %0.0f', totalPacketsTTsum - successPacketsTTsum ));
disp(sprintf('\tReliability: %0.2f %%', successPacketsTTsum/totalPacketsTTsum*100 ));
disp(sprintf('\tMean Delay \\Delta_t = %0.2f ms', mean(delayTTsf)*1E3));
disp(sprintf('\tControl cost all states: %0.3f', JTT));
disp(sprintf('\tControl cost u: %0.3f', JuTT));
disp(sprintf('\tControl cost [x, theta]: %0.3f', J1TT));
disp(sprintf('\tControl cost [alpha beta]: %0.3f', J2TT));
disp(sprintf('\tControl cost RMS: %0.3f', JRMSTT));

disp('...........');

disp('Event triggered control');
disp(sprintf('\tTotal Packets: %0.0f', totalPacketsETsum ));
disp(sprintf('\tSuccess Packets: %0.0f', successPacketsETsum ));
disp(sprintf('\tLoss Packets: %0.0f', totalPacketsETsum - successPacketsETsum ));
disp(sprintf('\tReliability: %0.2f %%', successPacketsETsum/totalPacketsETsum*100 ));
disp(sprintf('\tMean Delay \\Delta_t = %0.2f ms', mean(delayETsf)*1E3));
disp(sprintf('\tControl cost all states: %0.3f', JET));
disp(sprintf('\tControl cost u: %0.3f', JuET));
disp(sprintf('\tControl cost [x, theta]: %0.3f', J1ET));
disp(sprintf('\tControl cost [alpha beta]: %0.3f', J2ET));
disp(sprintf('\tControl cost RMS: %0.3f', JRMSET));

disp('...........');

disp('Event triggered control vs Periodic');
disp(sprintf('\tRate packets: %0.2f %%', 100*(totalPacketsTTsum - totalPacketsETsum)/totalPacketsTTsum ));
disp(sprintf('\tRate packets: %0.2f', totalPacketsTTsum/totalPacketsETsum));

disp(sprintf('\tRate cost all states: %0.3f', JTT/JET ));
disp(sprintf('\tRate cost RMS: %0.3f', JRMSTT/JRMSET ));

disp('...........');

disp('----------------------------------------');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tools
    function removePoints(hLine, x, y, idx)
        yNew = y(idx);
        xNew = x(idx);
        set(hLine, 'XData', xNew, 'YData', yNew);
    end

    function setLineProperties(line, width, style)
        %set(line, 'LineStyle', style);
        set(line, 'LineWidth', width);
    end

    function [reliability] = getReliability(packetsSuccess, packetsTotal)
        reliability = zeros(length(packetsSuccess), 1);
        for i=1:length(packetsSuccess)
           last = min(i+SLIDING_WINDOW, length(packetsSuccess));
           reliability(i) = sum(packetsSuccess(i:last))/sum(packetsTotal(i:last));
        end
        reliability = smooth(smooth(reliability, SMOOTH_SPAN));
        
    end
    function [ao, bo, co] ...
            = syncSimulationSf(sp, endP, a, b, c)
        
        sp=1;
        ao = a(sp:(endP));
        bo = b(sp:(endP));
        co = c(sp:(endP));
        
    end

    function [ao, bo, co, do, eo, fo, go, ho, io, jo, ko, lo, mo] ...
            = syncSimulation(sp, endP, a, b, c, d, e, f, g, h, i, j, k, l, m)
        if sp ==1
            endP = endP - sp;
        end
        ao = a(sp:(sp+endP));
        bo = b(sp:(sp+endP));
        co = c(sp:(sp+endP));
        do = d(sp:(sp+endP));
        eo = e(sp:(sp+endP));
        fo = f(sp:(sp+endP));
        go = g(sp:(sp+endP));
        ho = h(sp:(sp+endP));
        io = i(sp:(sp+endP));
        jo = j(sp:(sp+endP));
        ko = k(sp:(sp+endP));
        lo = l(sp:(sp+endP));
        mo = m(sp:(sp+endP));
        
    end


end