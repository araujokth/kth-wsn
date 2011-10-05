function [ output_args ] = plotGlobal( file, initTime, endTime)

LINE_STYLE = {'b-', 'r-', 'c-', 'g-.', 'rx', 'bo', 'mx', 'go'};
COLOR= ['b';'r';'m';'g';'c'];
LINE_WIDTH = [1 3];

% We load the file here and then it will be easy to compare
% different controllers
load(file); % It load the WTSelf
% 
if nargin < 3
    initTime = find(WTSelf.logger.ttStart >= 220, 1, 'first');
    if isempty(initTime) 
        initTime = 1;
    end
    endTime = find(WTSelf.logger.ttStart > 560+100, 1, 'first');
    if isempty(endTime) || initTime == endTime
        endTime = length(WTSelf.logger.ttStart);
    end
end

fprintf('plotGlobal(''%s'', %d, %d); \n', file, initTime, endTime);

nFields = fieldnames(WTSelf.logger);
for i=1:size(nFields,1)
    f = getfield(WTSelf.logger, nFields{i});
    % we have all the vector we want to cut
    if (~strcmp(nFields{i}, 'nSamples') && ...
       ~strcmp(nFields{i}, 'x10') && ...
       ~strcmp(nFields{i}, 'x20') && ...
       ~strcmp(nFields{i}, 'x30') && ...
       ~strcmp(nFields{i}, 'serialOutputFileName'))
        WTSelf.logger = setfield(WTSelf.logger, nFields{i}, f(initTime:endTime,:,:));
    end
end
%% COMM PERFORMANCE
%commPerformance(WTSelf.logger.serialOutputFileName);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Set the physical variables
WTSelf.logger.y = WTSelf.logger.y/WTSelf.WT_CALIBRATION;
WTSelf.logger.y_initial = WTSelf.logger.y_initial/WTSelf.WT_CALIBRATION;

WTSelf.logger.u = WTSelf.logger.u/273;
WTSelf.logger.u_initial = WTSelf.logger.u_initial/273;

lgd = {};

slotsFig = figure;
generalFig = figure;

tanksLevelFig = figure;

figure,
hLine = plot(WTSelf.logger.x10, '+'),
ylabel('x10 = upperTank - finalValue'),
xlabel('Time [s]'),
removePoints(hLine(1,:), WTSelf.logger.ttStart, WTSelf.logger.x10, find(abs(WTSelf.logger.x10) > 0));

figure,
hLine = plot(WTSelf.logger.x20, '+'),
ylabel('x20 = lowerTank - reference'),
xlabel('Time [s]'),
removePoints(hLine(1,:), WTSelf.logger.ttStart, WTSelf.logger.x20, find(abs(WTSelf.logger.x20) > 0));

figure,
hLine = plot(WTSelf.logger.x30, '+'),
ylabel('x30 = integralLowerTanks - finalValue'),
xlabel('Time [s]'),
removePoints(hLine(1,:), WTSelf.logger.ttStart, WTSelf.logger.x30, find(abs(WTSelf.logger.x30) > 0));


for i=1:WTSelf.nWaterTanks
%for i=1:2
    figure(generalFig);
    %% generate the legend for other plots
    lgd{i} = sprintf('Water tank %0.0f', i);
    
    %% Tank levels
    subplot(WTSelf.nWaterTanks,2,i);
    hold on;
    plot(WTSelf.logger.ttStart,  WTSelf.logger.y(:,i, 1),...
        LINE_STYLE{1}, 'LineWidth', LINE_WIDTH(1)),
    plot(WTSelf.logger.ttStart,  WTSelf.logger.y(:,i, 2), ...
        LINE_STYLE{2}, 'LineWidth', LINE_WIDTH(1)),
    plot(WTSelf.logger.ttStart, WTSelf.logger.u(:,i), LINE_STYLE{3}, ...
        'LineWidth', LINE_WIDTH(1)),
    
    % References
    plot([WTSelf.logger.ttStart(1) WTSelf.logger.ttStart(end)], [10 10], ...
        LINE_STYLE{4});
    
    hold off;
    title(sprintf('%s Tank Levels', lgd{i})),
    xlabel('Time [s]'),
    xlim([WTSelf.logger.ttStart(1) WTSelf.logger.ttStart(end)]),
    legend('Upper tank level [cm]', 'Lower tank level [cm]', ...
        'Actuation level [V]', 'Reference [cm]','Location','Best'),
    h=gca;
    %set(h, 'FontSize', 30);
    
    %% Integrals
    subplot(WTSelf.nWaterTanks,2, WTSelf.nWaterTanks+i);
    hold on;
    plot(WTSelf.logger.ttStart, WTSelf.logger.integral(:,i), LINE_STYLE{2}),
    hold off;
    h=gca;
    %set(h, 'FontSize', 30);
    legend('Integrated error','Location','Best');
    title(sprintf('%s Integral', lgd{i})),
    xlabel('Time [s]'),
        xlim([WTSelf.logger.ttStart(1) WTSelf.logger.ttStart(end)]),

    
    %% Plot each tank individually
    figure(tanksLevelFig);
    hold on;
    plot(WTSelf.logger.ttStart,  WTSelf.logger.y(:,i, 1),...
        LINE_STYLE{1}, 'LineWidth', LINE_WIDTH(2)),
    plot(WTSelf.logger.ttStart,  WTSelf.logger.y(:,i, 2), ...
        LINE_STYLE{2}, 'LineWidth', LINE_WIDTH(2)),
    plot(WTSelf.logger.ttStart, WTSelf.logger.u(:,i), LINE_STYLE{3}, 'LineWidth', LINE_WIDTH(2)),
    hold off;
    title(sprintf('%s Tank Levels', lgd{i})),
    xlabel('Time [s]'),
        xlim([WTSelf.logger.ttStart(1) WTSelf.logger.ttStart(end)]),

    legend('Upper tank level [cm]', 'Lower tank level [cm]', 'Actuation level [V]', 'Reference [cm]','Location','Best'),
    h=gca;
    %set(h, 'FontSize', 30);
    
    
    %% Generate the timing for the slots
    % tank1(6) = ttStart;
    % tank1(7) = assignedSlotM1;
   
    % Why Joao change the slot assigned 0 to 9 for WT1 and to 11 for WT2 ???
    
    figure(slotsFig),
    % Plot the desired time slot
    subplot(WTSelf.nWaterTanks, 1, 1);
    h=gca;
    %set(h, 'FontSize', 15);
    idx = find(WTSelf.logger.tau(:,i) == 10000);
    idxPlot = setxor(1:size(WTSelf.logger.tau,1), idx);
    hold on;
    stem(WTSelf.logger.ttStart(idxPlot), WTSelf.logger.tau(idxPlot,i),...
        LINE_STYLE{i+4}, 'LineWidth', LINE_WIDTH(2), 'MarkerSize', 18);
    hold off;
    legend('Tank System 1','Tank System 2','Location','Best');
    xlabel('Time [s]');% Plot success for the dummy sensors' communication
        xlim([WTSelf.logger.ttStart(1) WTSelf.logger.ttStart(end)]),

    ylabel('$\tau_i$ (s)','Interpreter', 'latex');
    
    % Plot the obtained time slot
    subplot(WTSelf.nWaterTanks, 1, 2);
    
    % Find the consecutive slots to set the period
    [idxCol, idxRow, v] = find(WTSelf.logger.slots(:,:)' == 2*i-1);
    nBI = diff(idxRow);
    dSlots = diff(idxCol);
    %dSlots = diff(WTSelf.logger.slots(idx));
    hat_tau = dSlots* WTSelf.slotLength * WTSelf.SYMBOL + nBI*WTSelf.tBI;
        
%       idx = find(WTSelf.logger.slots(:,2*i-1) == WTSelf.IEEE154_aNumSuperframeSlots);
%     nBI = diff(idx);
%     dSlots = diff(WTSelf.logger.slots(idx));
%     hat_tau = dSlots* WTSelf.slotLength * WTSelf.SYMBOL + nBI*WTSelf.tBI;
%     
    h=gca;
    %set(h, 'FontSize', 30);

    hold on;
    stem(WTSelf.logger.ttStart(idxRow(1:end-1)), hat_tau, LINE_STYLE{i+4}, ...
        'LineWidth', LINE_WIDTH(2), 'MarkerSize', 18);
    hold off;
    
    legend('Tank System 1','Tank System 2','Location','Best');
    xlabel('Time [s]');
        xlim([WTSelf.logger.ttStart(1) WTSelf.logger.ttStart(end)]),

    ylabel('$\hat{\tau_i}$ (s)', 'Interpreter', 'latex');
    
    
end

% %% Plot success for the dummy sensors' communication
% figure;
% for i=1:(WTSelf.nWaterTanks+1) % We show all the water tanks and one sensor
%     subplot(WTSelf.nWaterTanks+1,1,i);
%     
%     success = ( WTSelf.logger.slots(:, i) ~= WTSelf.IEEE154_aNumSuperframeSlots );
%     success = [0 ; success(1:end-1)];
%     failure = (-1)*( WTSelf.logger.lateMotes(:,i) == 2*i-1);
%     comm = sum([success' ; failure']);
%     idx = ( comm ~= 0 );
%     stem(WTSelf.logger.ttStart(idx), comm(idx) , LINE_STYLE{i+5});
%     h=gca;
%     %set(h, 'FontSize', 23, 'YTick', [0 1]);
%     if i > WTSelf.nWaterTanks
%         ylabel('Queue Soft Sensors');
%     else
%         ylabel(sprintf('%s Tank Levels', lgd{i})),
%     end
%     xlabel('Time [s]');
% end

clear WTSelf;


% Tools
    function removePoints(hLine, x, y, idx)
        yNew = y(idx);
        xNew = x(idx);
        set(hLine, 'XData', xNew, 'YData', yNew);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

