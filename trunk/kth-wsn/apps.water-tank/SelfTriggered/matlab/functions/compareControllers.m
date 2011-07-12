function compareControllers(wtSelf, wtPeriodic)

LINE_STYLE = {'c-', 'b-', 'r-', 'g-.', 'rx', 'bo'};
LINE_WIDTH = [1 3];

% Compare actuation
if wtSelf.nWaterTanks == wtPeriodic.nWaterTanks && ...
    wtSelf.deltaCAP == wtPeriodic.deltaCAP && ...
    wtSelf.slotLength == wtPeriodic.slotLength && ...
    wTSelf.REFERENCE == wtPeriodic.REFERENCE &&
    wTSelf.IEEE154_aNumSuperframeSlots == wtPeriodic.IEEE154_aNumSuperframeSlots && ...

    %% Synch both vectors
    
    % Create all the figures
    actuFig = figure;
    lowerTankFig = figure;
    % for one tank only
    %for i=1:wtSelf.nWaterTanks
    i=1;
    %% Compare the actuator levels
    figure(actuFig),
    hold on,
    plot(wtSelf.logger.ttStart, wtSelf.logger.u(:,1), ...
        'LineWidth', LINE_WIDTH(2)),
    
    plot(wtPeriodic.logger.ttStart, wtPeriodic.logger.u(:,1), ...
        'LineWidth', LINE_WIDTH(2)),
    hold off,
    xlabel('Time [s]'),
    ylabel('Control input u [V]');
    legend('Actuation level (P)','Actuation level (ST)','Location','Best');
    
    h=gca;
    %set(h, 'FontSize', 30);
    
    %% Compare the lower tank levels
    figure(lowerTankFig),
    hold on,
    plot(wtSelf.logger.ttStart, wtSelf.logger.y(:,i, 2), ...
        'LineWidth', LINE_WIDTH(2)),
    
    plot(wtPeriodic.logger.ttStart, wtPeriodic.logger.y(:,i, 2), ...
        'LineWidth', LINE_WIDTH(2)),
    % References
    plot([wtSelf.logger.ttStart(1) wtSelf.logger.ttStart(end)], [wTSelf.REFERENCE wTSelf.REFERENCE], ...
        LINE_STYLE{4});
    
    hold off,
    xlabel('Time [s]'),
    ylabel('Tank water level [cm]');
    legend('Lower tank level (P)', 'Lower tank level (ST)', ...
    'Reference','Location','Best');
    
    h=gca;
    %set(h, 'FontSize', 30);
    

    
    %end
    
else
    error('The files did not have the same number of tanks');
end
end