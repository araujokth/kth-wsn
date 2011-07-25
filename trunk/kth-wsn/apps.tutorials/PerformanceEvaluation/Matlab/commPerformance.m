function [ output_args ] = commPerformance( fileToRead )

% For this function we need to run the seriallisten that we have modified
% seriallisten /dev/ttyUSB0 115200 > file

SMOOTH_SPAN = 10;
SLIDING_WINDOW = 10;
T_SYMBOLS = 1/(2*32768);
FONT_SIZE = 14,

%% Structure def
commData = struct; % struct that contains all the links
commDataByLinks = cell(1,1);
commDataByWaterTank = cell(1,1);
commHeader = {};
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

data = importdata(fileToRead);
%% there are two fields inside data {data textdata}
commHeader = data.textdata;
% we assume that every node only sends to one node
% Create new variables in the base workspace from those fields.

for i = 1:length(commHeader) 
    commData.( commHeader{i} ) = data.data(1:(end-2),i);
end

%% Modify constants
commData.timestamp = commData.timestamp - commData.timestamp(1);
commData.timestamp = commData.timestamp * T_SYMBOLS;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Sort data by links
diffSrcId = unique(commData.srcId);
nLinks = length(diffSrcId);
commDataByLinks = cell(nLinks, 1);
for i=1:nLinks
    idx = commData.srcId == diffSrcId(i);
    wtStruct = struct;
    for j=1:length(commHeader)
        f =  commData.( commHeader{j} );
        wtStruct.( commHeader{j} ) = f(idx);
    end
    commDataByLinks{i} = wtStruct;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 
fgInterEvent = figure;

fgRssi = figure;
fgLQI = figure;
fgDelay = figure;
fgReliability = figure;

colourOrder = get(gca,'ColorOrder');

for i=1:nLinks
        %% Inter-event time
        figure(fgInterEvent);hold on;
        diffET =  diff(commDataByLinks{i,1}.timestamp);
        stampET = commDataByLinks{i,1}.timestamp(2:end);
        plot(stampET, diffET, '+', 'Color', colourOrder(i,:)),
        hold off;
        
        %% RSSI
        figure(fgRssi);hold on;
        printRssi = smooth(smooth(commDataByLinks{i,1}.rssi, SMOOTH_SPAN));
        plot(commDataByLinks{i,1}.timestamp, printRssi, 'Color', colourOrder(i,:));
        hold off;
        
        %% LQI
        figure(fgLQI);hold on;
        printLQI = smooth(smooth(commDataByLinks{i,1}.lqi, SMOOTH_SPAN));
        plot(commDataByLinks{i,1}.timestamp, printLQI, 'Color', colourOrder(i,:));
        hold off;    
        
        %% Reliability
        figure(fgReliability);hold on;
        printReliability = getReliability(commDataByLinks{i,1}.pckSuccess, commDataByLinks{i,1}.pckTotal);
        plot(commDataByLinks{i,1}.timestamp, printReliability, 'Color', colourOrder(i,:));
        hold off;    
        
        %% Delay
        figure(fgDelay);hold on;
        printDelay = 1000*smooth(smooth(diff(commDataByLinks{i,1}.timestamp), SMOOTH_SPAN));
        plot(commDataByLinks{i,1}.timestamp(2:end), printDelay, 'Color', colourOrder(i,:));
        hold off;    
end
%%%% print the labels and the legends
figure(fgInterEvent);
ylabel('Inter-event time t_{k+1} - t_{k} (s)', 'FontSize', FONT_SIZE, 'Color', 'black');
xlabel('Time (s)',  'FontSize', FONT_SIZE),
legend('Link 1', 'Link 2','FontSize', FONT_SIZE);


figure(fgRssi);
ylabel('RSSI [dBm]', 'FontSize', FONT_SIZE, 'Color', 'black');
xlabel('Time [s]',  'FontSize', FONT_SIZE),
legend('Link 1', 'Link 2','FontSize', FONT_SIZE);

figure(fgLQI);
ylabel('LQI = (0, 255)', 'FontSize', FONT_SIZE, 'Color', 'black');
xlabel('Time [s]',  'FontSize', FONT_SIZE),
legend('Link 1', 'Link 2','FontSize', FONT_SIZE);

figure(fgReliability);
ylabel('Reliability', 'FontSize', FONT_SIZE, 'Color', 'black');
xlabel('Time [s]',  'FontSize', FONT_SIZE),
legend('Link 1', 'Link 2','FontSize', FONT_SIZE);

figure(fgDelay);
ylabel('Delay [ms]', 'FontSize', FONT_SIZE, 'Color', 'black');
xlabel('Time [s]',  'FontSize', FONT_SIZE),
legend('Link 1', 'Link 2','FontSize', FONT_SIZE);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot results

% By Link
fprintf('-------------------\nResults by link\n');
for i=1:nLinks
    fprintf('Link %u \n', i);
    printfSimplePerformance(commDataByLinks{i,1}.pckSuccess, commDataByLinks{i,1}.pckTotal);
end


% Global
fprintf('-------------------\nGlobal results\n');
printfSimplePerformance(commData.pckSuccess, commData.pckTotal);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Extra functions
    function printfSimplePerformance(pckSuccess, packetsTotal)
        totalPacketsSum = sum(packetsTotal);
        successPacketsSum = sum(pckSuccess);
        
        disp(sprintf('\tTotal Packets: %0.0f', totalPacketsSum ));
        disp(sprintf('\tSuccess Packets: %0.0f', successPacketsSum ));
        disp(sprintf('\tLoss Packets: %0.0f', totalPacketsSum - successPacketsSum ));
        disp(sprintf('\tReliability: %0.2f %%', successPacketsSum/totalPacketsSum*100 ));
    end

    function [reliability] = getReliability(packetsSuccess, packetsTotal)
        reliability = zeros(length(packetsSuccess), 1);
        for m=1:length(packetsSuccess)
            last = min(i+SLIDING_WINDOW, length(packetsSuccess));
            reliability(m) = sum(packetsSuccess(m:last))/sum(packetsTotal(m:last));
        end
        reliability = smooth(smooth(reliability, SMOOTH_SPAN));
        
    end

    function [  successPackets, totalPackets, errors] = ...
            getStadistics( pckSuccess, pckTotal, events)
        
        errors = zeros(length(pckTotal),1);
        totalPackets = zeros(length(pckTotal),1);
        successPackets = zeros(length(pckTotal),1);
        lossPackets = zeros(length(pckTotal),1);
        
        for m=1:length(errors)
            diff_pck(m) = pckTotal(m)-pckSuccess(m);
        end
        errors = [diff(diff_pck) 0];
        
        for m=2:length(errors)
            % new packet o new sample?
            if ( pckTotal(m-1) ~= pckTotal(m) )
                totalPackets(m) = pckTotal(m) - pckTotal(m-1);
            end
            if ( pckSuccess(m-1) ~= pckSuccess(m) )
                successPackets(m) = pckSuccess(m) - pckSuccess(m-1);
            end
        end
        
    end
end

