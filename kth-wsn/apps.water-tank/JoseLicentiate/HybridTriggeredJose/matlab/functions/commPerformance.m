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

commData.rssi = typecast(uint8(commData.rssi), 'int8');
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

%% Sort data by WateTanks
% compute water tank id.
% i. Odd number (Actuators) -> id/2 = wtId
% ii. Even number (Sensors) -> (id+1)/2 = wtId

sensorsRows = commData.srcId > 0;
wtIds = (commData.srcId(sensorsRows)+1)/2;
waterTanks = unique(wtIds);
nWaterTanks = length(waterTanks);
commDataByWaterTank = cell(nWaterTanks, 1);

%% Find by water tanks
for i=1:nWaterTanks
    % srcId == wtId*2
    sensorsWT = commData.srcId == ( waterTanks(i)*2-1 );
    % dstId == wtId*2-1
    actuatorWT = commData.dstId == ( waterTanks(i)*2 );
    
    idx = sensorsWT | actuatorWT;
    
    wtStruct = struct;
    for j=1:length(commHeader)
        f =  commData.( commHeader{j} );
        wtStruct.( commHeader{j} ) = f(idx);
    end
    commDataByWaterTank{i} = wtStruct;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 
for i=1:nLinks
    %% Take only sensor
    if (mod(commDataByLinks{i,1}.srcId(1), 2) == 0)
        figure;
        diffET =  diff(commDataByLinks{i,1}.timestamp);
        stampET = commDataByLinks{i,1}.timestamp(2:end);
        plot(stampET, diffET, '+'),
        ylabel('Inter-event time t_{k+1} - t_{k} (s)', 'FontSize', FONT_SIZE, 'Color', 'black');
        xlabel('Time (s)',  'FontSize', FONT_SIZE); 
        lgd = legend('Event-triggered');
        %set(lgd, 'FontSize', FONT_SIZE);
        %set(gca, 'FontSize', FONT_SIZE);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot results

% By Link
fprintf('-------------------\nResults by link\n');
for i=1:nLinks
    fprintf('Link %u \n', i);
    printfSimplePerformance(commDataByLinks{i,1}.pckSuccess, commDataByLinks{i,1}.pckTotal);
end

% By Water Tank
fprintf('-------------------\nResults by water tank\n');
for i=1:nWaterTanks
    fprintf('Water Tank %u \n', i);
    printfSimplePerformance(commDataByWaterTank{i}.pckSuccess, commDataByWaterTank{i}.pckTotal);
end

% Global
fprintf('-------------------\nGlobal results\n');
printfSimplePerformance(commData.pckSuccess, commData.pckTotal);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot results

% RSSI
figure,
plot(commData.timestamp, commData.rssi),
xlabel('Time [s]'),
ylabel('RSSI [dBm]');

% LQI
figure,
plot(commData.timestamp, commData.lqi),
xlabel('Time [s]'),
ylabel('LQI = (0, 255)');


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
        for i=1:length(packetsSuccess)
            last = min(i+SLIDING_WINDOW, length(packetsSuccess));
            reliability(i) = sum(packetsSuccess(i:last))/sum(packetsTotal(i:last));
        end
        reliability = smooth(smooth(reliability, SMOOTH_SPAN));
        
    end

    function [  successPackets, totalPackets, errors] = ...
            getStadistics( pckSuccess, pckTotal, events)
        
        errors = zeros(length(pckTotal),1);
        totalPackets = zeros(length(pckTotal),1);
        successPackets = zeros(length(pckTotal),1);
        lossPackets = zeros(length(pckTotal),1);
        
        for i=1:length(errors)
            diff_pck(i) = pckTotal(i)-pckSuccess(i);
        end
        errors = [diff(diff_pck) 0];
        
        for i=2:length(errors)
            % new packet o new sample?
            if ( pckTotal(i-1) ~= pckTotal(i) )
                totalPackets(i) = pckTotal(i) - pckTotal(i-1);
            end
            if ( pckSuccess(i-1) ~= pckSuccess(i) )
                successPackets(i) = pckSuccess(i) - pckSuccess(i-1);
            end
        end
        
    end
end

