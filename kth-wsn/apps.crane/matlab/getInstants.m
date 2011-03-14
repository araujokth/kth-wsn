function [totalPackets, successPackets, ...
    idxPck, idxErrorsSf, idxErrorsWireless, packets, timestamp, delay] ...
    = getInstants(fileToRead1)
    rawData1 = importdata(fileToRead1);

    T_SYMBOL= 1/(2*32678);
    THRESHOLD_DELAY = 0.05; %50 ms

    close all;
    % For some simple files (such as a CSV or JPEG files), IMPORTDATA might
    % return a simple array.  If so, generate a structure so that the output
    % matches that from the Import Wizard.
    [~,name] = fileparts(fileToRead1);
    newData1.(genvarname(name)) = rawData1;

    pckTotal= rawData1(:,1);
    pckSuccess  = rawData1(:,2);
    timestamp = rawData1(:,3)*T_SYMBOL;
    delay = rawData1(:,4)*T_SYMBOL;
    
    timestamp = timestamp - timestamp(1);
    timestamp = timestamp(1:end-1);
    
    packets = diff(pckTotal);
    % figure; plot(diff(pckSuccess));
    %figure; plot(packets);
     %figure; plot(diff(timestamp));

    [ totalPackets, successPackets, errorsWireless] = ...
        getStadistics( pckTotal, pckSuccess, timestamp );

    idxPck = find(packets == 1);
    idxErrorsSf = find(packets > 1);
    idxErrorsWireless = find(errorsWireless == 1);
    
    %% clean delay
    for i=2:length(delay)

       if abs(delay(i)) > THRESHOLD_DELAY || delay(i) < 0
           delay(i) = delay(i-1);
       end
    end

    
    
   % plotInstants(packets, errorsWireless, timestamp, idxPck, idxErrorsSf, idxErrorsWireless);
%     figure;
%     hold on;
%     t = timestamp(1:end-1);
%     hLine = stem(timestamp(1:end-1), packets, 'b');
%     yNew = packets(idxPck);
%     xNew = t(idxPck);
%     set(hLine, 'XData', xNew, 'YData', yNew);
% 
%     hLine = stem(timestamp(1:end-1), packets, 'g');
%     yNew = packets(idxErrorsSf);
%     xNew = t(idxErrorsSf);
%     set(hLine, 'XData', xNew, 'YData', yNew);
% 
%     hLine = stem(timestamp, errorsWireless, 'r');
%     yNew = packets(idxErrorsWireless);
%     xNew = t(idxErrorsWireless);
%     set(hLine, 'XData', xNew, 'YData', yNew);
%     hold off;

end
