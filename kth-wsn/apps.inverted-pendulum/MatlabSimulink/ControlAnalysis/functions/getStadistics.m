function [errors, reliability ] = ...
    getStadistics (update, pck_success,  pck_total)
%#eml
% 
persistent errors_window item pckTotalTrack prevTotalPackets;
 
SLIDING_WINDOW = 100;

% First initialization of the variables
if isempty(item)
    errors_window = zeros(SLIDING_WINDOW, 1);
    pckTotalTrack = zeros(SLIDING_WINDOW, 1);
    item = 1;
end

currentItem = mod(item + 1, SLIDING_WINDOW);
% It could be that the number of received packets
% is not the total packets sent, so we need
% to keep track of it.
if (pck_total == 0 || isempty(prevTotalPackets))
    pckTotalTrack(currentItem) = 1;
    prevTotalPackets = pck_total;
else
    pckTotalTrack(currentItem) = pck_total - prevTotalPackets;
end

% if the previous errors in less than the actual value
% it means that we get a need error
if errors_window(item) < (pck_total - pck_success)
    errors_window(currentItem) = (pck_total - pck_success) - errors_window(item);
else
    errors_window(currentItem) = 0;
end

item = currentItem;

%
% Map the outputs
%
errors = errors_window(item);
reliability = 1-sum(errors_window)/(sum(pckTotalTrack));