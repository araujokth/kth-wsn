function [ totalPackets, successPackets, errors] = ...
    getStadistics( pckTotal, pckSuccess , events)

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

%% Let's move the errors between two packets success
idxErrors = find(errors >= 1);
idxEvents = find(events >= 1);
errors2 = zeros(length(errors),1);

for j=1:length(errors)
    if ismember(j, idxErrors)
        id = find(idxEvents == j );
        id = id(find(id > 1));
     idx1 = idxEvents( id - 1);
      for k=1:errors(j)
        errors2(j - floor((j - idx1)/(errors(j)+1)*k)) = 1;
      end
    end
end
errors = errors2;
