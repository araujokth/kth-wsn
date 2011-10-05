function [timeSchedule, tau, ttStart, lateMotes, slots ] = updateTimeslots(y, extraSensors, u, time, integral, y_initial,integral_initial, u_initial,x_initial )
global WTSelf;
global COMM;
WTSelf.DEBUG = false;




% NOTE: SPECIFY THE END VALUES FOR UPPER AND LOWER TANKS BEFORE STARTING
% FOR EACH OF THE TANKS!

% rtk_i: rtk is the final value for the upper and lower tank. The
% subscript i means the tank number. In terms of r2t(1,i) is for the upper
% tank, r2t(2,i) is for the lower tank.

% {4.901961,5.000000}
end_upper = [4.901961 6.485084 8.361204];
end_lower = [5 5 5];
fprintf('+------------------+\nStarting MATLAB script...\n\n');

% Parameters
TAU_K = 10000 * ones(1, WTSelf.nWaterTanks + WTSelf.nSensors);          %set tau_k's to huge value so that we don't mess with calculating the min later on

        ttStart = time;                             % time in symbols for the beacon broadcast
        ttStart = ttStart * WTSelf.SYMBOL;          % time in seconds

% WTSelf.initApp = 0;
if sum(WTSelf.timeSchedule) == 0
    if WTSelf.initApp == false
       initVariables();
        
        fprintf('First run of the script!\n\n INITIALIZING..... \n\n');
        % Save the data
        timeSchedule = WTSelf.timeSchedule;
        tau = TAU_K;
        ttStart = ttStart;
        lateMotes = WTSelf.lateMote; 
        slots = WTSelf.slots;
        return;
    else

        WTSelf.timeSchedule = ttStart * ones(1, WTSelf.nTotal) + WTSelf.deltaCAP;

        %Initial scheduling set up at the BS assigns slots 1->1, 2->2 ... 7->7
        for i=1:WTSelf.nTotal % there are only WTSelf.cfpSlots available slots in the CFP
            WTSelf.timeSchedule(i) = WTSelf.timeSchedule(i) + (i-1) * WTSelf.SD * WTSelf.SYMBOL;     
        end
        
    end    
end

scheduled_next = 10000*ones(2, WTSelf.cfpSlots);

% Clean slots
WTSelf.slots    = WTSelf.IEEE154_aNumSuperframeSlots * ones(1, WTSelf.cfpSlots);

ttStart_next    = ttStart + WTSelf.tBI; % this is the time of the start of the next superframe
% Save the current timeSchedule
% 1) Compute new sampling times
% control loops
%execute ST to get tk+1 of i
% output should be tau_k
if WTSelf.DEBUG 
fprintf('Last beacon+WTSelf.deltaCAP: %f + %f = %f    ...(Anything in between should have been scheduled to transmit!)\nNext beacon+WTSelf.deltaCAP: %f + %f = %f\n', ttStart, WTSelf.deltaCAP, ttStart+WTSelf.deltaCAP, ttStart_next, WTSelf.deltaCAP, ttStart_next+WTSelf.deltaCAP);
fprintf('\nScheduled time for transmission:\n');
for i=1:WTSelf.nTotal
    fprintf('Node %d: %f', i,  WTSelf.timeSchedule(i));
    if WTSelf.timeSchedule(i) <= ttStart_next + WTSelf.deltaCAP
        if sum( WTSelf.lateMote == i )
            fprintf(' *IS GONNA MISS THE DEADLINE!');
        else
            fprintf(' (*)');
        end
    end
    fprintf('\n');
end
end

%% 3. Compute the new timeSchedule for the Water tanks and sensors
for i=1:WTSelf.nWaterTanks
    sensorIdx = 2*i-1;
    actuatorIdx = 2*i;

    % It was scheduled before
    if (WTSelf.timeSchedule( sensorIdx ) <= ttStart_next  + WTSelf.deltaCAP) && ~sum( WTSelf.lateMote == ( sensorIdx ) )
        x10=double(y(i,1))/WTSelf.WT_CALIBRATION - end_upper(i)-x_initial(1,i,1);
        x20=double(y(i,2))/WTSelf.WT_CALIBRATION - end_lower(i)-x_initial(1,i,2);
        %x30=double(integral(i)-end_integral(i));
       %  x1´´ = x1´ - x1_final = x1_measured - x1_operation_point - x1_final

        WTSelf.logger.x10(WTSelf.logger.nSamples,i) = x10;
        WTSelf.logger.x20(WTSelf.logger.nSamples,i) = x20;
        %WTSelf.logger.x30(WTSelf.logger.nSamples) = x30;

        % get next sample time
        %tmp =  WTSelf.lvm(:,:,i) * [x10^2 x10*x20 x10*x30 x20^2 x20*x30 x30^2]';
        tmp =  WTSelf.lvm(:,:,i) * [x10^2 x10*x20 x20^2]';
        %find the first negaive value
        next_timetmp = find(tmp < 0, 1, 'first'); % this is in number of beacons ahead to trigger
        next_time=next_timetmp*WTSelf.tBI; % this is in seconds!
        next_time=WTSelf.tBI; % Added for debugging
        
        if isempty(next_time)
            next_time = WTSelf.maxSampleInterval;
        elseif next_time > WTSelf.maxSampleInterval
            next_time = WTSelf.maxSampleInterval;
        end
        
        fprintf('Tank %d => next_time = %0.2f s \n', i, next_time);
        
        WTSelf.timeSchedule( sensorIdx ) = WTSelf.timeSchedule( sensorIdx ) ...
            + real(next_time); % WTSelf.timeSchedule stores the value of tk+1
        WTSelf.timeSchedule( actuatorIdx ) = WTSelf.timeSchedule( actuatorIdx ) ...
            + real(next_time); % WTSelf.timeSchedule stores the value of tk+1
        
        TAU_K(i)=real(next_time); % store all the periods
        if WTSelf.DEBUG 
        fprintf(' => Node %d was scheduled! => next time: %f => new scheduled time: %f \n', sensorIdx, TAU_K(i), WTSelf.timeSchedule( sensorIdx ));
        fprintf(' => Node %d was scheduled! => next time: %f => new scheduled time: %f \n', actuatorIdx, TAU_K(i), WTSelf.timeSchedule( actuatorIdx ));
        end
    end
end

    % low priority loops
    % we can set some conditions on the value of the measurement:
    % e.g. if the temperature is higher than something then we sample
    % faster!
for i=1:WTSelf.nSensors
    node = 2* WTSelf.nWaterTanks + i;
    
    if (WTSelf.timeSchedule(node) <= ttStart_next + WTSelf.deltaCAP) && ~sum( WTSelf.lateMote == node )  % control loops

        next_time = 1 + (WTSelf.nWaterTanks + i - 2 )*rand(1);
        next_time = WTSelf.tBI+i*0.02;%rand(1)/8;
        WTSelf.timeSchedule(node) = WTSelf.timeSchedule(node) + real(next_time);    % time_schedule stores the value of tk+1
        TAU_K(WTSelf.nWaterTanks +i) = real(next_time);                               % store all the periods
    if WTSelf.DEBUG 
        fprintf(' => Node %d was scheduled! => next time: %f => new scheduled time: %f \n', node, TAU_K(WTSelf.nWaterTanks +i),  WTSelf.timeSchedule(node));
    end
    end
end

WTSelf.currentBO = WTSelf.DEFAULT_BO; % Fixed BO!!!

%% 2.Scheduler
% 2) if the node has a tk+1, then we check if it is between the next
% superframe which is ttStart_next+WTSelf.deltaCAP to ttStart_next+deltaBI
% 2.a) if this is true, store the node in a new vector
% scheduled_next=[nodeID;time];

j=1;
if WTSelf.DEBUG 

fprintf('\nNow check if we need to allocate slots in the next beacon interval...\n');
fprintf('Last beacon+WTSelf.deltaCAP: %f + %f = %f \nNext beacon+WTSelf.deltaCAP: %f + %f = %f\n', ttStart, WTSelf.deltaCAP, ttStart+WTSelf.deltaCAP, ttStart_next, WTSelf.deltaCAP, ttStart_next+WTSelf.deltaCAP);

fprintf('Next beacon + beacon interval + WTSelf.deltaCAP= %f + %f + %f = %f\n', ttStart_next, WTSelf.tBI, WTSelf.deltaCAP, ttStart_next+WTSelf.tBI+WTSelf.deltaCAP);
fprintf('Anything before that should get a slot!\n');
fprintf('Scheduled time for transmission:\n');
end
for k=1:WTSelf.nTotal
    if WTSelf.DEBUG 
    fprintf('Node %d: %f', k, WTSelf.timeSchedule(k));
    if WTSelf.timeSchedule(k) <= ttStart_next + WTSelf.tBI + WTSelf.deltaCAP
        fprintf(' (*)');
    end
    fprintf('\n');
    end
    if WTSelf.timeSchedule(k) <= ttStart_next + WTSelf.tBI + WTSelf.deltaCAP %&& WTSelf.timeSchedule(i)>=ttStart_next+WTSelf.deltaCAP
        if WTSelf.DEBUG  fprintf(' => Node %u should get a slot in the next beacon interval!\n', k); end
        scheduled_next(1,j)=k;
        scheduled_next(2,j)=WTSelf.timeSchedule(k);% this vector contains the time of nodes scheduled for next superframe
        j=j+1;
    end
    
end

nextBeaconItems = j;

[B,pos]=sort(scheduled_next(2,:),2);

% Assign slots in EDF fashion, Note: the actuator needs to be assigned
% after the measurement
fprintf('\nHanding out slots...\n');
slot_num=1;
l=1;

for k=1:nextBeaconItems-1 % we can also write slot=pos
    node = scheduled_next(1,pos(k));
    
    % It is a Water tank?
    if (node <= WTSelf.nWaterTanks * 2)
        % Is an actuator?
        if mod(node, 2) == 1
            for j=node:(node+1)
                WTSelf.slots(j)          = slot_num; % slot stores the assigned slots
                WTSelf.timeSchedule(j)   = ttStart_next + WTSelf.deltaCAP + (slot_num-1) * WTSelf.slotLength * WTSelf.SYMBOL;
                % Added for debugging
                WTSelf.logger.timeAllocated (WTSelf.logger.nSamples,j)   = WTSelf.timeSchedule(j);
                fprintf(' => WT Node %u got slot %u!\n', j, slot_num);
                slot_num = slot_num+1;
            end
        end
    else  
        if slot_num <= WTSelf.cfpSlots
            % Sensor node "node" got an slot
            fprintf(' => Sensor node %u got slot %u!\n', node, slot_num);
            WTSelf.slots(node)          = slot_num;
            WTSelf.timeSchedule(node)   = ttStart_next + WTSelf.deltaCAP + (slot_num-1) * WTSelf.slotLength * WTSelf.SYMBOL;
            % Added for debugging
            WTSelf.logger.timeAllocated (WTSelf.logger.nSamples,node)   = WTSelf.timeSchedule(node);
            slot_num                    = slot_num + 1;
            
        else
             WTSelf.lateMote(l) = node;
             l = l + 1;
             fprintf(' => Node %u did not get a slot and is gonna be late!\n', node);
        end
    end
end

%% 4. Sort slots vector
%   Before sending the slots, in order to simplify the code in the motes we
%   sort them here

% The slot number is known in the mote and matlab, so we just assign who
% needs to transmit on each slot
idxValid = find(WTSelf.slots ~= WTSelf.IEEE154_aNumSuperframeSlots)
[tmp, idx ] = sort( WTSelf.slots(idxValid) )
idx = idxValid(idx)
WTSelf.IEEE154_aNumSuperframeSlots * ones(1, WTSelf.cfpSlots);
WTSelf.slots    =20*ones(1, WTSelf.cfpSlots); % 2WTSelf.IEEE154_aNumSuperframeSlots * ones(1, WTSelf.cfpSlots)
% put them in their positions
WTSelf.slots(tmp) = idx;
disp(['printing ', num2str(WTSelf.slots(tmp)) ]);


message = SuperframeConfigMsg;

message.set_BI( WTSelf.BI(WTSelf.currentBO+1) );
message.set_timeslots( WTSelf.slots );
 
 send(COMM.TOS_BCAST_ADDR, message, ...
     sprintf('%s%d', WTSelf.connectionName));

% Save the data
timeSchedule = WTSelf.timeSchedule;
tau = TAU_K;
ttStart = ttStart;
lateMotes = WTSelf.lateMote; 
slots = WTSelf.slots;



%fprintf('\n');

    function initVariables()
        WTSelf.currentBO    = WTSelf.DEFAULT_BO;
        WTSelf.tBI          = WTSelf.BI(WTSelf.currentBO+1);

        WTSelf.timeSchedule = zeros(1, WTSelf.nTotal);
    
        TAU_K               = zeros(1, WTSelf.nWaterTanks + WTSelf.nSensors);
        
        WTSelf.lateMote = zeros(1, WTSelf.nLate);
        for i=1:WTSelf.nLate
           WTSelf.lateMote(i) =  WTSelf.capDuration + WTSelf.cfpSlots+i;
        end
                WTSelf.initApp = true;


end
end