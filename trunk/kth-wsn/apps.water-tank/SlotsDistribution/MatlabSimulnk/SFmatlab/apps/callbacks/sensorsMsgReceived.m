function sensorsMsgReceived(address, WTAPPMsg, varargin)
global WTAPP
global WTPLOTS
global COMM

%% we receive the sensor values and convert to cm

currentTime = clock;
WTAPP.tbp = etime(currentTime, WTAPP.lastSampleTime);
WTAPP.lastSampleTime = currentTime;

WTAPP.tankLevel1 = double(WTAPPMsg.get_tankLevel1) / double(WTAPP.CALIBRATION);
WTAPP.tankLevel2 = double(WTAPPMsg.get_tankLevel2) / double(WTAPP.CALIBRATION);

if (isempty(WTAPP.offsets))
     WTAPP.offsets(:,1) = WTAPP.tankLevel1 / double(WTAPP.CALIBRATION);
     WTAPP.offsets(:,2) = WTAPP.tankLevel2 / double(WTAPP.CALIBRATION);
end

WTAPP.tankLevel1 = WTAPP.tankLevel1 - WTAPP.offsets(:,1);
WTAPP.tankLevel2 = WTAPP.tankLevel2 - WTAPP.offsets(:,2);


disp(sprintf('[rec: %0.3f s exp: %0.3f]', WTAPP.tbp, WTAPP.SAMPLING_INTERVAL));
double(WTAPP.tankLevel1);
double(WTAPP.tankLevel2);

double(WTAPP.offsets(:,1));
double(WTAPP.offsets(:,2));

%% Compute the control by using the CVX
WTAPP.u = computeControl(double(WTAPPMsg.get_tankLevel1), double(WTAPPMsg.get_tankLevel2));

%% Send the Actuation Matrix
message = ActuationMatrixMsg;

% Saturation
for i=1:length(WTAPP.tankLevel1)
    if (WTAPP.u(i) > 12) WTAPP.u(i) = 12;
        warning(sprintf('[WT = %d] Voltage bigger than 12V', i));
    elseif WTAPP.u(i) < 0 WTAPP.u(i) = 0;
        warning(sprintf('[WT = %d] Negative voltage', i));
    end
end
%WTAPP.u = (WTAPP.u - 0.0009710)/0.0006151; %conversion to DAC
WTPLOTS.u(WTPLOTS.instant, :) = WTAPP.u;
WTAPP.u = WTAPP.u*4095/12;
disp(WTAPP.u);
message.set_u( WTAPP.u );

% save data
WTPLOTS.tankLevel1(WTPLOTS.instant, :) = WTAPP.tankLevel1;
WTPLOTS.tankLevel2(WTPLOTS.instant, :) = WTAPP.tankLevel2;
WTPLOTS.integrals(WTPLOTS.instant, :) = WTAPP.integrals;
WTPLOTS.instant = WTPLOTS.instant + 1;

%% Check tanks levels
for i=1:length(WTAPP.tankLevel1)
    if ((WTAPP.tankLevel1(i) > WTAPP.LIMIT) || ...
            (WTAPP.tankLevel2(i) > WTAPP.LIMIT) ) 
        
        message.set_u(zeros(WTAPP.NUMBER_WT, 1));

        warning(sprintf('[WT = %d] Limit level reached', i));
    end
end


%timerFired();

% Send the message
       % message.set_u(zeros(WTAPP.NUMBER_WT, 1));

send(COMM.TOS_BCAST_ADDR, message);

