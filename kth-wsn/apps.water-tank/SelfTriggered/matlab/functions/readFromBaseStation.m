function readFromBaseStation(address, msg, varargin)
global WTSelf;
global WTModeling;
global COMM;

y = msg.get_y();
y_initial = msg.get_y_initial();
u = msg.get_u();
u_initial = msg.get_u_initial();
extraSensors = msg.get_extraSensors();
integral = msg.get_integral();
integral_initial = msg.get_integral_initial();
time = msg.get_time();
ref = msg.get_ref();
n = WTSelf.logger.nSamples;

        WTSelf.logger.timeSchedule(n,:) = WTSelf.tBI;
        WTSelf.logger.slots(n,:) = WTSelf.defaultSlots;
        WTSelf.logger.ttStart(n,1) = time*WTSelf.SYMBOL;
        WTSelf.logger.tau(n,:) = WTSelf.tBI * ones(1,WTSelf.nWaterTanks+ WTSelf.nSensors);
        WTSelf.logger.lateMotes(n,:) = zeros(1, WTSelf.nLate);
        
switch(WTSelf.controller)
    case WTSelf.EVENT_TRIGGERED_CONTROLLER
        if ref == WTSelf.REFERENCE
            
            tempDmax = 0.2;
            tempDmaxSlots = floor( tempDmax / (WTSelf.slotLength * WTSelf.SYMBOL) );
            
            message = SuperframeConfigMsg;
            message.set_BI( 6 );
            
            WTSelf.slots    = WTSelf.IEEE154_aNumSuperframeSlots * ones(1, WTSelf.cfpSlots);
            
            j=1; i= 1;
            maxNum = 1;
            
            while maxNum < WTSelf.cfpSlots
                maxNum = min(WTSelf.cfpSlots,  j + WTSelf.nWaterTanks*2 - 1);
                
                if (size(j:maxNum, 2) == WTSelf.nWaterTanks*2)
                    WTSelf.slots(j:(maxNum)) = 1:WTSelf.nWaterTanks*2;
                else
                    break;
                end
                
                j = maxNum + tempDmaxSlots;
                i = i + 1;
            end
            message.set_timeslots( WTSelf.slots );
            
            send(COMM.TOS_BCAST_ADDR, message, ...
                sprintf('%s%d', WTSelf.connectionName));
        end
    case WTSelf.PERIODIC_CONTROLLER
        message = SuperframeConfigMsg;

        % We send the slots when we want :P
        
        message.set_BI( 255 );                   % Before it was 0xFF and the mote thinks that it is the initialization
        
        % Build the slots for the water tanks when we have the period
        %WTSelf.slots = 1:length(WTSelf.slots);
        message.set_timeslots( WTSelf.slots );

         send(COMM.TOS_BCAST_ADDR, message, ...
             sprintf('%s%d', WTSelf.connectionName));
    case WTSelf.SELFTRIGGERED_CONTROLLER
        if ref == WTSelf.REFERENCE
            [WTSelf.logger.timeSchedule(n,:), WTSelf.logger.tau(n,:), ...
                WTSelf.logger.ttStart(n,1), ...
                WTSelf.logger.lateMotes(n,:), WTSelf.logger.slots(n,:)] = ...
                updateTimeslots(y, extraSensors, u, time, integral, y_initial,...
                integral_initial, u_initial );
        end
        
    case WTSelf.MODELING_CONTROLLER
        % constant values
WTModeling.u_5cm = [3];
WTModeling.u_10cm = [6];

% step response
WTModeling.controller = zeros(1, 3, WTSelf.nWaterTanks);
WTModeling.controller(:, :, 1) = [-0.9523   -0.0395   -0.0543];

        if WTModeling.state == WTModeling.INIT
            fprintf('Modeling. Initialization \n');
            fprintf('Modeling. Constant response.Time left: %d s \n', ...
                ceil(WTModeling.STEADY_STATE_TIME(1,1)/WTSelf.tBI) - WTSelf.logger.nSamples);
            


            WTModeling.u = WTModeling.u_5cm;
            if WTSelf.logger.nSamples > (WTModeling.STEADY_STATE_TIME(1,1)/WTSelf.tBI)
                WTModeling.state = WTModeling.STEP_RESPONSE;
                waterTanksSelfTriggered('resetIntegral');
            end
        elseif WTModeling.state == WTModeling.STEP_RESPONSE
            fprintf('Modeling. Step response.Time left: %d s \n', ...
                ceil(WTModeling.STEADY_STATE_TIME(1,2)/WTSelf.tBI) - WTSelf.logger.nSamples);

            % Implement the controller here
            for i=1:WTSelf.nWaterTanks
                state = [(double(y(i,:))/WTSelf.WT_CALIBRATION - 10) integral(i)];
                WTModeling.u(i) = state * WTModeling.controller(:, :, i)';
            end
            
            % Has it reached the steady state?
            if WTSelf.logger.nSamples > (WTModeling.STEADY_STATE_TIME(1,2)/WTSelf.tBI)
                WTModeling.STEADY_STATE_TIME(2,2) = WTSelf.logger.nSamples;
                WTModeling.u0 = WTModeling.u;
                WTModeling.xf = double(y)/WTSelf.WT_CALIBRATION;
                for i=1:WTSelf.nWaterTanks
                    WTModeling.parameters.gamma(i) = WTModeling.xf(i,2)/WTModeling.xf(i,1);
                end
                % Go to the next state
                WTModeling.state = WTModeling.INCR_RESPONSE;
            end
            
        elseif WTModeling.state == WTModeling.INCR_RESPONSE
            fprintf('Modeling. Increment response. Time left: %d s \n', ...
                ceil(WTModeling.STEADY_STATE_TIME(1,3)/WTSelf.tBI) - WTSelf.logger.nSamples);

            WTModeling.u = WTModeling.u0 + WTModeling.incr_u;
            
             % Has it reached the steady state?
            if WTSelf.logger.nSamples >= ceil(WTModeling.STEADY_STATE_TIME(1,3)/WTSelf.tBI)
                WTModeling.STEADY_STATE_TIME(2,3) = WTSelf.logger.nSamples;
                WTModeling.delta_xf = double(y)/WTSelf.WT_CALIBRATION - WTModeling.xf;
                WTModeling.parameters.k(:) = WTModeling.delta_xf(:,1)/WTModeling.incr_u;
                % Go to the next state
                WTModeling.state = WTModeling.END;
            end
            
        elseif WTModeling.state == WTModeling.END
            fprintf('Modeling. End \n');
            for i=1:WTSelf.nWaterTanks
                idx = find(WTSelf.logger.y(WTModeling.STEADY_STATE_TIME(2,2):WTModeling.STEADY_STATE_TIME(2,3), i, 1)/WTSelf.WT_CALIBRATION >= ...
                    (WTModeling.xf(i,1) + WTModeling.delta_xf(i,1) * 0.632), 1, 'first');
                WTModeling.parameters.tau(i) = idx * WTSelf.tBI;
                
                fprintf('[ %0.0f ] xf=%0.2f ; \n',WTModeling.STEADY_STATE_TIME(2,2), WTSelf.logger.y(WTModeling.STEADY_STATE_TIME(2,2), i, 1));
                fprintf('[ %0.0f ] 0.632*delta_xf=%0.2f ; \n',WTModeling.parameters.tau(i),  ...
                    (WTSelf.logger.y(WTModeling.STEADY_STATE_TIME(2,3), i, 1) - WTSelf.logger.y(WTModeling.STEADY_STATE_TIME(2,2), i, 1))*0.632);
                
                fprintf('[ %0.0f ] delta_xf=%0.2f ; \n',WTModeling.STEADY_STATE_TIME(2,3),  ...
                    WTSelf.logger.y(WTModeling.STEADY_STATE_TIME(2,3), i, 1) - WTSelf.logger.y(WTModeling.STEADY_STATE_TIME(2,2), i, 1));
            end
 
            WTModeling.u = zeros(WTSelf.nWaterTanks,1);
            if ~WTModeling.end
                % Save all the data
                c = clock;
                m_file = sprintf('mat/models/%dtanks_%s_%02.f%02.f%02.f.mat', ...
                     WTSelf.nWaterTanks, ...
                    datestr(date, 'yymmdd'), c(4), c(5), c(6) );
                save(m_file, 'WTModeling');   
                
                % Save the logger
                waterTanksSelfTriggered('saveInformation');
            end
            WTModeling.end = true;

        end
        
        % Send the command to start the modeling routine
        message = ControlMsg;
        WTModeling.u
        message.set_cmd(WTSelf.MODELING_CONTROLLER);
        message.set_u(WTModeling.u);
        
        send(COMM.TOS_BCAST_ADDR, message, ...
            sprintf('%s%d', WTSelf.connectionName));
        
    case WTSelf.CONSTANT_CONTROLLER
        message = ControlMsg;
        fprintf('Constant voltage: %0.5f \n', WTModeling.uConstant);
        WTModeling.u = WTModeling.uConstant * ones(WTSelf.nWaterTanks,1);
        message.set_cmd(WTSelf.CONSTANT_CONTROLLER);
        message.set_u(WTModeling.u);
        
        send(COMM.TOS_BCAST_ADDR, message, ...
            sprintf('%s%d', WTSelf.connectionName));
    otherwise
end

% Store the new data received
WTSelf.logger.y(n,:,:) = y;
WTSelf.logger.y_initial(n,:,:) = y_initial;
WTSelf.logger.u(n,:,:) = u;
WTSelf.logger.u_initial(n,:,:) = u_initial;
WTSelf.logger.integral(n,:) = integral;
WTSelf.logger.integral_initial(n,:) = integral_initial;
WTSelf.logger.time(n,:) = time;
WTSelf.logger.nSamples = n+1;

% Show the sensors information
for i=1:WTSelf.nWaterTanks
%     fprintf('[%0.2f] [WT=%d] upperTank=(%0.2f, %0.2f) ; lowerTank=(%0.2f, %0.2f) ; \n \t u=%0.2f ; u_init=%0.2f \n integral=%0.0f\n', ...
%         WTSelf.logger.ttStart(n,1), i, double(y_initial(i,1))/87.3932, double(y(i,1))/87.3932,...
%         double(y_initial(i,2))/87.3932, double(y(i,2))/87.3932 , double(u(i))/273, double(u_initial(i))/273,  double(integral(i)));

% fprintf('[%0.2f] [WT=%d] upperTank=%0.2f ; lowerTank=%0.2f ; \n \t u=%0.2f ;\n integral=%0.3f\n', ...
%         WTSelf.logger.ttStart(n,1), i, double(y_initial(i,1))/87.3932 + double(y(i,1))/87.3932,...
%         double(y_initial(i,2))/87.3932 + double(y(i,2))/87.3932 , double(u(i))/273,  integral(i));
fprintf('[%0.2f] [WT=%d] upperTank=%0.2f ; lowerTank=%0.2f ; \n \t u=%0.2f ; u_0=%0.2f \n integral=%0.3f integral_init=%0.3f \n ref=%0.3f cm \n', ...
        WTSelf.logger.ttStart(n,1), i, double(y(i,1))/WTSelf.WT_CALIBRATION,...
        double(y(i,2))/WTSelf.WT_CALIBRATION , double(u(i))/273, double(u_initial(i))/273,   integral(i), integral_initial(i), ref);

end
fprintf('...........\n');

        
    