% This is the callback function for the listener that we have created in
% the blinkToRadioApp
%
% @param address    it doesn't contains the address of the transmitter
% @param msg        contains the message that we have received. In this
%                   case we know that it will be a TestSerial.msgClass
%                    message
function readFromBaseStation(address, msg, varargin)
global TestSerial;
global COMM;

%% Read the message
counter = msg.get_counter();
set(TestSerial.receivedValue, 'String', counter);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Modify values
% Do we need to modify the controller/value or something?
% In our case we increment the counter
TestSerial.message.set_counter(counter + 5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Logging the date if desired
n = TestSerial.logger.nSamples;

% Here we could fill our logging variables
TestSerial.logger.counterReceived(n) = counter;
TestSerial.logger.counterSent(n) = TestSerial.message.get_counter();

% update the number of samples     
TestSerial.logger.nSamples = n+1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Send a message back to the mote
if TestSerial.tutorialApp == TestSerial.TEST_HIL
    sendPacket( 0, TestSerial.message.get_counter(), 1 );
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
        
    