% This is the callback function for the timer that we have created in
% the blinkToRadioApp
%
% @param obj
% @param event
% @param string_arg

% @see Callback functions
function timerFired( obj, event, string_arg)
global TestSerial;
global COMM;

% Check if it is not the time when we start the timer
if TestSerial.state ~= TestSerial.INITIALIZATION
    
    % To whatever we want to do periodically
    sendPacket( 0, TestSerial.message.get_counter()+1, 0 );
    
else
    TestSerial.state = TestSerial.RUNNING;
end

end