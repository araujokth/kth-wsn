function sendPacket( dummy, counter, transferTo )

global COMM;
global TestSerial;

    TestSerial.message.set_dummy(dummy);
    TestSerial.message.set_counter(counter);
    
    send(COMM.TOS_BCAST_ADDR, TestSerial.message, ...
     sprintf('%s%d', TestSerial.connectionName, ...
     TestSerial.connectionPortInit + transferTo)); 
    
    % Write the value in the GUI if is used
    if ~isempty(TestSerial.sentValue)
        set(TestSerial.sentValue, 'String', TestSerial.message.get_counter());
    end
end

