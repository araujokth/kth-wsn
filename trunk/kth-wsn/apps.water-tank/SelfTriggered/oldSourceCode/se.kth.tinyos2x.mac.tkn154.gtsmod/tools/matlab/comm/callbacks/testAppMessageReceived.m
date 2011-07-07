function testAppMessageReceived(address, testAppMsg, varargin)
    global TESTAPP
    global COMM 
    disp(sprintf('recieved: id=%d counter=%d ',testAppMsg.get_sensor_id, testAppMsg.get_value ));
    message = TESTAPP.MsgClass;
    message.set_sensor_id(1);
    message.set_value(10);

    send(COMM.TOS_BCAST_ADDR, message);
    disp(sprintf('sent to %d : sensor_id=%d value=%d ', COMM.TOS_BCAST_ADDR, message.get_sensor_id, message.get_value ));
  
