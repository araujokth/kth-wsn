function timerFired(obj, event, string_arg )
    global COMM
    global TESTAPP

    disp('fired');
    message = TESTAPP.MsgClass;
    message.set_sensor_id(0);
    message.set_value(10);

    send(COMM.TOS_BCAST_ADDR, message);
    disp(sprintf('sent to %d : src_id=%d counter=%d ', COMM.TOS_BCAST_ADDR, message.get_sensor_id, message.get_value ));


end

