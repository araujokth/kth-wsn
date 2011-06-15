function [ output_args ] = plotGlobal( file, initTime, endTime)

global TestSerial;

LINE_STYLE = {'b-', 'r-', 'c-', 'g-.', 'rx', 'bo', 'mx', 'go'};
COLOR= ['b';'r';'m';'g';'c'];
LINE_WIDTH = [1 3];

% We load the file here and then it will be easy to compare
% different controllers
load(file); % It load the TestSerial

initTime = 1;
endTime = TestSerial.logger.nSamples - 1;

fprintf('plotGlobal(''%s'', %d, %d); \n', file, initTime, endTime);

%% Plot the reciver/sent counter
figure,
plot([TestSerial.logger.counterReceived' , TestSerial.logger.counterSent']),
xlabel('Samples'),
ylabel(''),
legend('Counter Received from the Sensor', 'Counter sent to the actuator');

% Tools
    function removePoints(hLine, x, y, idx)
        yNew = y(idx);
        xNew = x(idx);
        set(hLine, 'XData', xNew, 'YData', yNew);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

