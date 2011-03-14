function timerFired(obj, event, string_arg, s2 )
global WTPLOTS;
global WTAPP
hf = findall(0, 'tag', 'figure1');
handles = guidata(hf);

     %axes(handles.u);
     thisLineHandle = handles.hl;
        % Get the data currently being displayed on the axis
        xdata = get(thisLineHandle,'XData');
        ydata = get(thisLineHandle,'YData');
        
        if length(xdata) < 10
        if isempty(xdata)
            newXData =[1];
        else
            newXData = [xdata xdata(end)+1];
        end
        newYData = [ydata WTPLOTS.u(end,1)];
    else
        newXData = [xdata(2:end) xdata(end)+1];
        newYData = [ydata(2:end) WTPLOTS.u(end,1)];
    end
    set(thisLineHandle,...
        'XData',newXData,...
        'YData',newYData);
    
%     thisLineHandle = handles.lineHandles(idx);
%         
%     % Get the data currently being displayed on the axis
%     xdata = get(thisLineHandle,'XData');
%     ydata = get(thisLineHandle,'YData'); 
%     %creates a vector from 0 to 10, [0 1 2 3 . . . 10]
%     x = 0:10;
%     %creates a vector from 0 to 10, [0 1 2 3 . . . 10]
%     y = 0:10;
% 
%     %plots the x and y data
%     plot(WTPLOTS.u(end-1:end, WTAPP.ACTUATOR_WT));
%     %adds a title, x-axis description, and y-axis description
%     title('Voltage levels');
%     xlabel('time []');
%     ylabel('voltage [V]');
    
end
