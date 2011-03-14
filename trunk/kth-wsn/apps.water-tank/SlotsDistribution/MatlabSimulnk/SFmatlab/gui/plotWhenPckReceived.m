function plotWhenPckReceived()
global WTPLOTS;
global WTAPP
hf = findall(0, 'tag', 'figure1');
handles = guidata(hf);

     axes(handles.u);
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
    plot(WTPLOTS.u(end-1:end, WTAPP.ACTUATOR_WT));
    %adds a title, x-axis description, and y-axis description
    title('Voltage levels');
    xlabel('time []');
    ylabel('voltage [V]');
    
end
