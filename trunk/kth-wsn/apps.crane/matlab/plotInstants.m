function plotInstants(packets, errorsWireless, t, idxPck, idxErrorsSf, idxErrorsWireless)
figure;
    hold on;
    hLine = stem(t, packets, 'b');
    yNew = packets(idxPck);
    xNew = t(idxPck);
    set(hLine, 'XData', xNew, 'YData', yNew);

    hLine = stem(t, packets, 'g');
    yNew = packets(idxErrorsSf);
    xNew = t(idxErrorsSf);
    set(hLine, 'XData', xNew, 'YData', yNew);

    hLine = stem(t, errorsWireless(1:end-1), 'r');
    yNew = errorsWireless(idxErrorsWireless);
    xNew = t(idxErrorsWireless);
    set(hLine, 'XData', xNew, 'YData', yNew);
    hold off;
    
end