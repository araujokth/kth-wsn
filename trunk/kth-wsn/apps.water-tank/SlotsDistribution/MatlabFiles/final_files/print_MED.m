% prints the figures for MED_2011 conference

clc
clear all
close all


load cdc_sinusoid

sim_time = size(Xlog,2);
N = data.nSensors;
times = data.h:data.h:data.h*sim_time; 



% Plot traces and envelope
figure(1)
range = sqrt(cost.c{1}); % assumes all are controlled to the same range
plot(times,Xlog(2:2:20,:))
hold on
plot([times(1),times(sim_time)],[Xref-range, Xref-range],'--k')
plot([times(1),times(sim_time)],[Xref+range, Xref+range],'--k')
%plot(times,eta(1,:),'.k')
%plot(times,eta(2,:),'.k')
axis([data.h sim_time*data.h 0 20]);
xlabel('seconds')
ylabel('centimeters')
title('Bottom tank water level vs. time')
    



figure(2)
range = cost.c{1}; % assumes all are controlled to the same range
plot(data.h:data.h:data.h*sim_time,Xlog(1:2:20,:))
hold on
plot([data.h,sim_time*data.h],[Xref-range, Xref-range],'--k')
plot([data.h,sim_time*data.h],[Xref+range, Xref+range],'--k')
axis([data.h sim_time*data.h 0 20]);
xlabel('seconds')
ylabel('centimeters')
title('Top5 tank water level vs. time')

figure(3)
plot(times,Ulog(1:data.nActuators,1:sim_time))
axis([data.h sim_time*data.h 0 20]);
xlabel('seconds')
ylabel('volts')
title('Controller input vs. time')

figure(4)
bar(sum(Qlog,2)/sim_time)
axis([0,21,0,1])
xlabel('sensor number')
ylabel('probability of selection')
title('Probability of selection vs. sensor')


vals = zeros(size(Qlog,1)*size(Qlog,2),2);
k = 0;
for i = 1:size(Qlog,1)
    for j = 1:size(Qlog,2)
        if (Qlog(i,j) == 1)
            k = k + 1;
            vals(k,1) = i;
            vals(k,2) = j;
        end
    end
end

figure(5)
%plot(vec(kron(times,ones(1,20))),vec(Qlog),'k.')
plot(vals(1:k,2),vals(1:k,1),'k.')
axis([0, length(times),0,21])
xlabel('seconds')
ylabel('sensor number')
title('Sensor schedule vs. time')

figure(6)
plot(times,ErrorLog/25)
xlabel('seconds')
ylabel('Error percentage')
title('Error percentage vs. time')

