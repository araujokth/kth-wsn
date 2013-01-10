clc;clear all ;close all;

%%
load  'Node1.txt';
load  'Node2.txt';

figure
subplot(2,1,1)
plot( Node1(:,1),'b','LineWidth',2);
hold on
plot( Node2(:,1),'b:','LineWidth',2);


xlabel('Samples'); ylabel('Water Level');
legend('No breaking','Breaking + CTP')

subplot(2,1,2)
plot( Node1(:,3),'r','LineWidth',2);
hold on
plot( Node2(:,3),'r:','LineWidth',2);

xlabel('Samples'); ylabel('Voltage');



