close all
tmp=textread('xtraSensor_2_2_2_2048', '%f','headerlines',1);
tmp=transpose(reshape(tmp,4,[]));

toms=15.2588/1000000;
figure(1)
subplot(2,1,1);
stem(toms*tmp(:,4),toms*tmp(:,3));
hold on
% PDR
fprintf ('PDR = %f\n',tmp(end,2)/tmp(end,1));
% Average Delay
fprintf ('Average Delay = %f\n',(mean(toms*tmp(:,3))));
min(toms*tmp(:,3))
max(toms*tmp(:,3))

tmp=textread('xtraSensor_2_2_2_2026', '%f','headerlines',1);
tmp=transpose(reshape(tmp,4,[]));

toms=15.2588/1000000;
figure(2)
subplot(2,1,1);
stem(toms*tmp(:,4),toms*tmp(:,3));
hold on
% PDR
fprintf ('PDR = %f\n',tmp(end,2)/tmp(end,1));
% Average Delay
fprintf ('Average Delay = %f\n',(mean(toms*tmp(:,3))));
min(toms*tmp(:,3))
max(toms*tmp(:,3))

% tmp=textread('h_2_1_1853', '%f','headerlines',1);
% tmp=transpose(reshape(tmp,11,[]));

% figure(1);
% 
% %hold on;
% %plot(tmp(:,3)*15.2588/1000000,tmp(:,1),'b');
% %plot(tmp(:,3)*15.2588/1000000,tmp(:,2),'g');
% subplot(2,1,2);
% stem(tmp(:,4)*15.2588/1000000,tmp(:,5)*15.2588/1000000,'b*');
% hold on;
% stem(tmp(:,6)*15.2588/1000000,tmp(:,7)*15.2588/1000000,'r*');
% %hold on;
% %plot(tmp(:,8)*15.2588/1000000,tmp(:,9)*15.2588/1000000,'g*');
% 
% 
% % iae
% fprintf ('iae (before disturbance) = %f\n',tmp(150,10));
% fprintf ('iae (end) = %f\n',tmp(end,10));
% % number of tx
% fprintf ('Number of samples = %f\n',tmp(end,11));