close all
tmp=textread('st_3_3', '%f','headerlines',1);
tmp=transpose(reshape(tmp,11,[]));

figure(1);
subplot(2,1,1);
hold on;
plot(tmp(:,3)*15.2588/1000000,tmp(:,1),'b');
plot(tmp(:,3)*15.2588/1000000,tmp(:,2),'g');
subplot(2,1,2);
plot(tmp(:,4)*15.2588/1000000,tmp(:,5)*15.2588/1000000,'b*');
hold on;
plot(tmp(:,6)*15.2588/1000000,tmp(:,7)*15.2588/1000000,'r*');
hold on;
plot(tmp(:,8)*15.2588/1000000,tmp(:,9)*15.2588/1000000,'g*');


% iae
fprintf ('iae (before disturbance) = %f\n',tmp(150,10));
fprintf ('iae (end) = %f\n',tmp(end,10));
% number of tx
fprintf ('Number of samples = %f\n',tmp(end,11));


