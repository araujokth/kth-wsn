close all
load ('data_mode2_1tanks_110906_124444');

NUM_BEACONS_TO_STEADY = 100;

for i=1:WTSelf.nWaterTanks
    
    % Process Adjusted Intersampling
    timeSchedulledAdj=WTSelf.logger.timeAllocated;
    timeSchedulledAdj=timeSchedulledAdj(NUM_BEACONS_TO_STEADY:end,i);
    idx=find(timeSchedulledAdj);
    timeSchedulledAdj=timeSchedulledAdj(idx);
    
    tauAdj=timeSchedulledAdj;
    
    tmp2=tauAdj(2:end);
    tmp1=tauAdj(1:end-1);
    
    tauAdj=tmp2-tmp1;
    
    figure(5);
    plot (timeSchedulledAdj(1:end-1),tauAdj,'*')
    
    % Process Non-Adjusted Intersampling
    tau=WTSelf.logger.tau;
    tau=tau(NUM_BEACONS_TO_STEADY:end,i);
    tau=tau(2:end);
    idx=find(tau~=10000);
    idx=idx(2:end);
    tau=tau(idx);
    tau=tau(1:length(tauAdj));
        
    timeScheduled=WTSelf.logger.timeSchedule;
    timeScheduled=timeScheduled(NUM_BEACONS_TO_STEADY:end,i);
    timeScheduled=timeScheduled(idx-1);
    timeScheduled=timeScheduled(1:length(tau));
    
    hold on
    figure(5);
    plot (timeScheduled(1:end),tau,'r*')
    
    legend('adjusted','not adjusted')
    
    figure,
    subplot(211)
    plot (WTSelf.logger.x(:,:,1),'b')
    hold on
    plot (WTSelf.logger.x(:,:,2),'r')
    hold on
    subplot(212)
    plot (timeScheduled(1:end),tau,'r*')
    

end

 figure()
 plot (WTSelf.logger.x(:,:,1),'b')
 hold on
plot (WTSelf.logger.x(:,:,2),'r')
hold on
plot ((WTSelf.logger.u)/273,'g')
figure();
plot ((WTSelf.logger.x10),'b*')
hold on
plot ((WTSelf.logger.x20),'r*')