function plotsPaper(filenameST, filenamePer, final_time)

close all

file = open(filenameST);
dataST = file.data;
file = open(filenamePer);
dataPer = file.data;


% Label rows
assignedBO     = 2;
assignedSlotM1 = 3;
assignedSlotA1 = 4;
assignedSlotM2 = 5;
assignedSlotA2 = 6;
assignedSlotS1 = 7;
assignedSlotS2 = 8;
assignedSlotS3 = 9;
assignedSlotS4 = 10;
assignedSlotS5 = 11;
TAU_K1         = 12;
TAU_K2         = 13;
TAU_K3         = 14;
TAU_K4         = 15;
TAU_K5         = 16;
TAU_K6         = 17;
TAU_K7         = 18;
TAU_K8         = 19;
TAU_K9         = 20;
levelUTank1    = 21;
levelLTank1    = 22;
levelUTank2    = 23;
levelLTank2    = 24;
measurementS1  = 25;
measurementS2  = 26;
measurementS3  = 27;
integrator1    = 28;
integrator2    = 29;
actuation1     = 30;
actuation2     = 31;
ttStart        = 32;
initBit        = 33;
measurementS4  = 34;
measurementS5  = 35;
lateOUT1       = 36;
lateOUT2       = 37;
lateOUT3       = 38;
lateOUT4       = 39;
assignedSlotS6 = 40;
assignedSlotS7 = 41;
levelUTank1_ini= 42;
levelLTank1_ini= 43;
levelUTank2_ini= 44;
levelLTank2_ini= 45;
actuation1_ini = 46;
actuation2_ini = 47;
integrator1_ini= 48;
integrator2_ini= 49;


% Correct for unsigned integer underflow in tank levels

for i=1:size(dataST,2)
    if(dataST(levelUTank1,i)>50)
        dataST(levelUTank1,i)=-((65536/87.36)-dataST(levelUTank1,i));
    end
    if(dataST(levelLTank1,i)>50)
        dataST(levelLTank1,i)=-((65536/87.36)-dataST(levelLTank1,i));
    end
    if(dataST(levelUTank2,i)>50)
        dataST(levelUTank2,i)=-((65536/87.36)-dataST(levelUTank2,i));
    end
    if(dataST(levelLTank2,i)>50)
        dataST(levelLTank2,i)=-((65536/87.36)-dataST(levelLTank2,i));
    end
end
for i=1:size(dataPer,2)
    if(dataPer(levelUTank1,i)>50)
        dataPer(levelUTank1,i)=-((65536/87.36)-dataPer(levelUTank1,i));
    end
    if(dataPer(levelLTank1,i)>50)
        dataPer(levelLTank1,i)=-((65536/87.36)-dataPer(levelLTank1,i));
    end
    if(dataPer(levelUTank2,i)>50)
        dataPer(levelUTank2,i)=-((65536/87.36)-dataPer(levelUTank2,i));
    end
    if(dataPer(levelLTank2,i)>50)
        dataPer(levelLTank2,i)=-((65536/87.36)-dataPer(levelLTank2,i));
    end
end

%



last_indexST=find((dataST(ttStart,:)>=final_time), 1);
last_indexST=last_indexST-1;

last_indexPer=find((dataPer(ttStart,:)>=final_time), 1);
last_indexPer=last_indexPer-1;


% Plot tank 1
figure
subplot(2,2,1);
title('Tank System 1');
hold
plot(dataST(ttStart,2:last_indexST), dataST(levelUTank1,2:last_indexST)+dataST(levelUTank1_ini,2:last_indexST), 'c-');
plot(dataST(ttStart,2:last_indexST), dataST(levelLTank1,2:last_indexST)+dataST(levelLTank1_ini,2:last_indexST), 'b-');
plot(dataST(ttStart,2:last_indexST), dataST(actuation1,2:last_indexST), 'r-');
plot([dataST(ttStart,1) dataST(ttStart,last_indexST)], [10 10], 'g-.', [dataST(ttStart,1) dataST(ttStart,last_indexST)], [10.25 10.25], 'g-.', [dataST(ttStart,1) dataST(ttStart,last_indexST)], [9.75 9.75], 'g-.');
hold
xlim([dataST(ttStart, 1) dataST(ttStart, last_indexST)]);
ylim([4 14]);

subplot(2,2,3);
plot(dataST(ttStart,2:last_indexST), dataST(integrator1,2:last_indexST));
xlim([dataST(ttStart, 1) dataST(ttStart, last_indexST)]);
ylim([-150 0]);

% Plot tank 2
%figure
subplot(2,2,2);
title('Tank System 2');
hold
plot(dataST(ttStart,2:last_indexST), dataST(levelUTank2,2:last_indexST)+dataST(levelUTank2_ini,2:last_indexST), 'c-');
plot(dataST(ttStart,2:last_indexST), dataST(levelLTank2,2:last_indexST)+dataST(levelLTank2_ini,2:last_indexST), 'b-');
plot(dataST(ttStart,2:last_indexST), dataST(actuation2,2:last_indexST), 'r-');
plot([dataST(ttStart,1) dataST(ttStart,last_indexST)], [10 10], 'g-.', [dataST(ttStart,1) dataST(ttStart,last_indexST)], [10.25 10.25], 'g-.', [dataST(ttStart,1) dataST(ttStart,last_indexST)], [9.75 9.75], 'g-.');
hold
xlim([dataST(ttStart, 1) dataST(ttStart, last_indexST)]);
ylim([4 14]);
h=gcf;
set(h, 'Position', [0 0 1400 800]);
legend('Upper tank level', 'Lower tank level', 'Actuation level', 'Reference','Location','Best');

subplot(2,2,4);
plot(dataST(ttStart,2:last_indexST), dataST(integrator2,2:last_indexST));
legend('Integrated error','Location','Best');
xlim([dataST(ttStart, 1) dataST(ttStart, last_indexST)]);
ylim([-150 0]);


% Plot tank 1
figure
% subplot(2,2,1);
hold
plot(dataST(ttStart,2:last_indexST*2), dataST(levelUTank1,2:last_indexST*2)+dataST(levelUTank1_ini,2:last_indexST*2), 'c-','LineWidth',3);
plot(dataST(ttStart,2:last_indexST*2), dataST(levelLTank1,2:last_indexST*2)+dataST(levelLTank1_ini,2:last_indexST*2), 'b-','LineWidth',3);
plot(dataST(ttStart,2:last_indexST*2), dataST(actuation1,2:last_indexST*2), 'r--','LineWidth',3);
plot([dataST(ttStart,1) dataST(ttStart,last_indexST*2)], [10 10], 'g-.', [dataST(ttStart,1) dataST(ttStart,last_indexST*2)], [10.25 10.25], 'g-.', [dataST(ttStart,1) dataST(ttStart,last_indexST*2)], [9.75 9.75], 'g-.');
hold
xlim([400 dataST(ttStart, last_indexST*2)]);
ylim([0 30]);
h=gca;
set(h, 'FontSize', 30);
xlabel('Time (s)');
h=gcf;
set(h, 'Position', [0 0 1400 800]);
legend('Upper tank level (cm)', 'Lower tank level (cm)', 'Actuation level (V)', 'Reference','Location','Best');

% subplot(2,2,3);
% plot(dataST(ttStart,2:last_indexST*2), dataST(integrator1,2:last_indexST*2));
% xlim([400 dataST(ttStart, last_indexST*2)]);
% ylim([-400 70]);

% Plot tank 2
figure
% subplot(2,2,2);
hold
plot(dataST(ttStart,2:last_indexST*2), dataST(levelUTank2,2:last_indexST*2)+dataST(levelUTank2_ini,2:last_indexST*2), 'c-','LineWidth',3);
plot(dataST(ttStart,2:last_indexST*2), dataST(levelLTank2,2:last_indexST*2)+dataST(levelLTank2_ini,2:last_indexST*2), 'b-','LineWidth',3);
plot(dataST(ttStart,2:last_indexST*2), dataST(actuation2,2:last_indexST*2), 'r--','LineWidth',3);
plot([dataST(ttStart,1) dataST(ttStart,last_indexST*2)], [10 10], 'g-.', [dataST(ttStart,1) dataST(ttStart,last_indexST*2)], [10.25 10.25], 'g-.', [dataST(ttStart,1) dataST(ttStart,last_indexST*2)], [9.75 9.75], 'g-.');
hold
xlim([400 dataST(ttStart, last_indexST*2)]);
ylim([0 30]);
h=gca;
set(h, 'FontSize', 30);
xlabel('Time (s)');
h=gcf;
set(h, 'Position', [0 0 1400 800]);
legend('Upper tank level (cm)', 'Lower tank level (cm)', 'Actuation level (V)', 'Reference','Location','Best');

% subplot(2,2,4);
% plot(dataST(ttStart,2:last_indexST*2), dataST(integrator2,2:last_indexST*2));
% legend('Integrated error','Location','Best');
% xlim([400 dataST(ttStart, last_indexST*2)]);
% ylim([-400 70]);


% Plot tank 1
figure
subplot(2,2,1);
title('Tank System 1');
hold
plot(dataPer(ttStart,2:last_indexPer), dataPer(levelUTank1,2:last_indexPer)+dataPer(levelUTank1_ini,2:last_indexPer), 'c-');
plot(dataPer(ttStart,2:last_indexPer), dataPer(levelLTank1,2:last_indexPer)+dataPer(levelLTank1_ini,2:last_indexPer), 'b-');
plot(dataPer(ttStart,2:last_indexPer), dataPer(actuation1,2:last_indexPer), 'r-');
plot([dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [10 10], 'g-.', [dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [10.25 10.25], 'g-.', [dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [9.75 9.75], 'g-.');
hold
xlim([dataPer(ttStart, 1) dataPer(ttStart, last_indexPer)]);
ylim([4 14]);
subplot(2,2,3);
plot(dataPer(ttStart,2:last_indexPer), dataPer(integrator1,2:last_indexPer));
xlim([dataPer(ttStart,1) dataPer(ttStart, last_indexPer)]);
ylim([-150 0]);
% Plot tank 2
%figure
subplot(2,2,2);
title('Tank System 2');
hold
plot(dataPer(ttStart,2:last_indexPer), dataPer(levelUTank2,2:last_indexPer)+dataPer(levelUTank2_ini,2:last_indexPer), 'c-');
plot(dataPer(ttStart,2:last_indexPer), dataPer(levelLTank2,2:last_indexPer)+dataPer(levelLTank2_ini,2:last_indexPer), 'b-');
plot(dataPer(ttStart,2:last_indexPer), dataPer(actuation2,2:last_indexPer), 'r-');
plot([dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [10 10], 'g-.', [dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [10.25 10.25], 'g-.', [dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [9.75 9.75], 'g-.');
hold
xlim([dataPer(ttStart, 1) dataPer(ttStart, last_indexPer)]);
ylim([4 14]);
h=gcf;
set(h, 'Position', [0 0 1400 800]);
legend('Upper tank level (cm)', 'Lower tank level (cm)', 'Actuation level (V)', 'Reference','Location','Best');

subplot(2,2,4);
plot(dataPer(ttStart,2:last_indexPer), dataPer(integrator2,2:last_indexPer));
legend('Integrated error','Location','Best');
xlim([dataPer(ttStart, 1) dataPer(ttStart, last_indexPer)]);
ylim([-150 0]);


% Plot tank 1
figure
% subplot(2,2,1);
hold
plot(dataPer(ttStart,2:last_indexPer), dataPer(levelLTank1,2:last_indexPer)+dataPer(levelLTank1_ini,2:last_indexPer), 'b-','LineWidth',3);
plot(dataST(ttStart,2:last_indexST), dataST(levelLTank1,2:last_indexST)+dataST(levelLTank1_ini,2:last_indexST), 'r--','LineWidth',3);
plot([dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [10 10], 'g-.', [dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [10.25 10.25], 'g-.', [dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [9.75 9.75], 'g-.');
hold
xlim([dataPer(ttStart, 1) dataPer(ttStart, last_indexPer)]);
h=gca;
set(h, 'FontSize', 30);
xlabel('Time (s)');
ylim([4 12]);
ylabel('Tank water level (cm)');
legend('Lower tank level (P)', 'Lower tank level (ST)', 'Reference','Location','Best');
h=gcf;
set(h, 'Position', [0 0 1400 800]);
figure
% subplot(2,2,3);
hold
plot(dataPer(ttStart,2:last_indexPer), dataPer(actuation1,2:last_indexPer), 'b-','LineWidth',3);
plot(dataST(ttStart,2:last_indexST), dataST(actuation1,2:last_indexST), 'r--','LineWidth',3);
hold
xlim([dataPer(ttStart, 1) dataPer(ttStart, last_indexPer)]);
h=gca;
set(h, 'FontSize', 30);
xlabel('Time (s)');
ylim([4 6.5]);
ylabel('Control input u (V)');
h=gca;
set(h, 'FontSize', 30);
legend('Actuation level (P)','Actuation level (ST)','Location','Best');
h=gcf;
set(h, 'Position', [0 0 1400 800]);
% Plot tank 2
figure
% subplot(2,2,2);
hold
plot(dataPer(ttStart,2:last_indexPer), dataPer(levelLTank2,2:last_indexPer)+dataPer(levelLTank2_ini,2:last_indexPer), 'b-','LineWidth',3);
plot(dataST(ttStart,2:last_indexST), dataST(levelLTank2,2:last_indexST)+dataST(levelLTank2_ini,2:last_indexST), 'r--','LineWidth',3);
plot([dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [10 10], 'g-.', [dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [10.25 10.25], 'g-.', [dataPer(ttStart,1) dataPer(ttStart,last_indexPer)], [9.75 9.75], 'g-.');
hold
xlim([dataPer(ttStart, 1) dataPer(ttStart, last_indexPer)]);
h=gca;
set(h, 'FontSize', 30);
xlabel('Time (s)');
ylim([4 12]);
ylabel('Tank water level (cm)');
h=gca;
set(h, 'FontSize', 30);
legend('Lower tank level (P)', 'Lower tank level (ST)', 'Reference','Location','Best');
h=gcf;
set(h, 'Position', [0 0 1400 800]);
figure
% subplot(2,2,4);
hold
plot(dataPer(ttStart,2:last_indexPer), dataPer(actuation2,2:last_indexPer), 'b-','LineWidth',3);
plot(dataST(ttStart,2:last_indexST), dataST(actuation2,2:last_indexST), 'r--','LineWidth',3);
hold
xlim([dataPer(ttStart, 1) dataPer(ttStart, last_indexPer)]);
h=gca;
set(h, 'FontSize', 30);
xlabel('Time (s)');
ylim([4 6.5]);
ylabel('Control input u (V)');
h=gca;
set(h, 'FontSize', 30);
legend('Actuation level (P)','Actuation level (ST)','Location','Best');
h=gcf;
set(h, 'Position', [0 0 1400 800]);
figure
subplot(2,1,1);
h=gca;
set(h, 'FontSize', 30);
h=gcf;
set(h, 'Position', [0 0 1400 800]);
tank1 = dataST([12 21 22 28 30 32],find(dataST(12,2:last_indexST)<10000)+1);
tank1 = [tank1; dataST(assignedSlotM1,find(dataST(12,2:last_indexST)<10000))];
tank2 = dataST([13 23 24 29 31 32],find(dataST(13,2:last_indexST)<10000)+1);
tank2 = [tank2; dataST(assignedSlotM2,find(dataST(13,2:last_indexST)<10000))];

symbol_time=0.000015259 % time in seconds
deltaCAP=0.2637 % time in seconds

for i=1:size(tank1,2)-1
    if i==1
        tank1(:,i)
    end
    if tank1(7,i)==0
        i
        tank1(7,i)=9;
    end
    tank1(8,i) = (tank1(6,i+1)+deltaCAP+(tank1(7,i+1)-9)*1920*symbol_time)-(tank1(6,i)+deltaCAP+(tank1(7,i)-9)*1920*symbol_time);
end
for i=1:size(tank2,2)-1
    if tank2(7,i)==0
        i
        tank2(7,i)=11;
    end
    tank2(8,i) = (tank2(6,i+1)+deltaCAP+(tank2(7,i+1)-9)*1920*symbol_time)-(tank2(6,i)+deltaCAP+(tank2(7,i)-9)*1920*symbol_time);
end
max(tank2(8,:))
min(tank2(8,1:end-1))
max(tank1(8,:))
min(tank1(8,1:end-1))

hold
stem(tank1(6,:), tank1(1,:), 'rx','LineWidth',3, 'MarkerSize', 18);
stem(tank2(6,:), tank2(1,:), 'bo','LineWidth',3, 'MarkerSize', 18);
hold
legend('Tank System 1','Tank System 2','Location','Best');
xlim([dataST(ttStart, 1) dataST(ttStart, last_indexST)-5]);
xlabel('Time (s)');
ylim([3 11]);
ylabel('$\tau_i$ (s)','Interpreter', 'latex');

no_samples_tank_1_st = size(tank1, 2);
no_samples_tank_2_st = size(tank2, 2);

%figure
subplot(2,1,2);
h=gca;
set(h, 'FontSize', 30);
h=gcf;
set(h, 'Position', [0 0 1400 800]);
hold
stem(tank1(6,:), tank1(8,:), 'rx','LineWidth',3, 'MarkerSize', 18);
stem(tank2(6,:), tank2(8,:), 'bo','LineWidth',3, 'MarkerSize', 18);
hold
legend('Tank System 1','Tank System 2','Location','Best');
xlim([dataST(ttStart, 1) dataST(ttStart, last_indexST)-5]);
xlabel('Time (s)');
ylim([3 11]);
ylabel('$\hat{\tau_i}$ (s)', 'Interpreter', 'latex');

figure
h=gca;
set(h, 'FontSize', 15);
h=gcf;
set(h, 'Position', [0 0 1400 800]);
tank1 = dataST([12 21 22 28 30 32],find(dataST(12,2:last_indexST*2)<10000)+1);
tank2 = dataST([13 23 24 29 31 32],find(dataST(13,2:last_indexST*2)<10000)+1);
hold
stem(tank1(6,:), tank1(1,:), 'rx');
stem(tank2(6,:), tank2(1,:), 'bo');
hold
legend('\tau_k Tank System 1','\tau_k Tank System 2','Location','Best');
xlim([400 dataST(ttStart, last_indexST*2)]);
ylim([0 12]);


figure
h=gca;
set(h, 'FontSize', 15);
h=gcf;
set(h, 'Position', [0 0 1400 800]);
tank1 = dataST([12 21 22 28 30 32],find(dataST(12,2:last_indexST)<10000)+1);
tank2 = dataST([13 23 24 29 31 32],find(dataST(13,2:last_indexST)<10000)+1);
hold
stem(tank1(6,:), tank1(1,:).*0.9375, 'rx');
stem(tank2(6,:), tank2(1,:).*0.9375, 'bo');
hold
legend('\tau_k Tank System 1','\tau_k Tank System 2','Location','Best');
xlim([dataST(ttStart, 1) dataST(ttStart, last_indexST)]);
ylim([0 12]);

figure
h=gca;
set(h, 'FontSize', 15);
h=gcf;
set(h, 'Position', [0 0 1400 800]);
tank1 = dataST([12 21 22 28 30 32],find(dataST(12,2:last_indexST*2)<10000)+1);
tank2 = dataST([13 23 24 29 31 32],find(dataST(13,2:last_indexST*2)<10000)+1);
hold
stem(tank1(6,:), tank1(1,:).*0.9375, 'rx');
stem(tank2(6,:), tank2(1,:).*0.9375, 'bo');
hold
legend('\tau_k Tank System 1','\tau_k Tank System 2','Location','Best');
xlim([400 dataST(ttStart, last_indexST*2)]);
ylim([0 12]);


% Plot success for the dummy sensors' communication

figure

subplot(3,1,1);
communicationS1=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS1,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==5);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS1=sum([communicationS1; success; failure]);
stem(dataST(ttStart,find(communicationS1~=0)), communicationS1(find(communicationS1~=0)), 'ro');
hold

%subplot(9,1,2);
communicationS2=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS2,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==6);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS2=sum([communicationS2; success; failure]);
stem(dataST(ttStart,find(communicationS2~=0)), communicationS2(find(communicationS2~=0)), 'ro');


%subplot(9,1,3);
communicationS3=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS3,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==7);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS3=sum([communicationS3; success; failure]);
stem(dataST(ttStart,find(communicationS3~=0)), communicationS3(find(communicationS3~=0)), 'ro');


%subplot(9,1,4);
communicationS4=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS4,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==8);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS4=sum([communicationS4; success; failure]);
stem(dataST(ttStart,find(communicationS4~=0)), communicationS4(find(communicationS4~=0)), 'ro');


%subplot(9,1,5);
communicationS5=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS5,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==9);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS5=sum([communicationS5; success; failure]);
stem(dataST(ttStart,find(communicationS5~=0)), communicationS5(find(communicationS5~=0)), 'ro');


%subplot(9,1,6);
communicationS6=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS6,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==10);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS6=sum([communicationS6; success; failure]);
stem(dataST(ttStart,find(communicationS6~=0)), communicationS6(find(communicationS6~=0)), 'ro');


%subplot(9,1,7);
communicationS7=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS7,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==11);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS7=sum([communicationS7; success; failure]);
stem(dataST(ttStart,find(communicationS7~=0)), communicationS7(find(communicationS7~=0)), 'ro');
hold
xlim([dataST(ttStart,1) 190]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [-1 0 1]);
ylabel('Queue Soft Sensors');
xlabel('Time (s)');

subplot(3,1,2);
communicationM1=zeros(1, size(dataST,2));
success = (dataST(assignedSlotM1,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==1);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationM1=sum([communicationM1; success; failure]);
stem(dataST(ttStart,find(communicationM1~=0)), communicationM1(find(communicationM1~=0)), 'go');
xlim([dataST(ttStart,1) 190]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [0 1]);
ylabel('Tank System 1');
xlabel('Time (s)');


subplot(3,1,3);
communicationM2=zeros(1, size(dataST,2));
success = (dataST(assignedSlotM2,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==3);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationM2=sum([communicationM2; success; failure]);
stem(dataST(ttStart,find(communicationM2~=0)), communicationM2(find(communicationM2~=0)));
xlim([dataST(ttStart,1) 190]);
% ylim([-1.5 1.5]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [0 1]);
ylabel('Tank System 2');
xlabel('Time (s)');
h=gcf;
set(h, 'Position', [0 0 1400 800]);


% Plot success for the dummy sensors' communication

figure

subplot(3,1,1);
communicationS1=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotS1,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==5);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS1=sum([communicationS1; success; failure]);
stem(dataPer(ttStart,find(communicationS1~=0)), communicationS1(find(communicationS1~=0)),'ro');
hold

%subplot(9,1,2);
communicationS2=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotS2,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==6);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS2=sum([communicationS2; success; failure]);
stem(dataPer(ttStart,find(communicationS2~=0)), communicationS2(find(communicationS2~=0)),'ro');


%subplot(9,1,3);
communicationS3=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotS3,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==7);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS3=sum([communicationS3; success; failure]);
stem(dataPer(ttStart,find(communicationS3~=0)), communicationS3(find(communicationS3~=0)),'ro');


%subplot(9,1,4);
communicationS4=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotS4,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==8);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS4=sum([communicationS4; success; failure]);
stem(dataPer(ttStart,find(communicationS4~=0)), communicationS4(find(communicationS4~=0)),'ro');


%subplot(9,1,5);
communicationS5=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotS5,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==9);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS5=sum([communicationS5; success; failure]);
stem(dataPer(ttStart,find(communicationS5~=0)), communicationS5(find(communicationS5~=0)),'ro');


%subplot(9,1,6);
communicationS6=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotS6,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==10);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS6=sum([communicationS6; success; failure]);
stem(dataPer(ttStart,find(communicationS6~=0)), communicationS6(find(communicationS6~=0)),'ro');


%subplot(9,1,7);
communicationS7=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotS7,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==11);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS7=sum([communicationS7; success; failure]);
stem(dataPer(ttStart,find(communicationS7~=0)), communicationS7(find(communicationS7~=0)),'ro');
hold
xlim([dataPer(ttStart,1) 190]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [-1 0 1]);
ylabel('Queue Soft Sensors');
xlabel('Time (s)');

subplot(3,1,2);
communicationM1=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotM1,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==1);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationM1=sum([communicationM1; success; failure]);
stem(dataPer(ttStart,find(communicationM1~=0)), communicationM1(find(communicationM1~=0)), 'go');
xlim([dataPer(ttStart,1) 190]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [0 1]);
ylabel('Tank System 1');
xlabel('Time (s)');


subplot(3,1,3);
communicationM2=zeros(1, size(dataPer,2));
success = (dataPer(assignedSlotM2,:)~=16);
success = [0 success(1:end-1)];
failure = (dataPer(lateOUT1:lateOUT4, :)==3);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationM2=sum([communicationM2; success; failure]);
stem(dataPer(ttStart,find(communicationM2~=0)), communicationM2(find(communicationM2~=0)));
xlim([dataPer(ttStart,1) 190]);
% ylim([-1.5 1.5]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [0 1]);
ylabel('Tank System 2');
xlabel('Time (s)');
h=gcf;
set(h, 'Position', [0 0 1400 800]);

figure

subplot(3,1,1);
communicationS1=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS1,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==5);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS1=sum([communicationS1; success; failure]);
stem(dataST(ttStart,find(communicationS1~=0)), communicationS1(find(communicationS1~=0)), 'ro');
hold

%subplot(9,1,2);
communicationS2=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS2,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==6);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS2=sum([communicationS2; success; failure]);
stem(dataST(ttStart,find(communicationS2~=0)), communicationS2(find(communicationS2~=0)), 'ro');


%subplot(9,1,3);
communicationS3=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS3,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==7);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS3=sum([communicationS3; success; failure]);
stem(dataST(ttStart,find(communicationS3~=0)), communicationS3(find(communicationS3~=0)), 'ro');


%subplot(9,1,4);
communicationS4=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS4,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==8);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS4=sum([communicationS4; success; failure]);
stem(dataST(ttStart,find(communicationS4~=0)), communicationS4(find(communicationS4~=0)), 'ro');


%subplot(9,1,5);
communicationS5=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS5,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==9);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS5=sum([communicationS5; success; failure]);
stem(dataST(ttStart,find(communicationS5~=0)), communicationS5(find(communicationS5~=0)), 'ro');


%subplot(9,1,6);
communicationS6=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS6,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==10);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS6=sum([communicationS6; success; failure]);
stem(dataST(ttStart,find(communicationS6~=0)), communicationS6(find(communicationS6~=0)), 'ro');


%subplot(9,1,7);
communicationS7=zeros(1, size(dataST,2));
success = (dataST(assignedSlotS7,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==11);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationS7=sum([communicationS7; success; failure]);
stem(dataST(ttStart,find(communicationS7~=0)), communicationS7(find(communicationS7~=0)), 'ro');
hold
xlim([400 dataST(ttStart, last_indexST*2)]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [-1 0 1]);
ylabel('Queue Soft Sensors');
xlabel('Time (s)');

subplot(3,1,2);
communicationM1=zeros(1, size(dataST,2));
success = (dataST(assignedSlotM1,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==1);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationM1=sum([communicationM1; success; failure]);
stem(dataST(ttStart,find(communicationM1~=0)), communicationM1(find(communicationM1~=0)), 'go');
xlim([400 dataST(ttStart, last_indexST*2)]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [0 1]);
ylabel('Tank System 1');
xlabel('Time (s)');


subplot(3,1,3);
communicationM2=zeros(1, size(dataST,2));
success = (dataST(assignedSlotM2,:)~=16);
success = [0 success(1:end-1)];
failure = (dataST(lateOUT1:lateOUT4, :)==3);
failure = [zeros(4,1) (-1).*failure(:,1:end-1)];
communicationM2=sum([communicationM2; success; failure]);
stem(dataST(ttStart,find(communicationM2~=0)), communicationM2(find(communicationM2~=0)));
xlim([400 dataST(ttStart, last_indexST*2)]);
% ylim([-1.5 1.5]);
h=gca;
set(h, 'FontSize', 23, 'YTick', [0 1]);
ylabel('Tank System 2');
xlabel('Time (s)');
h=gcf;
set(h, 'Position', [0 0 1400 800]);


% CALCULATE SETTLING TIME

Tset_candidate1 = find(~((dataPer(levelLTank1, 1:last_indexPer)+dataPer(levelLTank1_ini, 1:last_indexPer)>=9.75) .* (dataPer(levelLTank1, 1:last_indexPer)+dataPer(levelLTank1_ini, 1:last_indexPer)<=10.25)), 1, 'last');
found1 = 1;
if Tset_candidate1 >= last_indexPer
    found1 = 0;
end

Tset_candidate2 = find(~((dataPer(levelLTank2, 1:last_indexPer)+dataPer(levelLTank2_ini, 1:last_indexPer)>=9.75) .* (dataPer(levelLTank2, 1:last_indexPer)+dataPer(levelLTank2_ini, 1:last_indexPer)<=10.25)), 1, 'last');
found2 = 1;
if Tset_candidate2 >= last_indexPer
    found2 = 0;
end


fprintf('\n%%%%%% Settling time - P\n');
if found1 == 1
    fprintf('Settling time in Tank 1: %f s\t\t\t\t\t(+- 5%%)\n', dataPer(ttStart, Tset_candidate1) - dataPer(ttStart, 2));
else
    fprintf('Water level in Tank 1 did not settle within %u s\t\t\t(+- 5%%)\n', final_time);
end
if found2 == 1
    fprintf('Settling time in Tank 2: %f s\t\t\t\t\t(+- 5%%)\n', dataPer(ttStart, Tset_candidate2) - dataPer(ttStart, 2));
else
    fprintf('Water level in Tank 2 did not settle within %u s\t\t\t(+- 5%%)\n', final_time);
end

Tset_candidate1 = find(~((dataST(levelLTank1, 1:last_indexST)+dataST(levelLTank1_ini, 1:last_indexST)>=9.75) .* (dataST(levelLTank1, 1:last_indexST)+dataST(levelLTank1_ini, 1:last_indexST)<=10.25)), 1, 'last');
found1 = 1;
if Tset_candidate1 >= last_indexST
    found1 = 0;
end

Tset_candidate2 = find(~((dataST(levelLTank2, 1:last_indexST)+dataST(levelLTank2_ini, 1:last_indexST)>=9.75) .* (dataST(levelLTank2, 1:last_indexST)+dataST(levelLTank2_ini, 1:last_indexST)<=10.25)), 1, 'last');
found2 = 1;
if Tset_candidate2 >= last_indexST
    found2 = 0;
end


fprintf('\n%%%%%% Settling time - ST\n');
if found1 == 1
    fprintf('Settling time in Tank 1: %f s\t\t\t\t\t(+- 5%%)\n', dataST(ttStart, Tset_candidate1) - dataST(ttStart, 2));
else
    fprintf('Water level in Tank 1 did not settle within %u s\t\t\t(+- 5%%)\n', final_time);
end
if found2 == 1
    fprintf('Settling time in Tank 2: %f s\t\t\t\t\t(+- 5%%)\n', dataST(ttStart, Tset_candidate2) - dataST(ttStart, 2));
else
    fprintf('Water level in Tank 2 did not settle within %u s\t\t\t(+- 5%%)\n', final_time);
end


% CALCULATE ENERGY CONSUMPTION

t_beacon=0.003; %s
t_slot=0.0073286;
t_inactive=0.8146;

t_beacon = t_beacon / (60*60); %h
t_slot = t_slot / (60*60);
t_inactive = t_inactive / (60*60);

curr_beacon=22.8; %mA
curr_slot_tx=21.7;
curr_slot_nd=2.4;
curr_inactive=0.040;

t_bi = t_beacon+16*t_slot+t_inactive;
pow_bi_tx = curr_beacon*t_beacon + 15*t_slot*curr_slot_nd + t_slot*curr_slot_tx + t_inactive * curr_inactive;%mAh
pow_bi_nd =  curr_beacon*t_beacon + 9*t_slot*curr_slot_nd + 7*t_slot*curr_inactive + t_inactive * curr_inactive;


final_time-dataST(ttStart,1)
total_beacon_intervals = floor(((final_time-dataST(ttStart,1))/(60*60))/t_bi)

no_samples_tank_1_st
no_samples_tank_2_st
no_samples_tank_1_p = total_beacon_intervals
no_samples_tank_2_p = total_beacon_intervals

tot_pow_tank1_st = (total_beacon_intervals - no_samples_tank_1_st) * pow_bi_nd + no_samples_tank_1_st * pow_bi_tx%mAh
tot_pow_tank1_p = no_samples_tank_1_p * pow_bi_tx
tot_pow_tank2_st = (total_beacon_intervals - no_samples_tank_2_st) * pow_bi_nd + no_samples_tank_2_st * pow_bi_tx
tot_pow_tank2_p = no_samples_tank_2_p * pow_bi_tx

battery_capacity = 2900;%mAh

total_time = total_beacon_intervals * t_bi%h

n_tank1_st = battery_capacity / tot_pow_tank1_st;
battery_life_tank1_st = n_tank1_st * total_time;%h
n_tank2_st = battery_capacity / tot_pow_tank2_st;
battery_life_tank2_st = n_tank2_st * total_time;
n_tank1_p = battery_capacity / tot_pow_tank1_p;
battery_life_tank1_p = n_tank1_p * total_time;
n_tank2_p = battery_capacity / tot_pow_tank2_p;
battery_life_tank2_p = n_tank2_p * total_time;

fprintf('\n%%%%%% Battery life\n');
fprintf('Battery capacity: %u mAh\n', battery_capacity);
fprintf('\nP:\n');
fprintf('Battery life for ADC node on Tank system 1: %f days\n', battery_life_tank1_p/24);
fprintf('Battery life for ADC node on Tank system 2: %f days\n', battery_life_tank2_p/24);
fprintf('\nST:\n');
fprintf('Battery life for ADC node on Tank system 1: %f days  -  %f %% longer than periodic!\n', battery_life_tank1_st/24, (battery_life_tank1_st/battery_life_tank1_p - 1)*100);
fprintf('Battery life for ADC node on Tank system 2: %f days  -  %f %% longer than periodic!\n', battery_life_tank2_st/24, (battery_life_tank2_st/battery_life_tank2_p - 1)*100);
