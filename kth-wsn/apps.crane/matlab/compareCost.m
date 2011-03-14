function [ output_args ] = CompareCost()

COST_COMP_LIMIT = 50;
START_REF1 = .25;

TT_files_losses = {'outputs/data_201102102334'; 'sf_output/tt_n0_8_201102102334'};
TT_files_no_losses = {'outputs/data_201102102230';'sf_output/tt_201102102230'};
TT_files_losses_retx = {'outputs/data_201102142209';'sf_output/tt_n1_10_201102142209'};

ET_files_losses = {'outputs/data_201102102341';'sf_output/et_n0_8_201102102341'};
ET_files_no_losses =  {'outputs/data_201102102226'; 'sf_output/et_201102102226'};
ET_files_losses_retx = {'outputs/data_201102141950'; 'sf_output/et_n2_10_201102141950'};

[xET_losses, thetaET_losses, alphaET_losses, betaET_losses, u_xET_losses, u_thetaET_losses, tET_losses, ref1ET_losses, ref2ET_losses, ...
    xTT_losses, thetaTT_losses, alphaTT_losses, betaTT_losses, u_xTT_losses, u_thetaTT_losses, tTT_losses, ref1TT_losses, ref2TT_losses ...
    ] = Plots_TT_ET( TT_files_losses, ET_files_losses);



[xET_no_losses, thetaET_no_losses, alphaET_no_losses, betaET_no_losses, u_xET_no_losses, u_thetaET_no_losses, tET_no_losses, ref1ET_no_losses, ref2ET_no_losses, ...
    xTT_no_losses, thetaTT_no_losses, alphaTT_no_losses, betaTT_no_losses, u_xTT_no_losses, u_thetaTT_no_losses, tTT_no_losses, ref1TT_no_losses, ref2TT_no_losses ...
    ] = Plots_TT_ET( TT_files_no_losses, ET_files_no_losses);


[xET_losses_retx, thetaET_losses_retx, alphaET_losses_retx, betaET_losses_retx, u_xET_losses_retx, u_thetaET_losses_retx, tET_losses_retx, ref1ET_losses_retx, ref2ET_losses_retx, ...
    xTT_losses_retx, thetaTT_losses_retx, alphaTT_losses_retx, betaTT_losses_retx, u_xTT_losses_retx, u_thetaTT_losses_retx, tTT_losses_retx, ref1TT_losses_retx, ref2TT_losses_retx ...
    ] = Plots_TT_ET( TT_files_losses_retx, ET_files_losses_retx);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
count(1) = length(find(ref1ET_losses_retx == START_REF1));
count(2) = length(find(ref1ET_losses == START_REF1));
count(3) = length(find(ref1ET_no_losses == START_REF1));

counter = min(count);

sp(1) = find(ref1ET_losses_retx == START_REF1, 1, 'first');
sp(2) = find(ref1ET_losses == START_REF1, 1, 'first');
sp(3) = find(ref1ET_no_losses == START_REF1, 1, 'first');

for i=1:length(count)
    if (count(i) > counter)
        sp(i) = sp(i) + (count(i) - counter);
    end
end

% get the shortest simulation
simLength = min(length(ref1ET_losses_retx)-sp(1), length(ref1ET_losses)-sp(2));
simLength = min(simLength, length(ref1ET_no_losses)-sp(3));


[tTT_no_losses, alphaTT_no_losses, betaTT_no_losses, thetaTT_no_losses, u_thetaTT_no_losses ...
    u_xTT_no_losses, xTT_no_losses,  ref1TT_no_losses, ref2TT_no_losses ] = ...
    syncSimulation (sp(3), simLength, tTT_no_losses, alphaTT_no_losses, betaTT_no_losses, ...
    thetaTT_no_losses, u_thetaTT_no_losses, u_xTT_no_losses, xTT_no_losses, ref1TT_no_losses, ref2TT_no_losses);
[tET_no_losses, alphaET_no_losses, betaET_no_losses, thetaET_no_losses, u_thetaET_no_losses ...
    u_xET_no_losses,  xET_no_losses, ref1ET_no_losses, ref2ET_no_losses ] = ...
    syncSimulation (sp(3), simLength, tET_no_losses, alphaET_no_losses, betaET_no_losses, ...
    thetaET_no_losses, u_thetaET_no_losses, u_xET_no_losses, xET_no_losses, ref1ET_no_losses, ref2ET_no_losses);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%

[JET_no_losses, JuET_no_losses, J1ET_no_losses, J2ET_no_losses, JRMSET_no_losses] = ...
    getControlCost(xET_no_losses, thetaET_no_losses, alphaET_no_losses, betaET_no_losses, u_xET_no_losses, u_thetaET_no_losses, tET_no_losses, ref1ET_no_losses, ref2ET_no_losses);

[JTT_no_losses, JuTT_no_losses, J1TT_no_losses, J2TT_no_losses, JRMSTT_no_losses] = ...
    getControlCost(xTT_no_losses, thetaTT_no_losses, alphaTT_no_losses, betaTT_no_losses, u_xTT_no_losses, u_thetaTT_no_losses, tTT_no_losses, ref1TT_no_losses, ref2TT_no_losses);

[JET_losses, JuET_losses, J1ET_losses, J2ET_losses, JRMSET_losses] = ...
    getControlCost(xET_losses, thetaET_losses, alphaET_losses, betaET_losses, u_xET_losses, u_thetaET_losses, tET_losses, ref1ET_losses, ref2ET_losses);

[JTT_losses, JuTT_losses, J1TT_losses, J2TT_losses, JRMSTT_losses] = ...
    getControlCost(xTT_losses, thetaTT_losses, alphaTT_losses, betaTT_losses, u_xTT_losses, u_thetaTT_losses, tTT_losses, ref1TT_losses, ref2TT_losses);
[JET_losses_retx, JuET_losses_retx, J1ET_losses_retx, J2ET_losses_retx, JRMSET_losses_retx] = ...
    getControlCost(xET_losses_retx, thetaET_losses_retx, alphaET_losses_retx, betaET_losses_retx, u_xET_losses_retx, u_thetaET_losses_retx, tET_losses_retx, ref1ET_losses_retx, ref2ET_losses_retx);

[JTT_losses_retx, JuTT_losses_retx, J1TT_losses_retx, J2TT_losses_retx, JRMSTT_losses_retx] = ...
    getControlCost(xTT_losses_retx, thetaTT_losses_retx, alphaTT_losses_retx, betaTT_losses_retx, u_xTT_losses_retx, u_thetaTT_losses_retx, tTT_losses_retx, ref1TT_losses_retx, ref2TT_losses_retx);
disp(sprintf('\t[losses ]Control cost ET RMS: %0.5f', JRMSET_losses));
disp(sprintf('\t[No losses ]Control cost TT RMS: %0.5f', JRMSTT_no_losses));
disp(sprintf('\t[losses retx]Control cost ET RMS: %0.5f', JRMSET_losses_retx));


disp(sprintf('\t[losses ]Control cost TT RMS: %0.5f', JRMSTT_losses));
disp(sprintf('\t[No losses ]Control cost TT RMS: %0.5f', JRMSTT_no_losses));
disp(sprintf('\t[losses retx]Control cost TT RMS: %0.5f', JRMSTT_losses_retx));

disp(sprintf('Sum XET losses retx: %0.5f', sum(xET_losses_retx-ref1ET_losses_retx)));
disp(sprintf('Sum XET no losses: %0.5f', sum(xET_no_losses-ref1ET_no_losses)));
disp(sprintf('Sum XET losses: %0.5f', sum(xET_losses-ref1ET_losses)));
disp('.....');
disp(sprintf('Sum XTT losses retx: %0.5f', sum(xTT_losses_retx-ref1TT_losses_retx)));
disp(sprintf('Sum XTT no losses: %0.5f', sum(xTT_no_losses-ref1TT_no_losses)));
disp(sprintf('Sum XTT losses: %0.5f', sum(xTT_losses-ref1TT_losses)));
disp('-----');

disp(sprintf('Sum thetaET losses retx: %0.5f', sum(thetaET_losses_retx-ref2ET_losses_retx)));
disp(sprintf('Sum thetaET no losses: %0.5f', sum(thetaET_no_losses-ref2ET_no_losses)));
disp(sprintf('Sum thetaET losses: %0.5f', sum(thetaET_losses-ref2ET_losses)));
disp('.....');
disp(sprintf('Sum thetaTT losses retx: %0.5f', sum(thetaTT_losses_retx-ref2TT_losses_retx)));
disp(sprintf('Sum thetaTT no losses: %0.5f', sum(thetaTT_no_losses-ref2TT_no_losses)));
disp(sprintf('Sum thetaTT losses: %0.5f', sum(thetaTT_losses-ref2TT_losses)));

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all
% figure;
% hold on;
% title('Refences');
% plot(ref1ET_losses_retx, 'b')
% plot(ref1TT_losses_retx, 'g')
% legend('Losses and retx ET', 'Losses and retx TT');
% hold off;
% figure;
% hold on;
% title('Refences');
% plot(ref1ET_no_losses, 'b')
% plot(ref1TT_no_losses, 'g')
% legend('No losses ET', 'No losses TT');
% hold off;
% figure;
% hold on;
% title('Refences');
% plot(ref1ET_losses, 'b')
% plot(ref1TT_losses, 'g')
% legend('losses ET', 'losses TT');
% hold off;

figure;
hold on;
title('ET'),
plot(ref1ET_losses, 'c')
plot(xET_losses_retx, 'b')
plot(xET_no_losses, 'g')
plot(xET_losses, 'r'),
legend('ref','Losses and retx', 'No Losses', 'Losses');
hold off;


% figure;
% hold on;
% title('ET Refences');
% plot(ref1ET_losses_retx, 'b')
% plot(ref1ET_no_losses, 'g')
% plot(ref1ET_losses, 'r'),
% legend('Losses and retx', 'No Losses', 'Losses');
% hold off;
% 
% figure;
% hold on;
% title('TT Refences');
% plot(ref1TT_losses_retx, 'b')
% plot(ref1TT_no_losses, 'g')
% plot(ref1TT_losses, 'r'),
% legend('Losses and retx', 'No Losses', 'Losses');
% hold off;


figure;
hold on;
title('X - Refences');
plot(xET_losses_retx-ref1ET_losses_retx, 'b')
plot(xET_no_losses-ref1ET_no_losses, 'g')
plot(xET_losses-ref1ET_losses, 'r'),
legend('Losses and retx', 'No Losses', 'Losses');
hold off;

  function [ao, bo, co, do, eo, fo, go, ho, io] ...
            = syncSimulation(sp, endP, a, b, c, d, e, f, g, h, i)
        if sp ==1
            endP = endP - sp;
        end
        ao = a(sp:(sp+endP));
        bo = b(sp:(sp+endP));
        co = c(sp:(sp+endP));
        do = d(sp:(sp+endP));
        eo = e(sp:(sp+endP));
        fo = f(sp:(sp+endP));
        go = g(sp:(sp+endP));
        ho = h(sp:(sp+endP));
        io = i(sp:(sp+endP));
        
    