function [ output_args ] = CompareCost()

COST_COMP_LIMIT = 50;
START_REF1 = 25;

TT_files_losses = {'outputs/data_201102102334'; 'sf_output/tt_n0_8_201102102334'};
ET_files_losses = {'outputs/data_201102102341';'sf_output/et_n0_8_201102102341'};

TT_files_no_losses = {'outputs/data_201102102230';'sf_output/tt_201102102230'};
ET_files_no_losses =  {'outputs/data_201102102226'; 'sf_output/et_201102102226'};

TT_files_losses_retx = {'outputs/data_201102142209';'sf_output/tt_n1_10_201102142209'};
ET_files_losses_retx = {'outputs/data_201102141950'; 'sf_output/et_n2_10_201102141950'};


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Load the data

%% Data for the periodic control
[events_intervalTT_losses, tTT_losses, alphaTT_losses, betaTT_losses, thetaTT_losses, u_thetaTT_losses ...
    u_xTT_losses, u_zTT_losses, xTT_losses, zTT_losses, ref1TT_losses, ref2TT_losses, timestampTT_losses ] = ...
    loadLabViewDataFile(TT_files_losses{1});

%% Data for the event triggered control
[events_intervalET_losses, tET_losses, alphaET_losses, betaET_losses, thetaET_losses, u_thetaET_losses ...
    u_xET_losses, u_zET_losses, xET_losses, zET_losses, ref1ET_losses, ref2ET_losses,timestampET_losses ] = ...
    loadLabViewDataFile(ET_files_losses{1});

tET_losses = tET_losses - tET_losses(1);
tTT_losses = tTT_losses - tTT_losses(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control cost computation
costEndET_losses = find(tET_losses >= COST_COMP_LIMIT, 1, 'first');
sp = find(ref1TT_losses  == START_REF1);

[events_intervalET_losses, tET_losses, alphaET_losses, betaET_losses, thetaET_losses, u_thetaET_losses ...
    u_xET_losses, u_zET_losses, xET_losses, zET_losses, ref1ET_losses, ref2ET_losses, timestampET_losses ] = ...
    syncSimulation (sp, costEndET_losses, events_intervalET_losses, tET_losses, alphaET_losses, betaET_losses, ...
    thetaET_losses, u_thetaET_losses, u_xET_losses, u_zET_losses, xET_losses, zET_losses, ref1ET_losses, ref2ET_losses, timestampET_losses);

costEndTT_losses = find(tTT_losses >= COST_COMP_LIMIT, 1, 'first');
[events_intervalTT_losses, tTT_losses, alphaTT_losses, betaTT_losses, thetaTT_losses, u_thetaTT_losses ...
    u_xTT_losses, u_zTT_losses, xTT_losses, zTT_losses, ref1TT_losses, ref2TT_losses, timestampTT_losses ] = ...
    syncSimulation (sp, costEndTT_losses, events_intervalTT_losses, tTT_losses, alphaTT_losses, betaTT_losses, ...
    thetaTT_losses, u_thetaTT_losses, u_xTT_losses, u_zTT_losses, xTT_losses, zTT_losses, ref1TT_losses, ref2TT_losses, timestampTT_losses);

[JET_losses, JuET_losses, J1ET_losses, J2ET_losses, JRMSET_losses] = ...
    getControlCost(xET_losses, thetaET_losses, alphaET_losses, betaET_losses, u_xET_losses, u_thetaET_losses, tET_losses, ref1ET_losses, ref2ET_losses);

[JTT_losses, JuTT_losses, J1TT_losses, J2TT_losses, JRMSTT_losses] = ...
    getControlCost(xTT_losses, thetaTT_losses, alphaTT_losses, betaTT_losses, u_xTT_losses, u_thetaTT_losses, tTT_losses, ref1TT_losses, ref2TT_losses);


%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Load the data

%% Data for the periodic control
[events_intervalTT_no_losses, tTT_no_losses, alphaTT_no_losses, betaTT_no_losses, thetaTT_no_losses, u_thetaTT_no_losses ...
    u_xTT_no_losses, u_zTT_no_losses, xTT_no_losses, zTT_no_losses, ref1TT_no_losses, ref2TT_no_losses, timestampTT_no_losses ] = ...
    loadLabViewDataFile(TT_files_no_losses{1});


%% Data for the event triggered control
[events_intervalET_no_losses, tET_no_losses, alphaET_no_losses, betaET_no_losses, thetaET_no_losses, u_thetaET_no_losses ...
    u_xET_no_losses, u_zET_no_losses, xET_no_losses, zET_no_losses, ref1ET_no_losses, ref2ET_no_losses,timestampET_no_losses ] = ...
    loadLabViewDataFile(ET_files_no_losses{1});

tET_no_losses = tET_no_losses - tET_no_losses(1);
tTT_no_losses = tTT_no_losses - tTT_no_losses(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control cost computation
costEndET_no_losses = find(tET_no_losses >= COST_COMP_LIMIT, 1, 'first');
sp = find(ref1TT_no_losses  == START_REF1);

[events_intervalET_no_losses, tET_no_losses, alphaET_no_losses, betaET_no_losses, thetaET_no_losses, u_thetaET_no_losses ...
    u_xET_no_losses, u_zET_no_losses, xET_no_losses, zET_no_losses, ref1ET_no_losses, ref2ET_no_losses, timestampET_no_losses ] = ...
    syncSimulation (sp, costEndET_no_losses, events_intervalET_no_losses, tET_no_losses, alphaET_no_losses, betaET_no_losses, ...
    thetaET_no_losses, u_thetaET_no_losses, u_xET_no_losses, u_zET_no_losses, xET_no_losses, zET_no_losses, ref1ET_no_losses, ref2ET_no_losses, timestampET_no_losses);

costEndTT_no_losses = find(tTT_no_losses >= COST_COMP_LIMIT, 1, 'first');
[events_intervalTT_no_losses, tTT_no_losses, alphaTT_no_losses, betaTT_no_losses, thetaTT_no_losses, u_thetaTT_no_losses ...
    u_xTT_no_losses, u_zTT_no_losses, xTT_no_losses, zTT_no_losses, ref1TT_no_losses, ref2TT_no_losses, timestampTT_no_losses ] = ...
    syncSimulation (sp, costEndTT_no_losses, events_intervalTT_no_losses, tTT_no_losses, alphaTT_no_losses, betaTT_no_losses, ...
    thetaTT_no_losses, u_thetaTT_no_losses, u_xTT_no_losses, u_zTT_no_losses, xTT_no_losses, zTT_no_losses, ref1TT_no_losses, ref2TT_no_losses, timestampTT_no_losses);

[JET_no_losses, JuET_no_losses, J1ET_no_losses, J2ET_no_losses, JRMSET_no_losses] = ...
    getControlCost(xET_no_losses, thetaET_no_losses, alphaET_no_losses, betaET_no_losses, u_xET_no_losses, u_thetaET_no_losses, tET_no_losses, ref1ET_no_losses, ref2ET_no_losses);

[JTT_no_losses, JuTT_no_losses, J1TT_no_losses, J2TT_no_losses, JRMSTT_no_losses] = ...
    getControlCost(xTT_no_losses, thetaTT_no_losses, alphaTT_no_losses, betaTT_no_losses, u_xTT_no_losses, u_thetaTT_no_losses, tTT_no_losses, ref1TT_no_losses, ref2TT_no_losses);

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Load the data

%% Data for the periodic control
[events_intervalTT_losses_retx, tTT_losses_retx, alphaTT_losses_retx, betaTT_losses_retx, thetaTT_losses_retx, u_thetaTT_losses_retx ...
    u_xTT_losses_retx, u_zTT_losses_retx, xTT_losses_retx, zTT_losses_retx, ref1TT_losses_retx, ref2TT_losses_retx, timestampTT_losses_retx ] = ...
    loadLabViewDataFile(TT_files_losses_retx{1});

%% Data for the event triggered control
[events_intervalET_losses_retx, tET_losses_retx, alphaET_losses_retx, betaET_losses_retx, thetaET_losses_retx, u_thetaET_losses_retx ...
    u_xET_losses_retx, u_zET_losses_retx, xET_losses_retx, zET_losses_retx, ref1ET_losses_retx, ref2ET_losses_retx,timestampET_losses_retx ] = ...
    loadLabViewDataFile(ET_files_losses_retx{1});

tET_losses_retx = tET_losses_retx - tET_losses_retx(1);
tTT_losses_retx = tTT_losses_retx - tTT_losses_retx(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control cost computation
costEndET_losses_retx = find(tET_losses_retx >= COST_COMP_LIMIT, 1, 'first');
sp = find(ref1TT_losses_retx  == START_REF1);


[events_intervalET_losses_retx, tET_losses_retx, alphaET_losses_retx, betaET_losses_retx, thetaET_losses_retx, u_thetaET_losses_retx ...
    u_xET_losses_retx, u_zET_losses_retx, xET_losses_retx, zET_losses_retx, ref1ET_losses_retx, ref2ET_losses_retx, timestampET_losses_retx ] = ...
    syncSimulation (sp, costEndET_losses_retx, events_intervalET_losses_retx, tET_losses_retx, alphaET_losses_retx, betaET_losses_retx, ...
    thetaET_losses_retx, u_thetaET_losses_retx, u_xET_losses_retx, u_zET_losses_retx, xET_losses_retx, zET_losses_retx, ref1ET_losses_retx, ref2ET_losses_retx, timestampET_losses_retx);

costEndTT_losses_retx = find(tTT_losses_retx >= COST_COMP_LIMIT, 1, 'first');
[events_intervalTT_losses_retx, tTT_losses_retx, alphaTT_losses_retx, betaTT_losses_retx, thetaTT_losses_retx, u_thetaTT_losses_retx ...
    u_xTT_losses_retx, u_zTT_losses_retx, xTT_losses_retx, zTT_losses_retx, ref1TT_losses_retx, ref2TT_losses_retx, timestampTT_losses_retx ] = ...
    syncSimulation (sp, costEndTT_losses_retx, events_intervalTT_losses_retx, tTT_losses_retx, alphaTT_losses_retx, betaTT_losses_retx, ...
    thetaTT_losses_retx, u_thetaTT_losses_retx, u_xTT_losses_retx, u_zTT_losses_retx, xTT_losses_retx, zTT_losses_retx, ref1TT_losses_retx, ref2TT_losses_retx, timestampTT_losses_retx);

[JET_losses_retx, JuET_losses_retx, J1ET_losses_retx, J2ET_losses_retx, JRMSET_losses_retx] = ...
    getControlCost(xET_losses_retx, thetaET_losses_retx, alphaET_losses_retx, betaET_losses_retx, u_xET_losses_retx, u_thetaET_losses_retx, tET_losses_retx, ref1ET_losses_retx, ref2ET_losses_retx);

[JTT_losses_retx, JuTT_losses_retx, J1TT_losses_retx, J2TT_losses_retx, JRMSTT_losses_retx] = ...
    getControlCost(xTT_losses_retx, thetaTT_losses_retx, alphaTT_losses_retx, betaTT_losses_retx, u_xTT_losses_retx, u_thetaTT_losses_retx, tTT_losses_retx, ref1TT_losses_retx, ref2TT_losses_retx);

%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure;
hold on;
plot(xET_losses_retx, 'b')
plot(xET_no_losses, 'g')
plot(xET_losses, 'r')
hold off;



 function [ao, bo, co, do, eo, fo, go, ho, io, jo, ko, lo, mo] ...
            = syncSimulation(sp, endP, a, b, c, d, e, f, g, h, i, j, k, l, m)

        ao = a(sp:(endP));
        bo = b(sp:(endP));
        co = c(sp:(endP));
        do = d(sp:(endP));
        eo = e(sp:(endP));
        fo = f(sp:(endP));
        go = g(sp:(endP));
        ho = h(sp:(endP));
        io = i(sp:(endP));
        jo = j(sp:(endP));
        ko = k(sp:(endP));
        lo = l(sp:(endP));
        mo = m(sp:(endP));
        
    end
end

