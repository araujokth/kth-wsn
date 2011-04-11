load 1_process.mat
% move to single process vars

sp_x1 = x1_v;
sp_x2 = x2_v;
sp_xref = xref_v;

% wrong name!!!!
sp_pck_total = pck_success_v(1:20:end);
sp_pck_success = pck_total_v(1:20:end);
sp_p_error = 1- sum(sp_pck_success)/sum(sp_pck_total);


load 2_process.mat

dp_x1 = x1_v;
dp_x2 = x2_v;
dp_xref = xref_v;

dp_pck_total = pck_success_v(1:20:end);
dp_pck_success = pck_total_v(1:20:end);


dp_p_error = 1- sum(dp_pck_success)/sum(dp_pck_total);

maxLength = min(length(sp_x1), length(dp_x1));
%% 1 sample sensor = 20 samples simulink
plot( [sp_xref(1:maxLength), sp_x1(1:maxLength), dp_x1(1:maxLength)]),
legend('Reference', sprintf('Single Process: \n Packet loss: %0.2f %%', sp_p_error*100), ...
    sprintf('Double Process  \n Packet loss: %0.2f %%', dp_p_error*100))
hold off,