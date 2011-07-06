

x1_v = wt.signals.values(:,1);
x2_v = wt.signals.values(:,2);
xref_v = wt.signals.values(:,3);

pck_total_v = pck.signals.values(:,1);
pck_success_v = pck.signals.values(:,2);


% x1_v = x1.signals.values;
% x2_v = x2.signals.values;
% xref_v = xref.signals.values;
% 
% pck_total_v = pck_total.signals.values;
% pck_success_v = pck_success.signals.values;

% initial = find(pck_total_v > 0, 1, 'first');
% final = find(pck_total_v == 499);
% %final = final(6*20);
% final = length(xref_v);

% x1_v = x1_v(initial:end);
% x2_v = x2_v(initial:end);
% xref_v = xref_v(initial:final);

%%convert
for i=1:length(x1_v)
    x1_v(i) = x1_v(i)/87.3932;
    x2_v(i) = x2_v(i)/87.3932;
end

% wrong name!!!!
pck_total = pck_success_v(1:20:end);
pck_success = pck_total_v(1:20:end);
p_error = 1- sum(pck_success)/sum(pck_total);
disp(p_error*100)

% % Build xref
% 
% xref = zeros(size(pck_total_v),1);
% j=1;
% for i=1:size(xref,1) 
%     if pck_total_v(i) == (sample_interval -1)
%         if (x_ref_vector(j) ~= x_ref_vector(end))
%          j=j+1;
%         end
%     end
%     xref(i) = x_ref_vector(j);
% end

grid on; figure()
plot([x1_v, xref_v]),

%save 2_a_process.mat;





