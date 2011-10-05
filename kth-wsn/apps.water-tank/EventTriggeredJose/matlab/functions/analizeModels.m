NUMBER_WATER_TANKS = 1;
OFFSET = 0;
% d = dir(sprintf('mat/models/%0.0ftanks*', NUMBER_WATER_TANKS));
 d = dir(sprintf('mat/models/david/WT_Models/modeling/process5/*.mat'));
if isempty(d)
    return;
end
tau = zeros(NUMBER_WATER_TANKS, length(d));
k = zeros(NUMBER_WATER_TANKS, length(d));
gamma = zeros(NUMBER_WATER_TANKS, length(d));

for i=1:length(d)
    file = d(i).name;
    load(sprintf('mat/models/david/WT_Models/modeling/process5/%s',file));
    params = WTModeling.parameters;
    tau(:,i) = params.tau;
    k(:,i) = params.k;
    gamma(:,i) = params.gamma;
    
    for j=1:NUMBER_WATER_TANKS
        % Plots each experiment individually
        fprintf('Water tank %d......\n', j+OFFSET);
        fprintf('\t [controller] y_upper=%0.2f \t y_lower=%0.2f \t u=%0.2f \n', ...
            WTModeling.xf(j,1), WTModeling.xf(j,2), WTModeling.u0(j) );
        fprintf('\t [end] y_upper=%0.2f \t y_lower=%0.2f \t u=%0.2f \n', ...
            WTModeling.xf(j,1) + WTModeling.delta_xf(j,1), ...
            WTModeling.xf(j,2) +  WTModeling.delta_xf(j,2), ...
            WTModeling.u0(j) + WTModeling.incr_u );
    end    
    
end

for i=1:NUMBER_WATER_TANKS
    fprintf('WT %0.0f-----\n', i+OFFSET);

    fprintf('[ K ] \t\t mean=%0.3f ; std= %0.3f \n', mean(k(i,:)), std(k(i,:)));
    fprintf('[ tau ] \t mean=%0.3f ; std= %0.3f \n', mean(tau(i,:)), std(tau(i,:)));
    fprintf('[ gamma ] \t mean=%0.3f ; std= %0.3f \n', mean(gamma(i,:)), std(gamma(i,:)));
end

d = dir(sprintf('mat/models/david/WT_Models/logger/process5/*.mat'));
if isempty(d)
    return;
end

for i=1:length(d)
    file = d(i).name;
    load(sprintf('mat/models/david/WT_Models/logger/process5/%s',file));
        
    for j=1:NUMBER_WATER_TANKS
        % Plots each experiment individually
        figure();
        subplot(2,1,1);
        plot(WTSelf.logger.x(:,:,1));
        subplot(2,1,2);
        plot(WTSelf.logger.x(:,:,2));
    end    
    
end
