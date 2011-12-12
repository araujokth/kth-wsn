close all
SHOW_TABLES_ENABLED = false;
INF_VALUE = 10*60*1000;
MIN_CONV = 1000*60; % 1 ms = 1/ (1000*60) min

folder = '/home/david/workspace/tinyos-2.x_tsch_criticalTime_br/apps/OpenWSN/matlab/tmpResults_20_06/';
d = dir(folder);
files = [];
cnt = 0;
for i=1:length(d)
    if (~d(i).isdir)
        files = [files d(i)];
    end
end

linksArray = [];

linksStr = [];

for i=1:length(files)
    load(sprintf('%s%s', folder, files(i).name));
    
    if isempty(linksArray)
        linksArray = zeros( length(files), power(TCAapp.motelist.nMotes,2));
        linksStr = cell( length(files), power(TCAapp.motelist.nMotes,2));
        nLinks = power(TCAapp.motelist.nMotes,2)-TCAapp.motelist.nMotes;
    end
    % fill the linksArray
    %experiment = reshape(1,  power(TCAapp.motelist.nMotes,2), TCAapp.criticalTime);
    if (power(TCAapp.motelist.nMotes,2) ~= size(linksArray,2))
        continue;
    end
    experiment = reshape(TCAapp.criticalTime(1:TCAapp.motelist.nMotes, 1:TCAapp.motelist.nMotes), 1, power(TCAapp.motelist.nMotes,2));
    linksArray(i, :) = experiment;
    if SHOW_TABLES_ENABLED
        
        n = 1;
        for l=1:TCAapp.motelist.nMotes
            for m=1:TCAapp.motelist.nMotes
                linksStr{i, n} = sprintf('%s - %s', TCAapp.motelist.moteList{l}.reference, TCAapp.motelist.moteList{m}.reference);
                n = n +1;
            end
        end
        
        % create a latex table
        name = strrep( d(i).name, '_', ' ' );
        caption=sprintf('Critical time Table for %s', name);
        label = sprintf('table:critical_tme_%s', d(i).name);
        ncolumns = TCAapp.motelist.nMotes;
        nrows = ncolumns;
        
        fprintf('\\begin{table}[h] \n \\centering \n \\begin{tabularx}{\\textwidth}{');
        % build the header
        for i=1:(ncolumns+1)
            if (i ~= (ncolumns+1))
                fprintf('c |');
            else
                fprintf('c }\n \n');
            end
        end
        fprintf(' & ');
        for j=1:(ncolumns+1)
            % Fill header
            if j == 1
                for k=1:nrows
                    fprintf('\\textbf{%s}', TCAapp.motelist.moteList{k}.reference);
                    if (k ~= nrows)
                        fprintf(' & ');
                    else
                        fprintf(' \\\\ \n');
                        %fprintf(' \\midrule \n');
                    end
                end
            else
                % for all tx, show all the
                for i=1:(ncolumns +1)
                    if i == 1
                        fprintf('\\textbf{%s} &', TCAapp.motelist.moteList{j-1}.reference);
                    elseif (i ~= (ncolumns+1))
                        fprintf(' %d &', TCAapp.criticalTime(j-1,i-1));
                    else
                        fprintf(' %d \\\\ \n', TCAapp.criticalTime(j-1,i-1));
                    end
                end
            end
        end
        disp(sprintf('\n  \\end{tabularx} \n  \\caption{%s}\\label{%s}\n\\end{table}\n', caption, label));
    end
end

nMotes = TCAapp.motelist.nMotes;
%% Get stadistics from the linksArray
% delete the columns with -1 values
idxNull = find(linksArray(1,:) == -1);
idxValid = setxor(idxNull, 1:length(linksArray(1,:)));
linksArray = linksArray(:,idxValid);

% replace the Inf values
idxInf = linksArray > INF_VALUE & linksArray ~= Inf;
%check how many critical time are bigger than INF_VALUE
if sum(sum(idxInf > 0))
    tmp = linksArray(idxInf);
    disp((tmp-INF_VALUE)/1000);
end
idxInf = linksArray == Inf | linksArray > INF_VALUE;
linksArray(idxInf) = INF_VALUE;

% second moment unbiased
stdLinks = std(linksArray(:,:));
% mean estimator
meanLinks = mean(linksArray(:,:));

nExperiments = size(linksArray,1);
nLinks = size(linksArray,2);

%% Confidence interval over the mean value
% mean ~ nomal random variable with mean nu and standard deviation
% sigma/sqrt(n)
%         (m1 - nu)
% T = ------------------
%     sqrt((Sn^2/(n-1)))
% Where nu is the desired mean value
% and Sn is the standard deviation resulting

% The confidence interval is defined by
% P(m1-t2*sqrt(S(n-1)^2/n) < nu < m1-t2*sqrt(S(n-1)^2/n) = 1- alfa
% 
% Let's consider that we want a 90% confidence

% What is the 99th percentile of the T-Distribution with nExperiments
confidence = 0.99;
alfa = 1 - confidence;
t1 = tinv(1-alfa/2, nExperiments-1);

upperInterval = meanLinks + t1*stdLinks/sqrt(nExperiments);
lowerInterval = meanLinks - t1*stdLinks/sqrt(nExperiments);

for i=1:nLinks
    fprintf('Link %d: (%0.3f, %0.3f) min\n', i, lowerInterval(i)/MIN_CONV, upperInterval(i)/MIN_CONV);
end

%% Skewness for each pair as a TX and RX
skewnessArray = zeros(nLinks/2, 1);
row=1;
col=2;
for i=1:nLinks/2
    
%     i
%     fprintf('( %d, %d) = %d \n',row, col, (row-1)*(nMotes-1)+col - 1);
%     fprintf('( %d, %d) = %d \n',col, row, (col-1)*(nMotes-1)+row );
%     meanLinks(1,[(row-1)*(nMotes-1)+col - 1 (col-1)*(nMotes-1)+row]);

    % Skewness
    skewnessArray(i, :) = skewness(reshape(linksArray(:,[(row-1)*(nMotes-1)+col - 1 (col-1)*(nMotes-1)+row]), nExperiments*2, 1, 1));
    
    if col ==  nMotes
        row = row + 1;
        col = row + 1 ;
    else
        col = col + 1;
    end
end


if SHOW_TABLES_ENABLED
    
    %% Create the latex table with the results
    % create a latex table
    name = strrep( d(i).name, '_', ' ' );
    caption=sprintf('Stadistics for the %d different pair of links and %d experiments', nLinks, length(files) );
    label = sprintf('table:stadistics_%d_motes', TCAapp.motelist.nMotes);
    ncolumns = 4; % Pair Desc & Mean & Std  -1
    nrows = nLinks;
    
    fprintf('\\begin{table}[h] \n \\centering \n \\begin{tabularx}{0.4\\textwidth}{');
    for i=1:(ncolumns+1)
        if (i ~= (ncolumns+1))
            fprintf('c |');
        else
            fprintf('c }\n \n');
        end
    end
    
    % Fill header
    fprintf(' & \\textbf{Mean [s]} & \\textbf{Mean [min]} &  \\textbf{Std [s]}* & \\textbf{Std [min]}* \\\\ \n');
    
    %% Fill table
    for i=1:(nrows+TCAapp.motelist.nMotes)
        if (mod(i,TCAapp.motelist.nMotes+1) ~= 1)
            fprintf('%s & %0.2f & %0.2f & %0.2f & %0.2f \\\\ \n', ...
                linksStr{1, i},  meanLinks(i)/1000, meanLinks(i)/(1000*60), ...
                stdLinks(i)/1000, stdLinks(i)/(1000*60)  );
            
        else
            fprintf('\\midrule \n')
        end
    end
    
    disp(sprintf('\n  \\end{tabularx} \n  \\caption{%s}\\label{%s}\n\\end{table}\n', caption, label));
    
    %% Plot the histograms
    % Number of pairs
    nLinks = power(TCAapp.motelist.nMotes,2) - TCAapp.motelist.nMotes;
    columnsFigure = 2;
    rowsFigure = ceil(nLinks / columnsFigure);
    
end
