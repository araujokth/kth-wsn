function calibrationResults()
global WTAPP;
global WTCALIBRATION;

WTAPP.NUMBER_WT = 8;
files = dir();
nFiles = size(files,1)-2;

%%save all the steady states
steadyState = zeros(2, WTAPP.NUMBER_WT, length(WTCALIBRATION.U_CONSTANTS), WTCALIBRATION.WINDOW_LENGTH);

clear WTCALIBRATION;

m_file = sprintf('%s/%s',pwd(), files(3).name);
load(m_file);
steadyState =  WTCALIBRATION.steadyState;


stdSteadyState = zeros(2, WTAPP.NUMBER_WT, length(WTCALIBRATION.U_CONSTANTS));
for i=1:length(WTCALIBRATION.U_CONSTANTS)
    for j=1:WTAPP.NUMBER_WT
        %tmp = reshape(steadyState(2, j, i, : ), nFiles, 1);
        stdSteadyState(1, j, i) = std(steadyState(1, j, i, : ));
        stdSteadyState(2, j, i) = std(steadyState(2, j, i, : ));
        meanSteadtState(1, j, i) = mean(steadyState(1, j, i, : ));
        meanSteadtState(2, j, i) = mean(steadyState(2, j, i, : ));
    end
end
hf = figure;
colourOrder = ['b' 'g' 'r' 'c' 'm' 'y' 'k'  'b'];
coeffs1 = cell(WTAPP.NUMBER_WT,1);

        hold on;
        fprintf('Cubic aproximation coeffs:\n');

    for j=1:WTAPP.NUMBER_WT
       % errorbar(WTCALIBRATION.U_CONSTANTS, meanSteadtState(2, j, :),stdSteadyState(2, j, :) , colourOrder(j));
       plot(WTCALIBRATION.U_CONSTANTS', reshape(meanSteadtState(2, j, :), size(meanSteadtState,3), 1)', colourOrder(j));
       
        % Find coefficients for polynomial (order = 3)
        fitResults1 = polyfit(WTCALIBRATION.U_CONSTANTS', reshape(meanSteadtState(2, j, :), size(meanSteadtState,3)', 1), 3);
        coeffs1{j} = fitResults1;
        fprintf('[WT %d] %s \n',j, getEquationString(4, coeffs1{j}, 4));
        % Evaluate polynomial
        yplot1 = polyval(fitResults1, WTCALIBRATION.U_CONSTANTS);
    end
    legend('wt1','wt2', 'wt3', 'wt4', 'wt5', 'wt6', 'wt7', 'wt8');
    hold off;
    
    
%-------------------------------------------------------------------------%
function [s1] = getEquationString(fittype1, coeffs1, digits1)
%GETEQUATIONSTRING(FITTYPE1,COEFFS1,DIGITS1,AXESH1)
%  Get show equation string
%  FITTYPE1:  type of fit
%  COEFFS1:  coefficients
%  DIGITS1:  number of significant digits
%  AXESH1:  axes
if isequal(fittype1, 0)
    s1 = 'Cubic spline interpolant';
elseif isequal(fittype1, 1)
    s1 = 'Shape-preserving interpolant';
else
    op = '+-';
    format1 = ['%s %0.',num2str(digits1),'g*x^{%s} %s'];
    format2 = ['%s %0.',num2str(digits1),'g'];
    fit =  fittype1 - 1;
    s1 = sprintf('y =');
    if abs(coeffs1(1) < 0)
        s1 = [s1 ' -'];
    end
    for i = 1:fit
        sl = length(s1);
        if ~isequal(coeffs1(i),0) % if exactly zero, skip it
            s1 = sprintf(format1,s1,abs(coeffs1(i)),num2str(fit+1-i), op((coeffs1(i+1)<0)+1));
        end
        if (i==fit) && ~isequal(coeffs1(i),0)
            s1(end-5:end-2) = []; % change x^1 to x.
        end

    end
     if ~isequal(coeffs1(fit+1),0)
        sl = length(s1);
        s1 = sprintf(format2,s1,abs(coeffs1(fit+1)));
     end
end