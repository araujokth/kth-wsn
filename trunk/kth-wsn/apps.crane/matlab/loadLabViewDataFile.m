
function [events_interval, t, alpha, beta, theta, u_theta ...
    u_x, u_z, x, z, ref1, ref2, timestamp] = loadLabViewDataFile(fileToread)

    T_SYMBOL = 1/(2*32678);

    % Initializations
    cdm = [];
    events_interval = []; t = []; alpha =[]; beta = []; theta = [];
    u_theta = []; u_x = []; u_z = []; x = []; z = []; ref1 = []; ref2 = [];
    timestamp = []; Ts = []; events = []; pckSuccess = []; pckTotal = [];

    if nargin < 1
        file = '../Outputs';
    end

   
    run(fileToread);
    timestamp = timestamp*T_SYMBOL;
    
    
    %% repair bad data
    reference = [25 40 30];
    [ref1] = repairSteps(reference, ref1);

    reference = [1.5708 2.3562 1.5708];
    [ref2] = repairSteps(reference, ref2);

    function [out] = repairSteps(reference, v)
        
        prevStep = 1;
        %% repair
        for i=1:length(v)
            last = min(i+4,length(v));
            meanS = mean(v(i:last));
            if ~ismember(meanS, reference)
                stepsA = abs(reference - meanS);
                for j=1:4
                    if ~ismember(v(i+j-1), reference)
                        v(i+j-1) =  reference(prevStep);
                    end
                end
            else
                prevStep = find(reference == meanS, 1, 'first');
            end
            out =v;
        end
    end

end