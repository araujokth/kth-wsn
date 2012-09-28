%%

function [ volt] = controller(Tank1, Tank2,WT_MAX,x_ref,phase)

if( (Tank1>WT_MAX) || (Tank2>WT_MAX)  )
x_ref=0;
end

%% Constant Voltage Actuator
if(phase==1)
volt=273*x_ref;
end

%% Controll Algorithum
if(phase==2)
volt=273*(x_ref+2);
end
