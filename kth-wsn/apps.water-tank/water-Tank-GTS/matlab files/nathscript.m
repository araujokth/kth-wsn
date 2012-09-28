
%% Contrller mode selection
V_Act1=controller(S1_LT, S1_UT,WT_MAX,x_ref,phase);    %Voltage to Actuator 1
V_Act2=controller(S2_LT, S2_UT,WT_MAX,x_ref,phase);    %Voltage to Actuator 2
V_Act3=controller(S3_LT, S3_UT,WT_MAX,x_ref,phase);    %Voltage to Actuator 3
V_Act4=controller(S4_LT, S4_UT,WT_MAX,x_ref,phase);    %Voltage to Actuator 4

%% GTS sheduling for Sensor  & Actuator
V_Act2=5.87;
if(lock==1)
S8=1;
else
S8=0;
end
Sensor_GTS=S1 +S2*2+S3*4 +S4*8+S5*16+S6*32+S8*128 ;
Actuator_GTS=Sensor_GTS;
if(phase==2)
Sensor_GTS=Test+128;
Actuator_GTS=Test;
end


%% Voltage for Actuator 5 and 6 
V_Act5=3;
V_Act6=4;

%%for debugging purpose
temp=WT_MAX;
