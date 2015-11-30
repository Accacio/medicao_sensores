
%load('tension_hysteresis\0.5kgfirsttest.mat','angle_value','axisx','loadcell_mean','tension_value')
load('tension_hysteresis\1.5kgfourthtest.mat','angle_rawfilt_value','axisx','loadcell_rawfilt')
%load('tension_hysteresis\0.5kgfilteronly.mat','angle_rawfilt_value','axisx','loadcell_rawfilt')
loadcell_filt=loadcell_rawfilt;
angle_filt_value=angle_rawfilt_value;
%data of the forearm
Lf=0.28; %length of the forearm
Lh=0; %0.10; %lenght of the hand
Dcf=0.0475;   %clamping of the forearm distance
Dca=0.0575;   %clamping of the arm distance
m_exo=0.01; %despreciable
dcmf=1; %0.682; %forearm distance in percent of the center of mass
wcmf=1; %0.022;  %forearm weight in percent
Sw=0.5;     %subject weigth
Wp=0;       %perturbance weight
g=9.8;
dp=Lf+Lh;
Wf=Sw*wcmf;

%---  Parameters initialization
axis=0;
vector=0;

%-----filter parameters construction
fs=10;
[a,b]=butter(4,0.25*2/fs,'low'); %design of the low passband filter
%vector=filtfilt(a,b,angle_value)*pi/180; %implementation of the filter and filtered signal

%----Chosing the variable for the vector
vector=angle_filt_value*pi/180;
%vector=data3;

axisx(:,1)=(1:size(vector))*0.1;    %x vector construction

%----Calculating the speed
aux_speed_calc=diff(vector)./diff(axisx);       %speed calculation
speed_calc=zeros(size(aux_speed_calc,1)+1,1);   %initialiazing the vector
speed_calc(2:end,1)=-aux_speed_calc(:,1);       %shifting one space the speed vector       
speed_calc=filtfilt(a,b,speed_calc);            %filtering the speed vector

%----Calculating the acceleration
aux_accel=diff(speed_calc(2:end,1))./diff(axisx(2:end,1));  %acceleration calculation
accel_calc=zeros(size(aux_accel,1)+2,1);        %initialiazing the vector      
accel_calc(3:end,1)=aux_accel(:,1);             %shiftiing two speces the speed vector
accel_calc=filtfilt(a,b,accel_calc);            %filtering the acceleration vector

%---Hysteresis contstruction from speed calc
Upper_lim_speed=0.245;          %Upper limit of the speed band for the hysteresis
Lower_lim_speed=-0.125;         %Lower limit of the speed band for the hysteresis            
stepup_sgm=1/30;                %Quantity of steps to achieve the top of the hysteresis (kind of smootnes)
stepdown_sgm=1/9;              %Quantity of steps to achieve the bottom of the hysteresis (kind of smootnes)

Hdc_up=0.2333;                  %Factor of top hysteresis                
Hdc_low=-0.4526;                %Factor of bottom hysteresis

h1(1,1)=1;                      %Positive hysteresis function initialization
h2(1,1)=0;                      %Negative hysteresis function initialization

for i=2:size(speed_calc,1)      %construction of the hysteresis based the speed
    if(speed_calc(i,1)-Upper_lim_speed>0)
        h1(i,1)=min(1,h1(i-1,1)+stepup_sgm);
        h2(i,1)=max(0,h2(i-1,1)-stepdown_sgm);
    else
        if(speed_calc(i,1)-Lower_lim_speed<0)
            h1(i,1)=max(0,h1(i-1,1)-stepdown_sgm);
            h2(i,1)=min(1,h2(i-1,1)+stepup_sgm);
        else
            h1(i,1)=h1(i-1,1);
            h2(i,1)=h2(i-1,1);
        end
    end
end
H=Hdc_up*h1+Hdc_low*h2;         %Hysteresis function

%----- Trigonometrics to calculate the Beta angle
x=sqrt(Dca^2+Dcf^2-2*Dca*Dcf*cos(vector));  %tensor calculation
Beta=asin((Dcf*sin(vector))./x);    %Angle beta calculation            

%----Clamping Tension measured adequacy
Tc=loadcell_filt.*Dcf.*sin(Beta);

%----- Forearm Inertia calculation
Inertia_exo=m_exo*(Lf^2)/3;         %Exoesqueleton inertia
Inertia_arm=((Lf+Lh)*dcmf)^2*Wf;    %Forearm inertia
I=Inertia_exo+Inertia_arm;          %Total inertia factor

%---- Torques Calcuations
Tt=I*accel_calc;                        %Inertia momentum calculation
Torq_f=Wf*(Lf+Lh)*dcmf*g*sin(vector);   %Forearm torque calculation
Torq_p=dp*Wp;                           %Perturbance torque calcultion
Friction=Wf*g*sin(vector).*speed_calc*(Lf+Lh)*dcmf; %Friction over the cable calculation

%---- Clamping force Calculation
Fc=Tt+Torq_f+Torq_p+Friction;       %Clamping force theorical calculation
Fc_hysteresis=Torq_f+H+Tt;          %Clamping force based only on hysteresis, aceleration and forearm weight


%---- Least Squares Calculations
ls1=accel_calc;
ls2=g*sin(vector)*(Lf+Lh)*dcmf.*speed_calc;
ls3=g*sin(vector)*(Lf+Lh)*dcmf;
ls4=h1;
ls5=h2;

A_LS=[ls1,ls2,ls3,ls4,ls5];
B_LS=Tc;
LS_parameters=inv(A_LS'*A_LS)*A_LS'*B_LS;
Fc_least_squares=A_LS*LS_parameters;

Error_newtons=(Tc-Fc_least_squares)./Dcf.*sin(Beta);
%---- Angle, angular speed and angular aceleration figrue
figure
plot(axisx,vector,axisx,speed_calc,axisx,accel_calc)

figure
subplot(2,1,1)
plot(axisx,Tc,axisx,Fc_hysteresis,axisx,Tc-Fc_hysteresis,'r')
xlabel('Time')
ylabel('Adimensional (Newton*sin(rad))')
title('Manual aproximation of Fc angular presentation')
legend('Loadcell measure','Model signal','Error')
subplot(2,1,2)
plot(axisx,Tc./Dcf.*sin(Beta),axisx,Fc_hysteresis./Dcf.*sin(Beta),axisx,(Tc-Fc_hysteresis)/Dcf.*sin(Beta),'r')
xlabel('Time')
ylabel('Tension (Newton)')
title('Manual aproximation of Fc tension presentation')

figure
subplot(2,1,1)
plot(axisx,Tc,axisx,Fc_least_squares,axisx,Tc-Fc_least_squares,'r')
title('Least Squares aproximation of Fc angular presentation')
xlabel('Time')
ylabel('Adimensional Newton*sin(rad)')
legend('Loadcell measure','Model signal','Error')
subplot(2,1,2)
plot(axisx,Tc./Dcf.*sin(Beta),axisx,Fc_least_squares./Dcf.*sin(Beta),axisx,Error_newtons,'r')
title('Least Squares aproximation of Fc tension presentation')
xlabel('Time')
ylabel('Tension (Newton)')

