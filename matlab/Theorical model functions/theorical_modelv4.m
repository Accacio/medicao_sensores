
%load('tension_hysteresis\0.5kgfirsttest.mat','angle_value','axisx','loadcell_mean','tension_value')
load('tension_hysteresis\0.5kgsecondtest.mat','angle_filt_value','axisx','loadcell_filt')

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

%-----filter parameters construction
fs=10;
[a,b]=butter(4,0.25*2/fs,'low'); %design of the low passband filter
%vector=filtfilt(a,b,angle_value)*pi/180; %implementation of the filter and filtered signal

%----Chosing the variable for the vector
vector=angle_filt_value*pi/180;
%vector=data3;

axisx(:,1)=(1:size(vector))*0.1;    %x vector construction

%----Calculating the speed
speed_calc=zeros(size(aux_speed_calc,1)+1,1);   %initialiazing the vector
aux_speed_calc=diff(vector)./diff(axisx);       %speed calculation
speed_calc(2:end,1)=-aux_speed_calc(:,1);       %shifting one space the speed vector       
speed_calc=filtfilt(a,b,speed_calc);            %filtering the speed vector

%----Calculating the acceleration
accel_calc=zeros(size(aux_accel,1)+2,1);        %initialiazing the vector      
aux_accel=diff(speed_calc(2:end,1))./diff(axisx(2:end,1));  %acceleration calculation
accel_calc(3:end,1)=aux_accel(:,1);             %shiftiing two speces the speed vector
accel_calc=filtfilt(a,b,accel_calc);            %filtering the acceleration vector

figure
plot(axisx,vector,axisx,speed_calc,axisx,accel_calc)
Inertia_exo=m_exo*(Lf^2)/3;
Inertia_arm=((Lf+Lh)*dcmf)^2*Wf;
I=Inertia_exo+Inertia_arm;

%calculation of tensor
x=sqrt(Dca^2+Dcf^2-2*Dca*Dcf*cos(vector));
%angles calculations
teta_fc=asin((Dca*sin(vector))./x);
Beta=asin((Dcf*sin(vector))./x);

Ts=loadcell_filt.*Dcf.*sin(Beta);
Tt=I*accel_calc;
Torq_f=Wf*(Lf+Lh)*dcmf*g*sin(vector);   %torque of the forearm
Torq_p=dp*Wp;
Friction=Wf*g*sin(vector).*speed_calc*(Lf+Lh)*dcmf;
Fs=Tt+Torq_f+Torq_p+Friction;
figure
plot(axisx,vector,axisx,Fs,axisx,loadcell_filt,axisx,speed_calc)

band1=0.245;
band2=-0.125;
h1(1,1)=1;
h2(1,1)=0;
stepup_sgm=1/30;
stepdown_sgm=1/10;
Hdc_up=0.2333;
Hdc_low=-0.4526;

for i=2:size(speed_calc,1)
    if(speed_calc(i,1)-band1>0)
        h1(i,1)=min(1,h1(i-1,1)+stepup_sgm);
        h2(i,1)=max(0,h2(i-1,1)-stepdown_sgm);
    else
        if(speed_calc(i,1)-band2<0)
            h1(i,1)=max(0,h1(i-1,1)-stepup_sgm);
            h2(i,1)=min(1,h2(i-1,1)+stepdown_sgm);
        else
            h1(i,1)=h1(i-1,1);
            h2(i,1)=h2(i-1,1);
        end
    end
end
H=Hdc_up*h1+Hdc_low*h2;
Fs_hysteresis=Torq_f+H+Tt;

%minimos cuadrados
f1=accel_calc;
f2=Wf*g*sin(vector).*speed_calc*(Lf+Lh)*dcmf;
f3=Wf*(Lf+Lh)*dcmf*g*sin(vector);
f4=h1;
f5=h2;

A_matx=[f1,f2,f3,f4,f5];
B_matx=Ts;
Teta=inv(A_matx'*A_matx)*A_matx'*B_matx;
Fs_result=A_matx*Teta;

figure
subplot(2,1,1)
plot(axisx,Ts,axisx,Fs_hysteresis,axisx,Ts-Fs_hysteresis,'r')
xlabel('Time')
ylabel('Adimensional (Newton*sin(rad))')
title('Manual aproximation of Fs angular presentation')
legend('Loadcell measure','Model signal','Error')
subplot(2,1,2)
plot(axisx,Ts./Dcf.*sin(Beta),axisx,Fs_hysteresis./Dcf.*sin(Beta),axisx,(Ts-Fs_hysteresis)/Dcf.*sin(Beta),'r')
xlabel('Time')
ylabel('Tension (Newton)')
title('Manual aproximation of Fs tension presentation')

figure
subplot(2,1,1)
plot(axisx,Ts,axisx,Fs_result,axisx,Ts-Fs_result,'r')
title('Least Squares aproximation of Fs angular presentation')
xlabel('Time')
ylabel('Adimensional Newton*sin(rad)')
legend('Loadcell measure','Model signal','Error')
subplot(2,1,2)
plot(axisx,Ts./Dcf.*sin(Beta),axisx,Fs_result./Dcf.*sin(Beta),axisx,(Ts-Fs_result)./Dcf.*sin(Beta),'r')
title('Least Squares aproximation of Fs tension presentation')
xlabel('Time')
ylabel('Tension (Newton)')

