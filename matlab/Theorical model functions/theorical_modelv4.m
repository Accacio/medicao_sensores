
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

%create estructure of the gradient data
%filtered signal
fs=10;
[a,b]=butter(4,0.25*2/fs,'low'); %design of the low passband filter
%vector=filtfilt(a,b,angle_value)*pi/180; %implementation of the filter and filtered signal
%vector=aux;
vector=angle_filt_value*pi/180;
%vector=data3;
axisx(:,1)=(1:size(vector))*0.1;
grad_state=struct('ek_1',vector(2,1),'state',[3,0,0],'gradient',0,'quantizer',3);

gradient=0;
gradient(1,1)=0;
gradient(2,1)=0;

v_speeds=0;
v_speeds(1,1)=0;
v_speeds(2,1)=0;
ek_0=vector(2,1);
vk_0=0;
t=1;            %requiere confirmación del valor
for i=3:size(vector,1)
    ek_1=ek_0;
    ek_0=vector(i,1);
    [aux_grad,grad_state]=calculate_gradient(ek_0,grad_state);
    gradient(i,1)=aux_grad;
    vk_0=gradient(i,1)*abs(ek_0-ek_1)/t;
    v_speeds(i,1)=(vk_0+v_speeds(i-1,1)+v_speeds(i-2,1))/3;
end
aux_speed_calc=diff(vector)./diff(axisx);
speed_calc=zeros(size(aux_speed_calc,1)+1,1);
speed_calc(2:end,1)=-aux_speed_calc(:,1);
speed_calc=filtfilt(a,b,speed_calc);
aux_accel=diff(speed_calc(2:end,1))./diff(axisx(2:end,1));
accel_calc=zeros(size(aux_accel,1)+2,1);
accel_calc(3:end,1)=aux_accel(:,1);
accel_calc=filtfilt(a,b,accel_calc);

figure
plot(axisx,vector,axisx,gradient,axisx,v_speeds,axisx,speed_calc,axisx,accel_calc)
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
Fs_resul=A_matx*Teta;

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

