
load('tension_hysteresis\0.5kgfirsttest.mat','angle_value','axisx','loadcell_mean','tension_value')

%create estructure of the gradient data
%filtered signal
%[a,b]=butter(4,
%vector=aux;
%vector=angle_value;
vector=data3;
axisx=1:size(vector);
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
figure
plot(axisx,vector,axisx,gradient*20+100,axisx,v_speeds*20+100)
    