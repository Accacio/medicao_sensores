
%clear all

% if(isvalid(instrfind))
% ob=instrfind;
% fclose(ob);
% delete(ob);
% ob=instrfind
% end

%settings
cont_high=225;
cycles=3;

Port_com='com4';

s=serial(Port_com,'Baudrate',115200);
if (strncmp(s.status,['closed'],4))
    ob=instrfind;
    fclose(ob);
    delete(ob);
    s=serial(Port_com,'Baudrate',115200);
end

% if(serial_flag==0)
%     display('Error opening the serial port, please release the serial port connected to the Arduino')
      
fopen(s);

%initialization of variablesis
tmax=cycles*2*cont_high+1; %8000 Para 10 iterations % 2400 3
t=zeros(tmax,1);
amostra=[];
commas=[];
PWM_value=zeros(tmax,1);
tension=zeros(tmax,1);
angle=zeros(tmax,1);
ang_speed=zeros(tmax,1);
ang_accel=zeros(tmax,1);
h1=zeros(tmax,1);
h2=zeros(tmax,1);
sampl_time=zeros(tmax,1);

%data of the forearm
Lf=0.26; %length of the forearm
Lh=0.10; %0.10; %lenght of the hand
Dcf=0.0475;   %clamping of the forearm distance
Dca=0.0575;   %clamping of the arm distance
dcmf=0.682; %forearm distance in percent of the center of mass
g=9.8;




%choosing the parameters of menu on de arduino
arduino_data=fscanf(s)
fwrite(s,'Q');
pause(1);

flag_exit=1;
while(flag_exit==1)
   
    while(s.BytesAvailable==0)
        pause(1)
    end
    arduino_data=fscanf(s);
    if(strncmp(arduino_data,['Ready'],5))
        flag_exit=0;
    end
end

fprintf(s,num2str(cont_high));
pause(1);


%Collecting data
for i=1:tmax;
    arduino_data=fscanf(s);
    commas=strfind(arduino_data,',');

    PWM_value(i,1)=str2double(arduino_data(1:commas(1)-1));
    position=1;
    tension(i,1)=str2double(arduino_data(commas(position)+1:commas(position+1)-1));
    position=2;
    angle(i,1)=str2double(arduino_data(commas(position)+1:commas(position+1)-1))*pi/180;
    position=3;
    ang_speed(i,1)=str2double(arduino_data(commas(position)+1:commas(position+1)-1));
    position=4;
    ang_accel(i,1)=str2double(arduino_data(commas(position)+1:commas(position+1)-1));
    position=5;
    h1(i,1)=str2double(arduino_data(commas(position)+1:commas(position+1)-1));
    position=6;
    h2(i,1)=str2double(arduino_data(commas(position)+1:commas(position+1)-1));
    position=position+1;
    sampl_time(i,1)=str2double(arduino_data(commas(position)+1:commas(position+1)-1))/1000;
    t(i+1,1)=t(i,1)+sampl_time(i,1);
end
t=t(2:end,1);

%closing the movement over the arduino
aux='-1';
fwrite(s,num2str(aux));
pause(1);
flushinput(s);

%ploting the curves
figure
subplot(3,1,1)
plot(t,PWM_value,t,h1,t,h2)
title('PWM input signal and Hysteresis h1 and h2');

subplot(3,1,2)
plot(t,angle,t,ang_speed,t,ang_accel)
title('Angle, Speed and Acceleration')

subplot(3,1,3)
plot(t,tension)
title('Tension over the Load cell and Angle')

%------calculation of the LS the model

%----- Trigonometrics to calculate the Beta angle
x=sqrt(Dca^2+Dcf^2-2*Dca*Dcf*cos(angle));  %tensor calculation
Beta=asin((Dcf*sin(angle))./x);            %Angle beta calculation        

%----Clamping Tension measured adequacy
Tc=tension;

%---- Least Squares Calculations
% dist_cmf=(Lf+Lh)*dcmf;                     %distance of the center of mass of the forearm
% lin_speed=dist_cmf.*ang_speed;             %Linear speed calculations

ls1=ang_accel./(Dcf.*sin(Beta));                   %Inertia force
ls2=g*sin(angle)*(Lf+Lh)*dcmf.*ang_speed./(Dcf.*sin(Beta));   %Friction force
ls3=g*sin(angle)*(Lf+Lh)*dcmf./(Dcf.*sin(Beta));   %Weight force over the forearm
ls4=h1./(Dcf.*sin(Beta));                           %negative hysteresis offset
ls5=h2./(Dcf.*sin(Beta));                           %positive hysteresis offset

% ls1=ang_accel;    %Inertia force            
% ls2=g*sin(angle).*lin_speed; %Friction force
% ls3=g*sin(angle)*dist_cmf;  %Weight force over the forearm
% ls4=h1;                    %negative hysteresis offset
% ls5=h2;                    %positive hysteresis offset


%Least squares calculation
A_LS=[ls1,ls2,ls3,ls4,ls5];             % Matrix A for LS calculation
B_LS=Tc;                                % Matrix B for LS calculation
LS_parameters=inv(A_LS'*A_LS)*A_LS'*B_LS;   %LS parameters results
Fc_LS=A_LS*LS_parameters;               % Force clamping LS equation calculation

Error=Tc-Fc_LS;
Sens_uplim=max(Error(20:end,1));
Sens_lowlim=min(Error(20:end,1));

%Sending parameters to arduino
aux_param=[num2str(LS_parameters(1))];
for i=2:size(LS_parameters)
    aux_param=[aux_param,',',num2str(LS_parameters(i))];
end
aux_param=[aux_param,',',num2str(Sens_lowlim),',',num2str(Sens_uplim)]        

% fprintf(s,aux_param);
% pause(1);
% arduino_data=fscanf(s)

%ploting the clamping force
figure

plot(t,Tc,t,Fc_LS,t,Error,'r',t,Sens_uplim,'k',t,Sens_lowlim,'k')
title('Tension measured vs Clamping force Calculation')
xlabel('Time')
ylabel('Tension (Newtons)')








%closing the comunication
fclose(s);
delete(s);
