function Erro = obj_function(pop)
global Tc_measured_mean;
global Ag_angle;
global Ag_speed;
global Ag_accel;
global Dca;
global Dcf;
g=9.8;
%population asigment
Ag_I=pop(1);
Ag_Fric_factor=pop(2);
Ag_Tf_factor=pop(3);
Ag_Hdc_up=pop(4);
Ag_Hdc_low=pop(5);
Ag_stepup_sgm=pop(6);
Ag_stepdown_sgm=pop(7);


% %----Calculating the speed
% aux_speed_calc=diff(Ag_angle)./diff(axisx);       %speed calculation
% speed_calc=zeros(size(aux_speed_calc,1)+1,1);   %initialiazing the vector
% speed_calc(2:end,1)=-aux_speed_calc(:,1);       %shifting one space the speed vector       
% speed_calc=filtfilt(a,b,speed_calc);            %filtering the speed vector
% 
% %----Calculating the acceleration
% aux_accel=diff(speed_calc(2:end,1))./diff(axisx(2:end,1));  %acceleration calculation
% accel_calc=zeros(size(aux_accel,1)+2,1);        %initialiazing the vector      
% accel_calc(3:end,1)=aux_accel(:,1);             %shiftiing two speces the speed vector
% accel_calc=filtfilt(a,b,accel_calc);            %filtering the acceleration vector

%---Hysteresis contstruction from speed calc
Upper_lim_speed=0.245;          %Upper limit of the speed band for the hysteresis
Lower_lim_speed=-0.125;         %Lower limit of the speed band for the hysteresis            
%Ag_stepup_sgm=1/30;                %Quantity of steps to achieve the top of the hysteresis (kind of smootnes)
%Ag_stepdown_sgm=1/10;              %Quantity of steps to achieve the bottom of the hysteresis (kind of smootnes)

Hdc_up=0.2333;                  %Factor of top hysteresis                
Hdc_low=-0.4526;                %Factor of bottom hysteresis
h1=0;
h2=0;
h1(1,1)=1;                      %Positive hysteresis function initialization
h2(1,1)=0;                      %Negative hysteresis function initialization

for i=2:size(Ag_speed,1)      %construction of the hysteresis based the speed
    if(Ag_speed(i,1)-Upper_lim_speed>0)
        h1(i,1)=min(1,h1(i-1,1)+Ag_stepup_sgm);
        h2(i,1)=max(0,h2(i-1,1)-Ag_stepdown_sgm);
    else
        if(Ag_speed(i,1)-Lower_lim_speed<0)
            h1(i,1)=max(0,h1(i-1,1)-Ag_stepdown_sgm);
            h2(i,1)=min(1,h2(i-1,1)+Ag_stepup_sgm);
        else
            h1(i,1)=h1(i-1,1);
            h2(i,1)=h2(i-1,1);
        end
    end
end
Ag_H=Ag_Hdc_up*h1+Ag_Hdc_low*h2;         %Hysteresis function

%----- Trigonometrics to calculate the Beta angle
x=sqrt(Dca^2+Dcf^2-2*Dca*Dcf*cos(Ag_angle));  %tensor calculation
Beta=asin((Dcf*sin(Ag_angle))./x);    %Angle beta calculation            

%---- Torques Calcuations
Ag_Tt=Ag_I*Ag_accel;                                 %Inertia momentum calculation
Ag_Friction=0*Ag_Fric_factor*g*sin(Ag_angle).*Ag_speed;  %Friction over the cable calculation
Ag_Torq_f=Ag_Tf_factor*g*sin(Ag_angle);                  %Forearm torque calculation

Ag_Fc=(Ag_Tt+Ag_Friction+Ag_Torq_f+Ag_H)./sin(Beta);

%cuadratic erro calculation
Erro=0;
Erro=sum((Tc_measured_mean-Ag_Fc).^2);





end