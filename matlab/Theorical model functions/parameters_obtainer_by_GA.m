
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



%Generating the Hysteresis function with AG results

%---Hysteresis contstruction from speed calc
Upper_lim_speed=0.245;          %Upper limit of the speed band for the hysteresis
Lower_lim_speed=-0.125;         %Lower limit of the speed band for the hysteresis            
%Ag_stepup_sgm=1/30;                %Quantity of steps to achieve the top of the hysteresis (kind of smootnes)
%Ag_stepdown_sgm=1/10;              %Quantity of steps to achieve the bottom of the hysteresis (kind of smootnes)


stepup_sgm=Ga_results.x(6);                %Quantity of steps to achieve the top of the hysteresis (kind of smootnes)
stepdown_sgm=Ga_results.x(7);              %Quantity of steps to achieve the bottom of the hysteresis (kind of smootnes)

Hdc_up=Ga_results.x(4);                  %Factor of top hysteresis                
Hdc_low=Ga_results.x(5);                %Factor of bottom hysteresis
h1=0;
h2=0;
h1(1,1)=1;                      %Positive hysteresis function initialization
h2(1,1)=0;                      %Negative hysteresis function initialization

for i=2:size(Ag_speed,1)      %construction of the hysteresis based the speed
    if(Ag_speed(i,1)-Upper_lim_speed>0)
        h1(i,1)=min(1,h1(i-1,1)+stepup_sgm);
        h2(i,1)=max(0,h2(i-1,1)-stepdown_sgm);
    else
        if(Ag_speed(i,1)-Lower_lim_speed<0)
            h1(i,1)=max(0,h1(i-1,1)-stepdown_sgm);
            h2(i,1)=min(1,h2(i-1,1)+stepup_sgm);
        else
            h1(i,1)=h1(i-1,1);
            h2(i,1)=h2(i-1,1);
        end
    end
end
Agresult_H=Hdc_up*h1+Hdc_low*h2;         %Hysteresis function

%----- Trigonometrics to calculate the Beta angle
x=sqrt(Dca^2+Dcf^2-2*Dca*Dcf*cos(Ag_angle));  %tensor calculation
Beta=asin((Dcf*sin(Ag_angle))./x);    %Angle beta calculation            

%---- Torques Calcuations
Agresult_Tt=Ga_results.x(1)*Ag_accel;                                 %Inertia momentum calculation
Agresult_Friction=0*Ga_results.x(2)*g*sin(Ag_angle).*Ag_speed;  %Friction over the cable calculation
Agresult_Torq_f=Ga_results.x(3)*g*sin(Ag_angle);                  %Forearm torque calculation

Agresult_Fc=(Agresult_Tt+Agresult_Friction+Agresult_Torq_f+Agresult_H)./sin(Beta);

t=(1:size(Agresult_Fc,1))';
figure
plot(t,Tc_measured_mean,t,Agresult_Fc)