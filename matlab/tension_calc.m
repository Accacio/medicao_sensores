function [ Fcf, phi ] = tension_calc( Dca,phi_step,Dcf_div,subp_arrow,subp_pos)
%program to simulate the clamping force of the arm project
%Ffc tension over the clamping point in the forearme in Newtons
%phi angle of the elbow in radians

% phi_step=30*pi/180;               %variation in steps of rad
% Dcf_div=10;
% Dca=0.06;
% subp_arrow=2;
% subp_pos=1;

%Global variables
global shoulder_phi;   %initial position of shoulders angle
global arm_yini;        %high of the sholder from the origin
global arm_xini;        %distace of the shoulder from the origin
global forearm_yini;    
global forearm_xini;
global La;    %arm lenght in mts.
global Lf;    %forearm length in mts.
global Lh;    %hand lenght closed in mts.
global Dfcm;
global Dd;

%Initial position of the arm
shoulder_phi=pi;   %initial position of shoulders angle
arm_yini=0;        %high of the sholder from the origin
arm_xini=0;        %distace of the shoulder from the origin
forearm_yini=arm_yini+La*cos(shoulder_phi);    
forearm_xini=arm_xini+La*sin(shoulder_phi);

%definitions about the human arm and disturbance
La=0.28;    %arm lenght in mts.
Lf=0.26;    %forearm length in mts.
Lh=0.08;    %hand lenght closed in mts.
Sw=100;     %subject weight in Kg.
per_fcm=38.96/100;  %percentage center of mass of the forearm
per_fw=1.6/100;     %percentage weight of the forearm
per_hcm=82.00/100;  %percentage center of mass of the hand
per_hw=0.7/100;     %percentage weight of the hand
Wd_object=1;       % weight distubance in Kg.
g=9.8;      %gravity m/s.
elbow_min_open=30*pi/180;
elbow_max_open=150*pi/180;
%mechanical definitions
Ce=0.04;    %clamping elements length in mts.
Lp=0.04;    %pulley length in mts.
Llc=0.05;   %load cell length in mts.

%variables
%Dca=0.10;      % Clamping arm distance in mts.
%Dcf=0.10;      % Clamping forearm distance in mts.
lw=0;           % lenght of work in mts.
teta_fc=[];     % forearm clamping angle in radians.
teta_ac=[];     % arm clamping angle in radians
phi=0;          % elbow angle in radians.
max_phi_open=2.1679; %150 grados

%forearm variables
Wf=per_fw*Sw;       %weight of the forearm in kg.
Dfcm=per_fcm*Lf;    %distance of the center of mass in the forearm in mts.

%disturbance variables
Wd=per_hw*Sw+Wd_object; % weight of the disturbance included weight of the hand
Dd=Lf+per_hcm*Lh;       %ditance of the disturbance from the elbow

%work lenghts maximums to work
lw=La-Dca-(Ce+Lp+Llc);
if(lw>0)
    aux_teta_cf=asin((Dca*sin(max_phi_open))/lw);
    aux_teta_ca=pi-max_phi_open-aux_teta_cf;
    Dcf_max=(lw*sin(aux_teta_ca))/sin(max_phi_open);

    Dcf_step=Dcf_max/Dcf_div;
    phi=[0];
    Fcf=[0];
    Dcf=[0];
    x=[0];
    
    i=0;
    phi_val=['elbow ang='];
    leg_str=[0];

    for phi_aux=elbow_min_open:phi_step:elbow_max_open
        i=i+1;
        phi(i)=phi_aux;
        %leg_str(i,1)=num2str(phi(i)*180/pi)
        
        for j=1:Dcf_div
            Dcf(j,1)=Dcf_max-(j-1)*Dcf_step;
            
            %calculation of tensor 
            x(j,i)=sqrt(Dca^2+Dcf(j,1)^2-2*Dca*Dcf(j,1)*cos(phi(i)));
            
            %angles calculations
            teta_fc(j,i)=asin((Dca*sin(phi(i)))/x(j,i));
            teta_ac(j,i)=asin((Dcf(j,1)*sin(phi(i)))/x(j,i));

    
            %Clamping force calculation
            Tfc=Wf*Dfcm*g*sin(phi(i));
            Td=Wd*Dd*g*sin(phi(i));
            Fcf(j,i)=(Tfc+Td)/(Dca*sin(teta_ac(j,i)));
        end
    end
    if(subp_arrow>0 && subp_pos>0)
        subplot(subp_arrow,2,2*(subp_pos-1)+1)
        ploting_arm(shoulder_phi,phi,Dca,Dcf(1,:),x,teta_ac)
        subplot(subp_arrow,2,2*(subp_pos-1)+2)
        plot(Dcf',Fcf()')
    %    legend(leg_str)
    end
else
    display('Dca distance is too long')
end


end
