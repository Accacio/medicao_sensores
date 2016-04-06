clear
close all
clc
%% Serial Settings

Dados='..\Dados\';
plots_exp='..\plots\experimentais\';
if ~exist(plots_exp, 'dir')
    mkdir(plots_exp);
end

Port_com='com3';
s=serial(Port_com,'Baudrate',115200);
fopen(s);

%% Initialization of variables
%% Ang. Set , Ang. Msr , L.C Msr , Tens. Calc , Tens. Error , Force Outbound Flag , Ang. Controlled
out=[];
amostra=[];
commas=[];
% experiment Time
xmax=1000; % change at will
Ang_set=zeros(xmax,1);
Ang_Msr=zeros(xmax,1);
LC_Msr=zeros(xmax,1);
Tens_Calc=zeros(xmax,1);
Tens_Error=zeros(xmax,1);
Force_Outbound_Flag=zeros(xmax,1);
Ang_Controlled=zeros(xmax,1);


image='arduinoimage.png';
%fig=figure;
axisx=[];
axisx(1,1)=1
%PWM_value
PWM_value_ymin=0;
PWM_value_ymax=100;


%vim
vim_ymin=0;
vim_ymax=5;
%Rp
Rp_ymin=0;
Rp_ymax=100;
%vref
vref_ymin=0;
vref_ymax=1060;
%im
im_ymin=0;
im_ymax=1500;

loadcell_ymin=0;
loadcell_ymax=100;

sampl_time=0;
commas_error=0;

out=fscanf(s)
pause(1);
fwrite(s,'a');% armmovement
out=fscanf(s)
out=fscanf(s)
out=fscanf(s)
out=fscanf(s)
pause(5);
disp(['Continuos Control - 2'])
fwrite(s,char('2'));% continuos control
pause(5)
disp(['Deviation - 20'])
fwrite(s,char('20'));% 20% of deviation
flushinput(s)
pause(1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic
count_plot=0;
figure1=figure(1);
figure2=figure(2);
for i=2:xmax+1;


if i==50
  disp(['110 degrees'])
  fwrite(s,char('110'));% Set angle as 110 degrees
end

if i==300 %% rever valor
  disp(['45 degrees - Put some obstacle'])
  fwrite(s,'45')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
axisx(i,1)=i;
%pause(1)
out=fscanf(s)
commas=strfind(out,',');
 if numel(commas)<7

 Ang_set(i,1)=Ang_set(i-1,1);
 Ang_Msr(i,1)=Ang_Msr(i-1,1);
 LC_Msr(i,1)=LC_Msr(i-1,1);
 Tens_Calc(i,1)=Tens_Calc(i-1,1);
 Tens_Error(i,1)=Tens_Error(i-1,1);
 Force_Outbound_Flag(i,1)=Force_Outbound_Flag(i-1,1);
 Ang_Controlled(i,1)=Ang_Controlled(i-1,1);

 commas_error=commas_error+1;
 else
    commanum=1;
    Ang_set(i,1)=str2double(out(1:commas(commanum)-1));
    Ang_Msr(i,1)=str2double(out(commas(commanum)+1:commas(commanum+1)-1));
    commanum=commanum+1;
    LC_Msr(i,1)=str2double(out(commas(commanum)+1:commas(commanum+1)-1));
    commanum=commanum+1;
    Tens_Calc(i,1)=str2double(out(commas(commanum)+1:commas(commanum+1)-1));
    commanum=commanum+1;
    Tens_Error(i,1)=str2double(out(commas(commanum)+1:commas(commanum+1)-1));
    commanum=commanum+1;
    Force_Outbound_Flag(i,1)=str2double(out(commas(commanum)+1:commas(commanum+1)-1));
    commanum=commanum+1;
    Ang_Controlled(i,1)=str2double(out(commas(commanum)+1:commas(commanum+1)-1));
end


    %plot Rp
  if count_plot==10
    count_plot=0;

    % samples x ang_set and samples x ang_msr
    figure(1)
    plot(axisx(i-11*sign(i-11):i),Ang_set(i-11*sign(i-11):i),'r','LineWidth',2)
    hold on
    plot(axisx(i-11*sign(i-11):i),Ang_Msr(i-11*sign(i-11):i),'b','LineWidth',2)
    hold on
    title('Angle Measurement')
    legend('Reference Angle','Measured Angle')
    xlabel('Samples')
    ylabel('Angle')
    axis([0 xmax 0 200 ]);% rever

    % samples x
    figure(2)
    plot(axisx(i-11*sign(i-11):i),LC_Msr(i-11*sign(i-11):i),'g','LineWidth',2)
    hold on
    plot(axisx(i-11*sign(i-11):i),Tens_Calc(i-11*sign(i-11):i),'b','LineWidth',2)
    hold on
    plot(axisx(i-11*sign(i-11):i),Tens_Error(i-11*sign(i-11):i),'r','LineWidth',2)
    hold on
    title('Tension Measurement')
    legend('Measured Tension','Calculated Tension','Error')
    xlabel('Samples')
    ylabel('Tension')
    axis([0 xmax -200 200 ]);% rever


    drawnow
  else
  count_plot=count_plot+1;
  end
end
toc
fwrite(s,'-1');
fclose(s);
delete(s);

figure(1)
set(gcf, 'PaperPosition', [0 0 8 8]);
set(gcf, 'PaperSize', [8 8]);
saveas(gcf,[plots_exp 'Angle.pdf']);

figure(2)
set(gcf, 'PaperPosition', [0 0 8 8]);
set(gcf, 'PaperSize', [8 8]);
saveas(gcf,[plots_exp 'tension.pdf']);

