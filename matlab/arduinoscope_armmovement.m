clear
%settings
cont_high=400;
%percent_high=20;
xmax=500; %250 Para 1 iterations % 2400 3
Wf=0.5; %change to each weight value
Dca=0.057;
Dcf=0.047;
Port_com='com3';

s=serial(Port_com,'Baudrate',115200);
fopen(s);

%creation of class with principal data (is required first the transfer
%function class initialization)
data=transfer_function;

out=[];
amostra=[];
commas=[];
PWM_value=[];
vref=[];
im=[];
vpot=[];
vref_mean=[];
im_mean=[];
vpot_mean=[];

%image='arduinoimage.png';
fig=figure;
axisx=[];


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
fwrite(s,'m');
pause(1);
%fwrite(s,'40')
pause(1);
count_plot=0;

tic
for i=1:xmax+1;
  %if i==150
  %  fwrite(s,'140')
  %end
axisx(i,1)=i;
out=fscanf(s);
commas=strfind(out,',');
% if numel(commas)<2
% angle_value(i,1)=angle_value(i-1,1);
% loadcell_mean(i,1)=loadcell_mean(i-1,1);
% tension_value(i,1)=tension_value(i-1,1);
% commas_error=commas_error+1;
%  else
loadcell_mean(i,1)=str2double(out(1:commas(1)-1));
%vref(i,1)=str2double(out(commas(1)+1:commas(2)-1));
loadcell_filt(i,1)=str2double(out(commas(1)+1:commas(2)-1));
loadcell_rawfilt(i,1)=str2double(out(commas(2)+1:commas(3)-1));
angle_mean_value(i,1)=str2double(out(commas(3)+1:commas(4)-1));
angle_filt_value(i,1)=str2double(out(commas(4)+1:commas(5)-1));
angle_rawfilt_value(i,1)=str2double(out(commas(5)+1:commas(6)-1));
%tension_value(i,1)=tension_calc(Dca,Dcf,angle_value(i,1)*pi/180,Wf);

%end

%     %plot vref
%     subplot(3,1,1)
%     plot(axisx(end-1*sign(end-1):end),vref(end-1*sign(end-1):end),'LineWidth',2)
%     hold on
%     plot(axisx(end-1*sign(end-1):end),vref_mean(end-1*sign(end-1):end),'r','LineWidth',2)
%     axis([0 xmax vref_ymin vref_ymax]);

    %plot Rp
  if count_plot==10
    count_plot=0;


    % Hysteresis plot angle_mean_value
    subplot(2,2,1)
    %plot(angle_mean_value(end-11*sign(end-11):end),tension_value(end-11*sign(end-11):end),'LineWidth',2)
    hold on
    plot(angle_mean_value(end-11*sign(end-11):end),loadcell_mean(end-11*sign(end-11):end),'r','LineWidth',2)
    axis([0 180 0 250 ]);

    % Hysteresis plot angle_filt_value
    subplot(2,2,2)
    %plot(angle_filt_value(end-11*sign(end-11):end),tension_value(end-11*sign(end-11):end),'LineWidth',2)
    hold on
    plot(angle_filt_value(end-11*sign(end-11):end),loadcell_filt(end-11*sign(end-11):end),'g','LineWidth',2)
    axis([0 180 0 250 ]);

    % Hysteresis plot angle_rawfilt_value
    subplot(2,2,3)
    hold on
    plot(angle_rawfilt_value(end-11*sign(end-11):end),loadcell_rawfilt(end-11*sign(end-11):end),'b','LineWidth',2)
    axis([0 180 0 250 ]);

    %plot loadcell
    subplot(2,2,4)
    %plot(axisx(end-11*sign(end-11):end),tension_value(end-11*sign(end-11):end),'LineWidth',2)
    hold on
    plot(axisx(end-11*sign(end-11):end),loadcell_mean(end-11*sign(end-11):end),'r','LineWidth',2)
    hold on
    plot(axisx(end-11*sign(end-11):end),loadcell_filt(end-11*sign(end-11):end),'g','LineWidth',2)
    hold on
    plot(axisx(end-11*sign(end-11):end),loadcell_rawfilt(end-11*sign(end-11):end),'b','LineWidth',2)
    hold on
    axis([0 xmax 0 250]);

    drawnow
  else
  count_plot=count_plot+1;
  end
end
toc



%%%%%%%%%%%%%%%%%%%%%%%%%%%

%data.samp_t=sampl_time/(1000*xmax)
%commas_error
%data.t=0:sampl_time/1000:(xmax*sampl_time)/1000;
%data.input=PWM_value;
%data.output=vpot;
%aux='0,0';
%fwrite(s,aux);
%pause(1);
%out=fscanf(s);



fwrite(s,'-1');
fclose(s);
delete(s);


%another posibility for ploting point by point
% 2) create a single line object
% h = plot(x(1),y(1),'bo') ;
% for i=2:length(x),
%   pause
%   xd = get(h,'xdata') ;
%   yd = get(h,'ydata') ;
%   set(h,'xdata',[xd(:) ; x(i)],'ydata',[yd(:) ; y(i)]) ;
% end
%
% take a look at <set> and <get> and graphic handles ...
