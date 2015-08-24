s=serial('com3');
s.Baudrate=115200
out=[];%zeros(100,3);
amostra=[];
fopen(s);
pause(1);

cont_high=50;
percent_high=20;

fprintf(s,[num2str(cont_high),',',num2str(percent_high)]);

commas=[];

PWM_value=[];
vref=[];
im=[];
vpot=[];
vref_mean=[];
im_mean=[];
vpot_mean=[];



image='arduinoimage.png';
fig=figure

axisx=[];
xmax=400;

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
im_ymax=0.5;


tic
for i=1:xmax;
axisx(i,1)=i;
%flushinput(s);
%for j=1:3
out=fscanf(s);
commas=strfind(out,',');
PWM_value(i,1)=str2double(out(1:commas(1)-1))*100/180;
vref(i,1)=str2double(out(commas(1)+1:commas(2)-1));
im(i,1)=str2double(out(commas(2)+1:commas(3)-1))*100/1023;
vpot(i,1)=str2double(out(commas(3)+1:commas(4)-1))*100;
vref_mean(i,1)=str2double(out(commas(4)+1:commas(5)-1));
im_mean(i,1)=str2double(out(commas(5)+1:commas(6)-1))*100/1023;
vpot_mean(i,1)=str2double(out(commas(6)+1:commas(7)))*100;



%     %plot vref
%     subplot(3,1,1)
%     plot(axisx(end-1*sign(end-1):end),vref(end-1*sign(end-1):end),'LineWidth',2)
%     hold on
%     plot(axisx(end-1*sign(end-1):end),vref_mean(end-1*sign(end-1):end),'r','LineWidth',2)
%     axis([0 xmax vref_ymin vref_ymax]);
    
    %plot Rp
    subplot(3,1,1)
    plot(axisx(end-1*sign(end-1):end),vpot(end-1*sign(end-1):end),'LineWidth',2)
    hold on
    plot(axisx(end-1*sign(end-1):end),vpot_mean(end-1*sign(end-1):end),'r','LineWidth',2)
    axis([0 xmax Rp_ymin Rp_ymax]);
    
%     plot vim
%     subplot(4,1,3)
%     plot(axisx,vim,'LineWidth',2)
%     axis([0 xmax vim_ymin vim_ymax]);

    subplot(3,1,2)
    plot(axisx(end-1*sign(end-1):end),PWM_value(end-1*sign(end-1):end),'LineWidth',2)
    axis([0 xmax PWM_value_ymin PWM_value_ymax]);
    hold on
    
    %plot im
    subplot(3,1,3)
    plot(axisx(end-1*sign(end-1):end),im(end-1*sign(end-1):end),'LineWidth',2)
    hold on
    plot(axisx(end-1*sign(end-1):end),im_mean(end-1*sign(end-1):end),'r','LineWidth',2)
    axis([0 xmax im_ymin im_ymax]);
    drawnow
end
toc

   
    
fclose(s);

clear s
