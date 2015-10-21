clear
%settings
cont_high=400;
percent_high=95;
xmax=2400; %8000 Para 10 iterations % 2400 3
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
fprintf(s,[num2str(cont_high),',',num2str(percent_high)]);
pause(1);
count_plot=0;

tic
for i=1:xmax+1;
axisx(i,1)=i;
%flushinput(s);
%for j=1:3
out=fscanf(s);
commas=strfind(out,',');
if numel(commas)<6
PWM_value(i,1)=PWM_value(i-1,1);
%vref(i,1)=str2double(out(commas(1)+1:commas(2)-1));
%im(i,1)=str2double(out(commas(2)+1:commas(3)-1))*100/1023;
vpot(i,1)=vpot(i-1,1);
%vref_mean(i,1)=str2double(out(commas(4)+1:commas(5)-1));
%im_mean(i,1)=str2double(out(commas(5)+1:commas(6)-1))*100/1023;
vpot_mean(i,1)=vpot_mean(i-1,1);
loadcell(i,1)=loadcell(i-1,1);
loadcell_mean(i,1)=loadcell_mean(i-1,1);
sampl_time=sampl_time+sampl_time/i-1;

commas_error=commas_error+1;
else
PWM_value(i,1)=str2double(out(1:commas(1)-1))*100/180;
%vref(i,1)=str2double(out(commas(1)+1:commas(2)-1));
vpot(i,1)=str2double(out(commas(1)+1:commas(2)-1));
vpot_mean(i,1)=str2double(out(commas(2)+1:commas(3)-1));
%vref_mean(i,1)=str2double(out(commas(4)+1:commas(5)-1));
im(i,1)=str2double(out(commas(3)+1:commas(4)-1));
im_mean(i,1)=str2double(out(commas(4)+1:commas(5)-1));
loadcell(i,1)=str2double(out(commas(5)+1:commas(6)-1));
loadcell_mean(i,1)=str2double(out(commas(6)+1:commas(7)-1));
sampl_time=sampl_time+str2double(out(commas(7)+1:commas(8)-1));
end

%     %plot vref
%     subplot(3,1,1)
%     plot(axisx(end-1*sign(end-1):end),vref(end-1*sign(end-1):end),'LineWidth',2)
%     hold on
%     plot(axisx(end-1*sign(end-1):end),vref_mean(end-1*sign(end-1):end),'r','LineWidth',2)
%     axis([0 xmax vref_ymin vref_ymax]);

    %plot Rp
  if count_plot==10
    count_plot=0;
    subplot(2,2,1)
    plot(axisx(end-11*sign(end-11):end),vpot(end-11*sign(end-11):end),'LineWidth',2)
    hold on
    plot(axisx(end-11*sign(end-11):end),vpot_mean(end-11*sign(end-11):end),'r','LineWidth',2)
    axis([0 xmax Rp_ymin Rp_ymax]);

%     plot vim
%     subplot(4,1,3)
%     plot(axisx,vim,'LineWidth',2)
%     axis([0 xmax vim_ymin vim_ymax]);

    subplot(2,2,2)
    plot(axisx(end-11*sign(end-11):end),PWM_value(end-11*sign(end-11):end),'LineWidth',2)
    axis([0 xmax PWM_value_ymin PWM_value_ymax]);
    hold on

    %plot im
    subplot(2,2,3)
    plot(axisx(end-11*sign(end-11):end),im(end-11*sign(end-11):end),'LineWidth',2)
    hold on
    plot(axisx(end-11*sign(end-11):end),im_mean(end-11*sign(end-11):end),'r','LineWidth',2)
    axis([0 xmax im_ymin im_ymax]);

    %plot loadcell
    subplot(2,2,4)
    plot(axisx(end-11*sign(end-11):end),loadcell(end-11*sign(end-11):end),'LineWidth',2)
    hold on
    plot(axisx(end-11*sign(end-11):end),loadcell_mean(end-11*sign(end-11):end),'r','LineWidth',2)
    axis([0 xmax loadcell_ymin loadcell_ymax]);

    drawnow
  else
  count_plot=count_plot+1;
  end
end
toc

% Mean of im values

iterator=0;
IM=[];
IM_MEAN=[];
IM=[];
IM_MEAN=[];
while iterator<xmax;
    iterator=iterator+cont_high/2;
    IM=[IM im(iterator-180:iterator-30,1)];
    IM_MEAN=[IM_MEAN im_mean(iterator-180:iterator-30,1)];
end
Mean_im=sum(IM)/151;
Mean_im_mean=sum(IM_MEAN)/151;
iterator=5;
while iterator<=size(Mean_im,2);

   Mean_im(floor((iterator-1)/4)+1,mod(iterator,4)+4*(mod(iterator,4)==0))=Mean_im(1,iterator);
   Mean_im(1,iterator)=0;
   Mean_im_mean(floor((iterator-1)/4)+1,mod(iterator,4)+4*(mod(iterator,4)==0))=Mean_im_mean(1,iterator);
   Mean_im_mean(1,iterator)=0;
   iterator=iterator+1;
end
clear iterator, IM, IM_MEAN;

%%%%%%%%%%%%%%%%%%%%%%%%%%%

data.samp_t=sampl_time/(1000*xmax)
commas_error
data.t=0:sampl_time/1000:(xmax*sampl_time)/1000;
data.input=PWM_value;
data.output=vpot;
aux='0,0';
fwrite(s,aux);
pause(1);
out=fscanf(s);




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
