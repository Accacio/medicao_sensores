window=20;
aux=data2(:,1);
aux_win=aux(1:window);
new_data=aux_win;
for i=1:size(aux,1)
    aux_win=circshift(aux_win,[0,window-1]);
    aux_win(window)=aux(i);
    new_data(i)=mean(aux_win);
end
figure
plot(axisx,new_data,'r',axisx,data6.*10)
% hold on
% plot(axisx,aux,'g')
