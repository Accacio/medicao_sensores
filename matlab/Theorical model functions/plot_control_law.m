
tolerance=1.1;
left_lim=-6.1;
right_lim=3.98;
incl=1;
aux_plot=[];

pos_i=0;
for i=50:10:120
    pos_i=pos_i+1;
    aux_plot(pos_i,:)=control_law(i,left_lim*tolerance,right_lim*tolerance,incl);
end
et=-30:0.5:30;

plot(et,aux_plot)