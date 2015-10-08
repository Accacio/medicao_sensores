
Dca_max=0.065;   %max value that can reach Dca
Dca_min=0.005;  %min value that can be reduced Dca
Dca_step=0.001; %variation of Dca

i=0;
for Dca=Dca_min:Dca_step:Dca_max
    i=i+1;
    phi_step=30*pi/180;
    [Fcf(:,:,i),phi]=tension_calc(Dca,phi_step,10,0,0);
end

i=0;
figure
hold on
for Dca=Dca_min:(Dca_max-Dca_min)/2:Dca_max
    i=i+1;
    phi_step=30*pi/180;
    tension_calc(Dca,phi_step,10,3,i);
end
hold off

figure
x_dca=Dca_min:Dca_step:Dca_max;
for i=1:size(x_dca,2)
    for j=1:size(phi,2)
    y(i,j)=Fcf(1,j,i);
    end
end
plot(x_dca,y)