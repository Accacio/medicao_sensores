
min_phi=40;
max_phi=140;
var_phi=5;
phi_vect=min_phi:var_phi:max_phi;

Wf=1.5;
Dca=0.057;
Dcf=0.047;
F_dinamic=0;

for i=1:size(phi_vect,2)
    F_dinamic(i)=tension_calc(Dca,Dcf,phi_vect(i)*pi/180,Wf);
end
plot(phi_vect,F_dinamic)