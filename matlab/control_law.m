function [u] = control_law(teta_ref, a, b)
%teta_ref=60;
et=-20:0.5:20;
%a=-5;
%b=8;
max_open=140;
min_open=40;
u=teta_ref+(min_open-teta_ref)*(1-1./(1+exp(-(et-a+3))))+(max_open-teta_ref)*(1./(1+exp(-(et-b-3))));
plot(et,u)
end