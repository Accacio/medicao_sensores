function [u] = control_law(teta_ref, a, b,incl)
%teta_ref=60;
et=-30:0.5:30;
%a=-5;
%b=8;
max_open=120;
min_open=40;
u=teta_ref+(min_open-teta_ref)*(1-1./(1+exp(-((et+4)/incl-a))))+(max_open-teta_ref)*(1./(1+exp(-((et-4)/incl-b))));
%figure
plot(et,u)
end