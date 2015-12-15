function [u] = control_law(teta_ref, a, b,incl)
%teta_ref=60;
et=-30:0.5:30;
%a=-5;
%b=8;
max_open=120;
min_open=40;
aux_u=teta_ref-(20)*(1-1./(1+exp(-((et+4)/incl-a))))+(20)*(1./(1+exp(-((et-4)/incl-b))));
u=max(min_open,min(max_open,aux_u));
%figure
plot(et,u)
end