x=-20:0.5:20;
b=15;
a=10;
lim_bot=150;
lim_up=20;
fx=-lim_bot*(1-1./(1+exp(-(x+b))))+lim_up*(1./(1+exp(-(x-a))));
plot(x,fx)