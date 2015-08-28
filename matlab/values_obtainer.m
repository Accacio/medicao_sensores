cut=900;
max=1800;

t=1:max-cut;
for i=cut+1:max
    x(i-cut)=PWM_value(i);
    y_filter(i-cut)=vpot(i);
    y_mean(i-cut)=vpot_mean(i);    
end
figure
plot(t,x,t,y_filter,t,y_mean)
