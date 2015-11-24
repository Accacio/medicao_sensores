n=4;
aux_angle=angle_value;

for i=n:size(angle_value,1)
aux_angle(i,1)=sum(aux_angle(i-n+1:i,1))/n;
end
plot(axisx,angle_value,axisx,aux_angle)