
gradient=0;
gradient_0=0
gradient(1,1)=0;
gradient_0(1,1)=0;
vk_2=0;
vk_1=0;
vk_0=0;
ek_1=0;
ek_0=angle_value(1);
lim=8

gradient(1,1)=0;
for i=2:size(angle_value,1)
vk_2=vk_1;
vk_1=vk_0;
ek_1=ek_0;
ek_0=angle_value(i);
aux_ek=ek_0-ek_1;
if(aux_ek==0)
    vk_0=0;
else
    if(aux_ek<0)
        vk_0=-1;
    else
        vk_0=1;
    end
end
% desition of speed state
aux_grad(i,1)=vk_2+vk_1+vk_0;

if(aux_grad(i,1)<=-2)
    gradient_0(i,1)=min(3,max(0,gradient_0(i-1)-1));
    gradient(i,1)=min(lim,max(-lim,(gradient(i-1,1)-(2-gradient_0(i,1)))));
else
    if(aux_grad(i,1)>=2)
    gradient_0(i,1)=min(3,max(0,gradient_0(i-1)-1));
    gradient(i,1)=min(lim,max(-lim,(gradient(i-1,1)+(2-gradient_0(i,1)))));
    else
        gradient(i,1)=gradient(i-1,1)-sign(gradient(i-1,1));
        gradient_0(i,1)=min(3,max(0,gradient_0(i-1)+1));
    end
end
end
        
plot(axisx,angle_value,axisx,(100+sign(gradient)*20))