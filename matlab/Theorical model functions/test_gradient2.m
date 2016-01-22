gradient=0;
gradient(1,1)=0;
ek_1=0;
ek_0=angle_value(1);
grad_0=7;
grad_posi=0;
grad_nega=0;

for i=2:size(angle_value,1)
ek_1=ek_0;
ek_0=angle_value(i);
aux_ek=ek_0-ek_1;
if(aux_ek==0)
    grad_0=grad_0+1;
    if(grad_posi>0)
        grad_posi=grad_posi-1;
    else
        grad_nega=grad_nega-1;
    end
else
    if(aux_ek<0)
        grad_nega=grad_nega+1;
        if(grad_posi>0)
            grad_posi=grad_posi-1;
        else
            grad_0=grad_0-1;
        end
    else
        grad_posi=grad_posi+1;
        if(grad_nega>0)
            grad_nega=grad_nega-1;
        else
            grad_0=grad_0-1;
        end
    end
end

grad_0=min(7,max(0,grad_0));
grad_posi=min(7,max(0,grad_posi));
grad_nega=min(7,max(0,grad_nega));
% monitor_grad(i,1)=grad_0;
% monitor_grad(i,2)=grad_posi;
% monitor_grad(i,3)=grad_nega;

if(and(grad_0>=grad_posi,grad_0>=grad_nega))
    gradient(i,1)=0;
else
    if(and(grad_posi>=grad_0,grad_posi>=grad_nega))
        gradient(i,1)=1;
    else
        gradient(i,1)=-1;
    end
end
        
end
plot(axisx,angle_value,axisx,gradient*20+100)

