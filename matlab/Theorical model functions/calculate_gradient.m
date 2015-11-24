function [gradient, grad_state] = calculate_gradient(angle_value, grad_state)
%Function used to calculate the gradient of the curve
%Calculated from the gradient state vector (1x3) which saves the state of the past gradient
%and the actual angle position.

% gradient=0;
% gradient(1,1)=0;
% ek_1=0;
ek_1=grad_state.ek_1;
ek_0=angle_value;
grad_0=grad_state.state(1);
grad_posi=grad_state.state(2);
grad_nega=grad_state.state(3);

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

grad_0=min(grad_state.quantizer,max(0,grad_0));
grad_posi=min(grad_state.quantizer,max(0,grad_posi));
grad_nega=min(grad_state.quantizer,max(0,grad_nega));
% monitor_grad(i,1)=grad_0;
% monitor_grad(i,2)=grad_posi;
% monitor_grad(i,3)=grad_nega;

if(and(grad_0>=grad_posi,grad_0>=grad_nega))
    gradient=0;
else
    if(and(grad_posi>=grad_0,grad_posi>=grad_nega))
        gradient=1;
    else
        gradient=-1;
    end
end

grad_state.state(1)=grad_0;
grad_state.state(2)=grad_posi;
grad_state.state(3)=grad_nega;
grad_state.ek_1=ek_0;
end

