
function [dtheta_m_pos dtheta_m_neg sign_dtheta_m_pos sign_dtheta_m_neg] = frictionModes(dtheta_m, eps)

dtheta_m_pos = dtheta_m;
dtheta_m_neg = dtheta_m;
sign_dtheta_m_pos = dtheta_m;
sign_dtheta_m_neg = dtheta_m;

if nargin == 1
    eps = 0;
end

for i=1:length(dtheta_m)
    if(abs(dtheta_m(i))<eps)
        dtheta_m_neg(i) = 0;
        sign_dtheta_m_neg(i) = 0;
        sign_dtheta_m_pos(i) = 0;
    else if(dtheta_m(i)>=eps)
        dtheta_m_neg(i) = 0;
        sign_dtheta_m_neg(i) = 0;
        sign_dtheta_m_pos(i) = 1;
    else if(dtheta_m(i)<=eps) 
        dtheta_m_pos(i) = 0;
        sign_dtheta_m_pos(i) = 0;
        sign_dtheta_m_neg(i) = -1;
        end
        end
    end
end