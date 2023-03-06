% Trapezoidal integration of expm(Acl'*(s-t))*S_T*B1*w(s).
% Computes the integral from 't' to 't+h' with step size 'delta'

function [out] = get_disturb_delta1_integral(Acl, S_T, B1, t, h, delta)

    % Compute the integrand at s=t, s=t+delta, ..., s=t+N*delta
    N = round(h/delta);
    integrand = [];
    for s=t:delta:(t+N*delta)
        integrand = horzcat(integrand,expm(Acl'*(s-t))*S_T*B1*ddw(s));
    end
    
    % Compute the trapezoidal integration
    integral = trapz(delta,integrand')';
    
    out = integral;
    
end

