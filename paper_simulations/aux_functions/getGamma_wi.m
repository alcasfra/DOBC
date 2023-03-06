function [Gamma_wi] = getGamma_wi(Acl,Tc,i)

    sum = zeros(size(Acl));
    for k=0:1:i
        sum = sum + (Tc^k/factorial(k))*(-Acl')^(-(i+1)+k);
    end
    
    Gamma_wi = (-Acl')^(-(i+1))*expm(-Acl'*Tc) - sum;
    
end

