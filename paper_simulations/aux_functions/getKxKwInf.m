function [Kx,KvPhi] = getKxKwInf(A,B1,B2,Q,R,r)
    
    % Compute auxiliar matrices
    S_T = care(A,B2,Q,R); % Solve ARE (3.2).
    Acl = A-B2*inv(R)*B2'*S_T;
    PhiInf = [];
    for i = 1:1:r+1
        PhiInf = horzcat(PhiInf,(-Acl')^(-i)*S_T*B1);
    end
    
    Kx = -inv(R)*B2'*S_T;
    KvPhi = -inv(R)*B2'*PhiInf;
    
end

