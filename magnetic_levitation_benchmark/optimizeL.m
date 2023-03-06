% Copyright (C) <2020>  <Alberto Castillo>
% 
%     This program is free software: you can redistribute it and/or modify
%     it under the terms of the GNU General Public License as published by
%     the Free Software Foundation, either version 3 of the License, or
%     (at your option) any later version.
% 
%     This program is distributed in the hope that it will be useful,
%     but WITHOUT ANY WARRANTY; without even the implied warranty of
%     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%     GNU General Public License for more details.
% 
%     You should have received a copy of the GNU General Public License
%     along with this program.  If not, see <https://www.gnu.org/licenses/>.



function [L,alpha] = optimizeL(A,B1,B2,C2,r,K,M,N,delta)

    % Get extended matrices
    n = size(A,1); m = size(B2,2); q = size(B1,2); p = size(C2,1);
    l = n+(r+1)*q;
    Pi  = [eye(q) zeros(q,r*q)];
    Phi = [zeros(r*q,q) eye(r*q);
           zeros(q,q)   zeros(q,r*q)];
    bA  = [A                 B1*Pi;
           zeros((r+1)*q,n)   Phi];
    bB2 = [B2; zeros((r+1)*q,m)];
    bC2 = [C2 zeros(p,(r+1)*q)];
    bB1 = [zeros(q,n+r*q) eye(q)]';

    % LMI optimization. (Requires Yalmip + Sedumi)
    ops = sdpsettings('solver', 'sedumi', 'sedumi.eps', 1e-16, 'sedumi.cg.qprec', 1, 'sedumi.cg.maxiter', 50, 'sedumi.stepdif', 2);
    P = sdpvar(l);
    Y = sdpvar(l,p,'full');
    alpha = sdpvar(1);
    Psi = [P*bA+bA'*P-Y*bC2-bC2'*Y'+2*delta*P     P*bB1
                    (P*bB1)'                   -eye(q)];

    problem = [Psi<=0,P-alpha*(K'*K)>=0,alpha>=0,... 
              kron(N,P)+kron(M,P*bA-Y*bC2)+kron(M',bA'*P-bC2'*Y')<=0];
    optimize(problem,-alpha)                
    disp('Optimizing L. Maximize alpha')
    alpha = value(alpha);

    % Observer matrix
    L = inv(value(P))*value(Y);

end

