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



function [Kx,Kw] = getKxKw(A,B1,B2,C1,R,gam,r)

    % Compute matrices
    S_T = care(A,B2,C1'*C1,gam*R); % Solve ARE (3.2).
    Kx = -inv(gam*R)*B2'*S_T;
    Kv = -inv(gam*R)*B2';
    Acl = A+B2*Kx;
    Phi = [];
    for i = 1:1:r+1
        Phi = horzcat(Phi,(-Acl')^(-i)*S_T*B1);
    end
    Kw = Kv*Phi;
    
end

