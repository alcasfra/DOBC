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



function [Gamma_wi] = getGamma_wiAut(A,B1,Delta_t,i)
    
    sum = zeros(size(A));
    for j=0:1:80 % 30 iterations are normally enough
        sum = sum + (Delta_t)^(i+1+j)/factorial(i+1+j)*A^(j);
    end
    Gamma_wi = sum*B1;

end