function [c, ceq] = pathCst(z)
% [c, ceq] = pathCst(z)
%
% Computes a velocity-matching path constraint
%
% INPUTS:
%   z = [6, n] = [X; V1; V2; ... ] = state matrix
%
% OUTPUTS:
%   c = []
%   ceq = V1 - V2
%

V1 = z(1,:);
% V2 = z(1,:);
V2 = pi/2 * ones(size(z(1,:)));

% disp(V1-V2)
min_dist = 1.0;
c = min_dist - sqrt(angdiff(V1, V2).^2);
ceq = [] ;

end