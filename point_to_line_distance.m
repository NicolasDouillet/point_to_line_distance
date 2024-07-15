function [d2H, H] = point_to_line_distance(P, u, I0)
%% point_to_line_distance : function to compute the distance
% between the 3D point P and the line (I0,u) in the 3D space, and the coordinates
% of its projection, H.
%
% Author : nicolas.douillet9 (at) gmail.com, 2019-2024.
%
%
% Syntax
%
% d2H = point_to_line_distance(P, u, I0);
% [d2H, H] = point_to_line_distance(P, u, I0);
%
%
% Description
%
% d2H = point_to_line_distance(P, u, I0) computes and return the distance
% between P and the line (I0,u).
%
% [d2H, H] = point_to_line_distance(P, u, I0) also returns H, the
% orthogonal projected point of P on the line (I0,u).
%
%
% Input arguments
%
%       [ |  |  |]
% - P = [Px Py Pz], real matrix double, point or list of N points whichwe want the distance to the 3D line. size(P) = [N,3].
%       [ |  |  |]      
%
% - u = [ux uy uz], real row vector double, a director vector the line. size(u) = [1,3].
%
% - I0 = [I0x I0y I0z], real row vector double, a known point belonging to the line. size(I0) = [1,3].
%
%
% Output arguments
%
% - d2H : real scalar or vector double, the euclidian distance between P and the line (I0,u). size(d2H) = [N,1].      
%              
%       [ |  |  |]
% - H = [xH yH zH], real matrix double, the point projected on the line, size(H) = [N,3].
%       [ |  |  |]
%
%
% Example #1
% 3D
% I0 = [1 1 1];
% u = I0;
% P = [1 0 0];
% [d2H, H] = point_to_line_distance(P, u, I0) % expected distance : sqrt(2)/sqrt(3)
%
%
% Example #2
% 2D
% I0 = [1 -1 0];
% u = I0;
% P = [1 0 0];
% [d2H, H] = point_to_line_distance(P, u, I0) % expected distance : 0.5*sqrt(2)
%
%
% Example #3
% Array of points
% u = [3 4 0];
% I0 = u;
% P1 = [7 1 0];
% P2 = [-5 10 0];
% P = [P1; P2];
% [d2H, H] = point_to_line_distance(P, u, I0) % expected distance column vector : [5; 10]


%% Inputs parsing
assert(nargin > 2,'Not enough input arguments.');
assert(nargin < 4,'Too many input arguments.')

nb_pts = size(P,1);

assert((isequal(size(u),size(I0),[1,3]) || ...
        isequal(size(u),size(I0),[1,2])) && ...        
        isequal(ndims(P),ndims(u),2),...
        'Inputs u and P must have the same format (size, number of elments, and be of dimension 2).');
    
assert(size(u,2) == size(P,2),'Inputs P, u, and I0 must avec the same number of columns.');
assert(isreal(P) && isreal(u) && isreal(I0),'Input argument P, u, and I0 must be real.');


%% Body
t_H = (u(1)*(P(:,1)-repmat(I0(1),[nb_pts,1])) + ...
       u(2)*(P(:,2)-repmat(I0(2),[nb_pts,1])) + ...
       u(3)*(P(:,3)-repmat(I0(3),[nb_pts,1])) ) / ...
       sum(u.^2); 

x_H = I0(1) + t_H*u(1);
y_H = I0(2) + t_H*u(2);
z_H = I0(3) + t_H*u(3);


% Orthogonal projected point
H = zeros(size(P));
H(:,1) = x_H;
H(:,2) = y_H;
H(:,3) = z_H;


% Distance
d2H = vecnorm((P-H)',2)';
H = H(:,1:size(P,2));


end % point_to_line_distance