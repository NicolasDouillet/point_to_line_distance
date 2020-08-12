%% point_to_line_distance
% 
% Function to compute the distance
% from a point P to a line (I0,u) in the 3D space, and the coordinates
% of its projection, H.
%
% Author & support : nicolas.douillet (at) free.fr, 2019-2020.
%
%% Syntax
%
% d2H = point_to_line_distance(P, u, I0);
%
% [d2H, H] = point_to_line_distance(P, u, I0);
%
%% Description
%
% d2H = point_to_line_distance(P, u, I0) computes and return the distance
% between P and the line defined by (I0,u).
%
% [d2H, H] = point_to_line_distance(P, u, I0) also returns H, the
% orthogonal projected point of P on the line (I0,u).
%
%% See also
%
% <https://fr.mathworks.com/matlabcentral/fileexchange/73490-point-to-plane-distance?s_tid=prof_contriblnk point_to_plane_distance>
%
%% Input arguments
%
%        [ |  |  |]
% - P = [Px Py Pz], real matrix double, point or list of N points whichwe want the distance to the 3D line. size(P) = [N,3].
%        [ |  |  |]              
%
% - u = [ux uy uz], real row vector double, a director vector the line. size(u) = [1,3].
%
% - I0 = [I0x I0y I0z], real row vector double, a known point belonging to the line. size(I0) = [1,3].
%
%
%% Output arguments
%
% - d2H : real scalar or vector double, the euclidian distance between P and the line (I0,u). size(d2H) = [N,1].      
%              
%        [ |  |  |]
% - H = [xH yH zH], real matrix double, the point projected on the line, size(H) = [N,3].
%        [ |  |  |]
%
%% Example #1
% 3D

I0 = [1 1 1];
u = I0;
P = [1 0 0];
[d2H, H] = point_to_line_distance(P, u, I0) % expected distance : sqrt(2)/sqrt(3)

%% Example #2
% 2D

I0 = [1 -1];
u = I0;
P = [1 0];
[d2H, H] = point_to_line_distance(P, u, I0) % expected distance : 0.5*sqrt(2) 

%% Example #3
% Array of points

u = [3 4 0];
I0 = u;
P1 = [7 1 0];
P2 = [-2 14 0];
P = [P1; P2];
[d2H, H] = point_to_line_distance(P, u, I0) % expected distance column vector : [5; 10]