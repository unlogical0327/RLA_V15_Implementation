% This function finds the optimal Rigid/Euclidean transform in 2D space
% It expects as input a Nx2 matrix of 2D points.
% It returns R, T
% expects row data
function [R,t] = rigid_transform_2D(A, B)
if nargin ~= 2
    error('Missing parameters');
end
A;
B;
centroid_A = mean(A);  %
centroid_B = mean(B);
N=size(A,1);
H=(A-repmat(centroid_A,N,1))'*(B-repmat(centroid_B,N,1));
[U,S,V] = svd(H);
R = V*U';

if det(R) < 0
    disp('Reflection detected\n');
    V(:,2)= -V(:,2);
    %V(:,2)= -1;
    R = V*U';
end

t = -R*centroid_A' + centroid_B';
end
