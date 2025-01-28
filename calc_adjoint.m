function [adj] = calc_adjoint(A_g_B)
%UNTITLED2 Calculate the adjoint of a transformation matrix
%   Detailed explanation goes here
adj = zeros(6,6);
rot_mat = A_g_B(1:3,1:3);
lin_disp = A_g_B(1:3,4);
cross_mat_lin_disp = [0 -lin_disp(3,1) lin_disp(2,1);...
                      lin_disp(3,1) 0 -lin_disp(1,1);...
                      -lin_disp(2,1) lin_disp(1,1) 0];
adj = [rot_mat cross_mat_lin_disp*rot_mat;...
       zeros(3,3) rot_mat];
end