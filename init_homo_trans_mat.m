function [mat] = init_homo_trans_mat(linear_disp,axis,ang_disp)
%UNTITLED4 Create the homogeneous transformation matrix having the
%following state wrt the base frame
%   Detailed explanation goes here
mat = zeros(4,4);
% Linear displacement component
mat(1:4,4) = [linear_disp;1];
% Angular displacement component
len_axis = norm(axis);
axis = axis/len_axis;
axis_cross_mat = [0 -axis(3,1) axis(2,1);...
                  axis(3,1) 0 -axis(1,1);...
                  -axis(2,1) axis(1,1) 0];
rot_mat = eye(3)+axis_cross_mat*sin(ang_disp)+axis_cross_mat^2*(1-cos(ang_disp));
mat(1:3,1:3) = rot_mat;
end