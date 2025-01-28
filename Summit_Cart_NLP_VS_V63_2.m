clc
clear all
close all
% Adapt Fatrop to reduce jacobian computation.
%% Define Cost function weights
% Cost for each objective not met
Q = [0.002;... % Cost for larger inputs (not MB joints)
     0.01;... % Cost for larger time-steps
     1;... % Deviation from planned trajectory orientation
     100;... % Cost of missed OP
     1; ... 
     0.01]; % Penalty for the cart-origin from deviating from its planned-path

%% Set up simulation capture
video = true;
optimize_and_record = true;
file_name = strcat('Summit_Cart_SotA_OP_scan_V63_2_time_step_cost_',strrep(num2str(Q(2,1)),'.','_'),'_orient_dev_cost_',strrep(num2str(Q(3,1)),'.','_'),'_cart_dev_cost_',strrep(num2str(Q(5,1)),'.','_'));
% Single wide
sw_position_1 =[900 100 1000 450];
sw_position_2 =[900 100 1000 450];
sw_position_3 =[900 100 1000 400];
% Double wide
dw_position =[900 100 1000*2 550*2];
if(optimize_and_record)
    % delete any old diary
    if exist(strcat(file_name,'.txt'), 'file')==2
      delete(strcat(file_name,'.txt'));
    end
    % setup diary to record output
    diary(strcat(file_name,'.txt'))
    diary on
end
file_name

%% Define the Link lengths and screw axis
%% Summit Initial Conditions
summ_lin_dis_init = [0;0;0]; % Location of Summit w.r.t world frame
summ_pose_axis = [0;0;1]; % Orientation axis of Summit w.r.t world frame
summ_ang_dis_init = 0; % Orientation angle of Summit
mat_w_G_s_init = init_homo_trans_mat(summ_lin_dis_init,summ_pose_axis,summ_ang_dis_init);
% Location of points of Interest w.r.t body origin
% Summit (w.r.t COM)
summit_length = 0.62; % m
summit_width = 0.43; % m
summit_height = 0.56; % m
summit_color = [158/255 158/255 158/255];
summit_name = 'S';
summit = struct();
summit.Link_length = summit_length;
summit.Link_width = summit_width;
summit.Link_height = summit_height;
summit.Link_color = summit_color;
summit.Link_name = summit_name;
Point_Summit_1 = [-(summit_length/2-summit_width/2);0;0];
Point_Summit_2 = [(summit_length/2-summit_width/2);0;0];
Radius_Summit = (summit_length/2-Point_Summit_2(1,1))*sqrt(2);
% Initial state
x_init = [-1 0.5 0 0 0 0];
% Summit goal point
Summit_origin_goal_point = [2;...
                            0.5;...
                            0];
% Cart (w.r.t to rear axle mid-point frame)
cart_Link_length = 0.84; % m
cart_Link_width = 0.6; % m
cart_Link_height_vis = 0.54; % m
cart_Link_color = [0 0.4470 0.7410];
cart_Link_name = 'C0';
cart = struct();
cart.Link_length = cart_Link_length;
cart.Link_width = cart_Link_width;
cart.Link_height = cart_Link_height_vis;
cart.Link_color = cart_Link_color;
cart.Link_name = cart_Link_name;
num_of_points_cart = 3;
dist_cart = cart_Link_width/num_of_points_cart/2;
Point_Cart_1 = [cart_Link_length-dist_cart;dist_cart*2;0];
Point_Cart_2 = [cart_Link_length-dist_cart;0;0];
Point_Cart_3 = [cart_Link_length-dist_cart;-dist_cart*2;0];
Radius_Cart = dist_cart*sqrt(2);
%% WAM Initial condition
% displacement of WAM base (Base Link) from Summit origin
link_s_0_delta_lin = [0.14;0;0.405];
link_s_0_pose_axis = [0;0;1];
link_s_0_angle = 0;
mat_s_G_0_init = init_homo_trans_mat(link_s_0_delta_lin,link_s_0_pose_axis,link_s_0_angle);
mat_w_G_0_init = mat_w_G_s_init*mat_s_G_0_init;
%%   WAM link and Joint details
% Link 0 is attached to the Summit base
% Link 0: Base Link
sim_link_0_1_delta_lin = [0;0;0.346];
link_0_1_pose_axis = [0;0;1]; % w.r.t Link 0
link_0_1_angle = 0;
% Create homogeneous transformation matrices for base case
mat_0_G_1_init = init_homo_trans_mat(sim_link_0_1_delta_lin,link_0_1_pose_axis,link_0_1_angle);
mat_w_G_1_init = mat_w_G_0_init*mat_0_G_1_init; % This gives the pose of the 'Beam Link'
link_w_1_joint_axis = mat_w_G_0_init(1:3,1:3)*link_0_1_pose_axis;
link_w_1_joint_q_homo = mat_w_G_1_init*[0;0;0;1];
link_w_1_joint_q = link_w_1_joint_q_homo(1:3,1);

%% Cart
% Cart Handle Base
cart_handle_base_pos_init = [-0.6851;0;0.22]; % Location of the cart Handle base in world frame
cart_handle_base_pose_axis = [0;0;1];
cart_handle_base_ang_init = 0;
cart_handle_base_pose_init = init_homo_trans_mat(cart_handle_base_pos_init,cart_handle_base_pose_axis,cart_handle_base_ang_init);

% W.r.t cart handle base
cart_rear_axle_pos = [-cart_Link_length;0;0];
cart_rear_axle_pose_axis = [0;0;1];
cart_rear_axle_ang = 0;
cart_rear_axle_pose = init_homo_trans_mat(cart_rear_axle_pos,cart_rear_axle_pose_axis,cart_rear_axle_ang);

cart_COM_pos = [-cart_Link_length/2;0;0];
cart_COM_pose_axis = [0;0;1];
cart_COM_ang = 0;
cart_COM_pose = init_homo_trans_mat(cart_COM_pos,cart_COM_pose_axis,cart_COM_ang);

% Angle made by cart handle w.r.t cart x-axis
cart_handle_length = 0.531;
cart_handle_theta = asin((mat_w_G_1_init(3,4)-cart_handle_base_pos_init(3,1))/cart_handle_length);
% The above angle is w.r.t. the opposite initial direction of the y-axis of 
% the Cart Origin Frame (0,-1,0) as opposed to the y-axis direction
% (0,1,0). As a result, when the Cart Handles' orientation is to be set, 
% the negative of this angle has to be used when dealing with 
% Cart Handle
cart_handle_pos_init = [0;0;0]; % Location of the cart Handle base in world frame
cart_handle_pose_axis = [0;1;0];
cart_handle_ang_init = -cart_handle_theta;
cart_handle_pose_init = init_homo_trans_mat(cart_handle_pos_init,cart_handle_pose_axis,cart_handle_ang_init);

% Cart Handle Handle
cart_handle_handle_pos_init = [cart_handle_length;0;0]; % Location of the cart Handle base in world frame
cart_handle_handle_pose_axis = [0;1;0];
cart_handle_handle_ang_init = cart_handle_theta;
cart_handle_handle_pose_init = init_homo_trans_mat(cart_handle_handle_pos_init,cart_handle_handle_pose_axis,cart_handle_handle_ang_init);
cart_handle_handle_pose_init_1 = init_homo_trans_mat(cart_handle_handle_pos_init,cart_handle_handle_pose_axis,0);

% Cart Handle COM
cart_handle_COM_pos_init = [cart_handle_length/2;0;0]; % Location of the cart Handle base in world frame
cart_handle_COM_pose_axis = [0;1;0];
cart_handle_COM_ang_init = 0;
cart_handle_COM_pose_init = init_homo_trans_mat(cart_handle_COM_pos_init,cart_handle_COM_pose_axis,cart_handle_COM_ang_init);

% Adjoint transformation from Cart Origin to Cart Handle
adjoint_Cart_Origin_Handle = calc_adjoint_inv(cart_rear_axle_pose\(cart_handle_pose_init*cart_handle_handle_pose_init));

mat_w_G_c0_init = cart_handle_base_pose_init*cart_rear_axle_pose;
mat_w_G_c1_init = cart_handle_base_pose_init;
mat_w_G_c2_init = cart_handle_base_pose_init*cart_handle_pose_init;
mat_w_G_c3_init = cart_handle_base_pose_init*cart_handle_pose_init*cart_handle_handle_pose_init;
mat_w_G_c4_init = cart_handle_base_pose_init*cart_COM_pose;
mat_w_G_c5_init = cart_handle_base_pose_init*cart_handle_pose_init*cart_handle_handle_pose_init_1;

%%  Continue Summit-WAM
sim_link_2_3_delta_lin = [-0.14;0;0]; % Length of the Palm Link
% Determine where the wrist should be positioned so that the WAM would
% remain horizontal when grasping the cart handle
delta_x = cart_handle_base_pos_init(1,1)+cart_handle_length*cos(cart_handle_theta)+link_s_0_delta_lin(1,1)+sim_link_2_3_delta_lin(1,1);

% Link 1: WAM Beam Link
sim_link_1_2_delta_lin = [delta_x;0;0]; % Length where the Wrist Link starts
link_1_2_pose_axis = [0;0;1]; % w.r.t link 1
link_1_2_angle = 0;
% Create homogeneous transformation matrices for base case
mat_1_G_2_init = init_homo_trans_mat(sim_link_1_2_delta_lin,link_1_2_pose_axis,link_1_2_angle);
mat_w_G_2_init = mat_w_G_1_init*mat_1_G_2_init; % This gives the pose of the origin of the 'Wrist Link'
link_w_2_joint_axis = mat_w_G_1_init(1:3,1:3)*link_1_2_pose_axis;
link_w_2_joint_q_homo = mat_w_G_2_init*[0;0;0;1];
link_w_2_joint_q = link_w_2_joint_q_homo(1:3,1);
% This is the linear displacement joint axis
link_w_2_linear_joint_axis = mat_w_G_1_init(1:3,1:3)*[1;0;0];

% Link 2: Wrist Link

link_2_3_pose_axis = [1;0;0];
link_2_3_angle = 0;
% Create homogeneous transformation matrices for base case
mat_2_G_3_init = init_homo_trans_mat(sim_link_2_3_delta_lin,link_2_3_pose_axis,link_2_3_angle);
mat_w_G_3_init = mat_w_G_2_init*mat_2_G_3_init; % This gives the pose of the surface of the 'Palm Link'

% Adjoint transformation from Palm Surface to Palm Link Origin (at wrist
% joint)
adjoint_3_2 = calc_adjoint_inv(mat_2_G_3_init);
% Adjoint transformation from Cart Origin to Palm Link Origin (at wrist
% joint)
adjoint_Cart_origin_2 = calc_adjoint(cart_rear_axle_pose\(cart_handle_pose_init*cart_handle_handle_pose_init));
% Determine all initial joint twists w.r.t world frame
% Summit
twist_lin_x_summit = [1;0;0;0;0;0];
twist_lin_y_summit = [0;1;0;0;0;0];
twist_ang_z_summit = [0;0;0;0;0;1];
% This twist is for swinging the 'Beam Link' of the WAM left and right. 
twist_0_1_revolute_base = create_revolute_twist(link_w_1_joint_axis,link_w_1_joint_q);
% This twist is for extending the reach of the Wrist Link
twist_1_2_linear_base = [link_w_2_linear_joint_axis;zeros(3,1)];
% This twist is for swinging the 'Palm Link' of the Wrist Link Left and right.
twist_1_2_revolute_z_base = create_revolute_twist(link_w_2_joint_axis,link_w_2_joint_q);
% Cart
twist_lin_x_cart = [1;0;0;0;0;0];
twist_lin_y_cart = [0;1;0;0;0;0];
twist_ang_z_cart = [0;0;0;0;0;1];

%% Plot Current Summit Cart
fig_1 = figure(1);
% Plot Summit
% Link Summit
summit_origin = plot_Summit_2D(eye(4),mat_w_G_s_init,summit);
%plot_link(eye(4),eye(4),link_s_0_delta_lin,'S')
% Link 0: Base Link
plot_link(eye(4),mat_w_G_0_init,sim_link_0_1_delta_lin,'S0','k')
% Link 1: 'Beam'
plot_link(eye(4),mat_w_G_1_init,sim_link_1_2_delta_lin,'S1','k')
% Link 2: 'Wrist'
plot_link(eye(4),mat_w_G_2_init,[0.35;0;0],'S2','k')
% Link 3: 'Hand Palm'
plot_link(eye(4),mat_w_G_3_init,-sim_link_2_3_delta_lin,'S3','k')

% Plot Cart
% Link Cart
cart_origin = plot_Summit_2D(mat_w_G_c4_init,eye(4),cart);
% Link Handle Base
plot_link(eye(4),mat_w_G_c1_init,[0;0;0],'C1','k')
% Link Handle
plot_link(eye(4),mat_w_G_c2_init,cart_handle_handle_pos_init,'C2','k')
% Link Handle Handle
plot_link(eye(4),mat_w_G_c3_init,[0;0;0],'C3','k')

title('Motion of Summit-WAM-Cart ') %Graph title
axis equal
xlabel('X Axis')
ylabel('Y Axis')
zlabel('Z Axis')
set(gca,'fontsize',20)
view(2)
grid on
grid minor
hold off
%     axis([-1 1 -1 1 -1 1])
axis([-4 4 -2 6 -2 2])
% close(fig_1)
% pause(0.1)
%% Dynamic Properties of system
% Gravity
g = [0;0;-9.81;0]; % m/sec^2
% Summit
Summit_mass = 100; % kg
Summit_height = 0.55; % m
Summit_I_xx_COM = 1/12*Summit_mass*(summit_width^2+Summit_height^2);
Summit_I_yy_COM = 1/12*Summit_mass*(summit_length^2+Summit_height^2);
Summit_I_zz_COM = 1/12*Summit_mass*(summit_length^2+summit_width^2);
% Inertia tensor defined at the CoM
Summit_I_COM = [Summit_I_xx_COM 0 0;...
                0 Summit_I_yy_COM 0;...
                0 0 Summit_I_zz_COM];
% We want to define the Inertia tensor at the Origin of the Link, so, we
% need to move the Inrtia tensor from the CoM to the Link origin
Summit_Origin_COM_vec = [0;0;Summit_height/2]; % Point to the COM from the position where we want to identify the Inertia Tensor
Summit_Origin_COM_vec_cross = [0 -Summit_Origin_COM_vec(3,1) Summit_Origin_COM_vec(2,1);...
                               Summit_Origin_COM_vec(3,1) 0 -Summit_Origin_COM_vec(1,1);...
                               -Summit_Origin_COM_vec(2,1) Summit_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Summit_GI_Origin = [Summit_mass*eye(3) -Summit_mass*Summit_Origin_COM_vec_cross;...
                    Summit_mass*Summit_Origin_COM_vec_cross Summit_I_COM-Summit_mass*Summit_Origin_COM_vec_cross^2];

% Base Link
Base_Link_mass = 10; % kg
Base_Link_length = 0.35; % m
Base_Link_width = 0.28; % m
Base_Link_height = 0.44; % m
Base_Link_I_xx_COM = 1/12*Base_Link_mass*(Base_Link_width^2+Base_Link_height^2);
Base_Link_I_yy_COM = 1/12*Base_Link_mass*(Base_Link_length^2+Base_Link_height^2);
Base_Link_I_zz_COM = 1/12*Base_Link_mass*(Base_Link_length^2+Base_Link_width^2);
Base_Link_I_COM = [Base_Link_I_xx_COM 0 0;...
                   0 Base_Link_I_yy_COM 0;...
                   0 0 Base_Link_I_zz_COM];
Base_Link_Origin_COM_vec = [-0.035;0;0.206]; % Point to the COM from the position where we want to identify the Inertia Tensor
Base_Link_Origin_COM_vec_cross = [0 -Base_Link_Origin_COM_vec(3,1) Base_Link_Origin_COM_vec(2,1);...
                                  Base_Link_Origin_COM_vec(3,1) 0 -Base_Link_Origin_COM_vec(1,1);...
                                  -Base_Link_Origin_COM_vec(2,1) Base_Link_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Base_Link_GI_Origin = [Base_Link_mass*eye(3) -Base_Link_mass*Base_Link_Origin_COM_vec_cross;...
                       Base_Link_mass*Base_Link_Origin_COM_vec_cross Base_Link_I_COM-Base_Link_mass*Base_Link_Origin_COM_vec_cross^2];

% Beam Link
Beam_Link_mass = 5.6; % kg
Beam_Link_length = 0.35; % m
Beam_Link_radius = 0.06; % m
Beam_Link_I_xx_COM = 0.5*Beam_Link_mass*Beam_Link_radius^2;
Beam_Link_I_yy_COM = Beam_Link_mass*(0.25*Beam_Link_radius^2+1/12*Beam_Link_length^2);
Beam_Link_I_zz_COM = Beam_Link_I_yy_COM;
Beam_Link_I_COM = [Beam_Link_I_xx_COM 0 0;...
                   0 Beam_Link_I_yy_COM 0;...
                   0 0 Beam_Link_I_zz_COM];
Beam_Link_Origin_COM_vec = [-0.1;0;0.0]; % Point to the COM from the position where we want to identify the Inertia Tensor
Beam_Link_Origin_COM_vec_cross = [0 -Beam_Link_Origin_COM_vec(3,1) Beam_Link_Origin_COM_vec(2,1);...
                                  Beam_Link_Origin_COM_vec(3,1) 0 -Beam_Link_Origin_COM_vec(1,1);...
                                  -Beam_Link_Origin_COM_vec(2,1) Beam_Link_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Beam_Link_GI_Origin = [Beam_Link_mass*eye(3) -Beam_Link_mass*Beam_Link_Origin_COM_vec_cross;...
                       Beam_Link_mass*Beam_Link_Origin_COM_vec_cross Beam_Link_I_COM-Beam_Link_mass*Beam_Link_Origin_COM_vec_cross^2];

% Combined Base+Beam Link
% Location of Base Link CoM w.r.t Base Origin
Base_Link_COM_Base_Origin = [Base_Link_Origin_COM_vec;1];
% Location of Beam Link CoM w.r.t Base Origin
Beam_Link_COM_Base_Origin = mat_0_G_1_init*[Base_Link_Origin_COM_vec;1];
% COM Location
Base_Beam_Combined_mass = Base_Link_mass+Beam_Link_mass;
Combined_COM_loc_Base_Origin = (Base_Link_mass*Base_Link_COM_Base_Origin+...
                                  Beam_Link_mass*Beam_Link_COM_Base_Origin)/Base_Beam_Combined_mass;

% Base COM loc w.r.t COM loc
COM_to_Base_COM_vec = Base_Link_COM_Base_Origin(1:3,1)-Combined_COM_loc_Base_Origin(1:3,1);
COM_to_Base_COM_vec_cross = [0 -COM_to_Base_COM_vec(3,1) COM_to_Base_COM_vec(2,1);...
                             COM_to_Base_COM_vec(3,1) 0 -COM_to_Base_COM_vec(1,1);...
                             -COM_to_Base_COM_vec(2,1) COM_to_Base_COM_vec(1,1) 0];
% Relocate Summit Inertia Tensor to new COM
new_Base_Inertia_Tensor = Base_Link_I_COM-Base_Link_mass*COM_to_Base_COM_vec_cross^2;
% Beam COM loc w.r.t COM loc
COM_to_Beam_COM_vec = Beam_Link_COM_Base_Origin(1:3,1)-Combined_COM_loc_Base_Origin(1:3,1);
COM_to_Beam_COM_vec_cross = [0 -COM_to_Beam_COM_vec(3,1) COM_to_Beam_COM_vec(2,1);...
                             COM_to_Beam_COM_vec(3,1) 0 -COM_to_Beam_COM_vec(1,1);...
                             -COM_to_Beam_COM_vec(2,1) COM_to_Beam_COM_vec(1,1) 0];
% Relocate Summit Inertia Tensor to new COM
new_Beam_Inertia_Tensor = Beam_Link_I_COM-Beam_Link_mass*COM_to_Beam_COM_vec_cross^2;
% Combined Inertia Tensor
Combined_Base_Beam_Inertia_Tensor = new_Base_Inertia_Tensor+...
                                    new_Beam_Inertia_Tensor;

Combined_Inertia_Tensor_Base_Origin_COM_vec = Combined_COM_loc_Base_Origin(1:3,1)-[0;0;0]; % Point to the COM from the position where we want to identify the Inertia Tensor
Combined_Inertia_Tensor_Base_Origin_COM_vec_cross = [0 -Combined_Inertia_Tensor_Base_Origin_COM_vec(3,1) Combined_Inertia_Tensor_Base_Origin_COM_vec(2,1);...
                                                     Combined_Inertia_Tensor_Base_Origin_COM_vec(3,1) 0 -Combined_Inertia_Tensor_Base_Origin_COM_vec(1,1);...
                                                     -Combined_Inertia_Tensor_Base_Origin_COM_vec(2,1) Combined_Inertia_Tensor_Base_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Combined_Inertia_Tensor_Base_GI_Origin = [Base_Beam_Combined_mass*eye(3) -Base_Beam_Combined_mass*Combined_Inertia_Tensor_Base_Origin_COM_vec_cross;...
                                          Base_Beam_Combined_mass*Combined_Inertia_Tensor_Base_Origin_COM_vec_cross Combined_Base_Beam_Inertia_Tensor-Base_Beam_Combined_mass*Combined_Inertia_Tensor_Base_Origin_COM_vec_cross^2];

% Wrist Link
Wrist_Link_mass = 2.5; % kg
Wrist_Link_length = 0.35; % m
Wrist_Link_radius = 0.06; % m
Wrist_Link_I_xx_COM = 0.5*Wrist_Link_mass*Wrist_Link_radius^2;
Wrist_Link_I_yy_COM = Wrist_Link_mass*(0.25*Wrist_Link_radius^2+1/12*Wrist_Link_length^2);
Wrist_Link_I_zz_COM = Wrist_Link_I_yy_COM;
Wrist_Link_I_COM = [Wrist_Link_I_xx_COM 0 0;...
                    0 Wrist_Link_I_yy_COM 0;...
                    0 0 Wrist_Link_I_zz_COM];
Wrist_Link_Origin_COM_vec = [Wrist_Link_length/2;0;0]; % Point to the COM from the position where we want to identify the Inertia Tensor
Wrist_Link_Origin_COM_vec_cross = [0 -Wrist_Link_Origin_COM_vec(3,1) Wrist_Link_Origin_COM_vec(2,1);...
                                   Wrist_Link_Origin_COM_vec(3,1) 0 -Wrist_Link_Origin_COM_vec(1,1);...
                                   -Wrist_Link_Origin_COM_vec(2,1) Wrist_Link_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Wrist_Link_GI_Origin = [Wrist_Link_mass*eye(3) -Wrist_Link_mass*Wrist_Link_Origin_COM_vec_cross;...
                        Wrist_Link_mass*Wrist_Link_Origin_COM_vec_cross Wrist_Link_I_COM-Wrist_Link_mass*Wrist_Link_Origin_COM_vec_cross^2];

% Palm Link
Palm_Link_mass = 2.21797364000000; % kg
Palm_Link_I_COM = [0.00277625280650592 -1.13390039503831e-06 0.000203838993190041;...
                   -1.13390039503831e-06 0.00203610232893822 5.34007917058065e-06;...
                   0.000203838993190041 5.34007917058010e-06 0.00163424076982893];
Palm_Link_Origin_COM_vec = [0.0933389144216203;8.78206684804420e-05;-0.00307873900523976]; % Point to the COM from the position where we want to identify the Inertia Tensor
Palm_Link_Origin_COM_vec_cross = [0 -Palm_Link_Origin_COM_vec(3,1) Palm_Link_Origin_COM_vec(2,1);...
                                  Palm_Link_Origin_COM_vec(3,1) 0 -Palm_Link_Origin_COM_vec(1,1);...
                                  -Palm_Link_Origin_COM_vec(2,1) Palm_Link_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Palm_Link_GI_Origin = [Palm_Link_mass*eye(3) -Palm_Link_mass*Palm_Link_Origin_COM_vec_cross;...
                       Palm_Link_mass*Palm_Link_Origin_COM_vec_cross Palm_Link_I_COM-Palm_Link_mass*Palm_Link_Origin_COM_vec_cross^2];

% Cart
Cart_Link_mass = 5; % kg
Cart_Link_height = 0.05; % m
Cart_Link_I_xx_COM = 1/12*Cart_Link_mass*(cart_Link_width^2+Cart_Link_height^2);
Cart_Link_I_yy_COM = 1/12*Cart_Link_mass*(cart_Link_length^2+Cart_Link_height^2);
Cart_Link_I_zz_COM = 1/12*Cart_Link_mass*(cart_Link_length^2+cart_Link_width^2);
Cart_Link_I_COM = [Cart_Link_I_xx_COM 0 0;...
                   0 Cart_Link_I_yy_COM 0;...
                   0 0 Cart_Link_I_zz_COM];
Cart_Link_Origin_COM_vec = [cart_Link_length/2;0;0]; % Point to the COM from the position where we want to identify the Inertia Tensor
Cart_Link_Origin_COM_vec_cross = [0 -Cart_Link_Origin_COM_vec(3,1) Cart_Link_Origin_COM_vec(2,1);...
                                  Cart_Link_Origin_COM_vec(3,1) 0 -Cart_Link_Origin_COM_vec(1,1);...
                                  -Cart_Link_Origin_COM_vec(2,1) Cart_Link_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Cart_Link_GI_Origin = [Cart_Link_mass*eye(3) -Cart_Link_mass*Cart_Link_Origin_COM_vec_cross;...
                       Cart_Link_mass*Cart_Link_Origin_COM_vec_cross Cart_Link_I_COM-Cart_Link_mass*Cart_Link_Origin_COM_vec_cross^2];

% Cart Handle
Cart_Handle_Link_mass = 0.5; % kg
Cart_Handle_Link_length = cart_handle_length; % m
Cart_Handle_Link_width = 0.02; % m
Cart_Handle_Link_height = 0.02; % m
Cart_Handle_Link_I_xx_COM = 1/12*Cart_Handle_Link_mass*(Cart_Handle_Link_width^2+Cart_Handle_Link_height^2);
Cart_Handle_Link_I_yy_COM = 1/12*Cart_Handle_Link_mass*(Cart_Handle_Link_length^2+Cart_Handle_Link_height^2);
Cart_Handle_Link_I_zz_COM = 1/12*Cart_Handle_Link_mass*(Cart_Handle_Link_length^2+Cart_Handle_Link_width^2);
Cart_Handle_Link_I_COM = [Cart_Handle_Link_I_xx_COM 0 0;...
                          0 Cart_Handle_Link_I_yy_COM 0;...
                          0 0 Cart_Handle_Link_I_zz_COM];
Cart_Handle_Link_Origin_COM_vec = [Cart_Handle_Link_length/2;0;0]; % Point to the COM from the position where we want to identify the Inertia Tensor
Cart_Handle_Link_Origin_COM_vec_cross = [0 -Cart_Handle_Link_Origin_COM_vec(3,1) Cart_Handle_Link_Origin_COM_vec(2,1);...
                                         Cart_Handle_Link_Origin_COM_vec(3,1) 0 -Cart_Handle_Link_Origin_COM_vec(1,1);...
                                         -Cart_Handle_Link_Origin_COM_vec(2,1) Cart_Handle_Link_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Cart_Handle_Link_GI_Origin = [Cart_Handle_Link_mass*eye(3) -Cart_Handle_Link_mass*Cart_Handle_Link_Origin_COM_vec_cross;...
                              Cart_Handle_Link_mass*Cart_Handle_Link_Origin_COM_vec_cross Cart_Handle_Link_I_COM-Cart_Handle_Link_mass*Cart_Handle_Link_Origin_COM_vec_cross^2];

% Combined Inertia and COM location of Cart+Cart-handle+WAM-Palm
% COM Location
% Cart CoM
cart_COM_pos_1 = [cart_Link_length/2;0;0]; % w.r.t rear axle
cart_COM_pose_axis = [0;0;1];
cart_COM_ang = 0;
cart_COM_pose_Cart_Origin = init_homo_trans_mat(cart_COM_pos_1,cart_COM_pose_axis,cart_COM_ang);
cart_COM_pose_Cart_handle_handle_Origin = (cart_COM_pose_Cart_Origin\(cart_rear_axle_pose\(cart_handle_pose_init*cart_handle_handle_pose_init)))\eye(4);
% Handle CoM
cart_handle_COM_pose_Cart_Origin = cart_rear_axle_pose\(cart_handle_pose_init*cart_handle_COM_pose_init);
cart_handle_COM_pose_Cart_handle_handle_Origin = cart_handle_handle_pose_init\cart_handle_COM_pose_init;
% Palm COM
cart_handle_3_COM_delta_lin = Palm_Link_Origin_COM_vec;
cart_handle_3_COM_pose_axis = [1;0;0];
cart_handle_3_COM_angle = 0;
% Create homogeneous transformation matrices for base case
palm_COM_pose_Cart_Origin = cart_rear_axle_pose\(cart_handle_pose_init*cart_handle_handle_pose_init*init_homo_trans_mat(cart_handle_3_COM_delta_lin,cart_handle_3_COM_pose_axis,cart_handle_3_COM_angle));
palm_COM_pose_Cart_handle_handle_Origin = init_homo_trans_mat(cart_handle_3_COM_delta_lin,cart_handle_3_COM_pose_axis,cart_handle_3_COM_angle);
% Palm Origin
cart_handle_2_delta_lin = [0;0;0];
cart_handle_2_pose_axis = [1;0;0];
cart_handle_2_angle = 0;
% Create homogeneous transformation matrices for base case
palm_origin_pose = cart_rear_axle_pose\(cart_handle_pose_init*cart_handle_handle_pose_init*init_homo_trans_mat(cart_handle_2_delta_lin,cart_handle_2_pose_axis,cart_handle_2_angle));
% COM location
Cart_Handle_Palm_Combined_mass = Cart_Link_mass+Cart_Handle_Link_mass+Palm_Link_mass;
origin = [0;0;0;1];
Combined_COM_loc_Cart_Origin = (Cart_Link_mass*cart_COM_pose_Cart_Origin*origin+...
                                Cart_Handle_Link_mass*cart_handle_COM_pose_Cart_Origin*origin+...
                                Palm_Link_mass*palm_COM_pose_Cart_Origin*origin)/Cart_Handle_Palm_Combined_mass;
Combined_COM_loc_Cart_handle_handle_Origin = (Cart_Link_mass*cart_COM_pose_Cart_handle_handle_Origin*origin+...
                                              Cart_Handle_Link_mass*cart_handle_COM_pose_Cart_handle_handle_Origin*origin+...
                                              Palm_Link_mass*palm_COM_pose_Cart_handle_handle_Origin*origin)/Cart_Handle_Palm_Combined_mass;
% Inertia about COM
% Relocate Cart's Inertia
COM_to_Cart_COM_vec = cart_COM_pose_Cart_Origin(1:3,4)-Combined_COM_loc_Cart_Origin(1:3,1);
COM_to_Cart_COM_vec_cross = [0 -COM_to_Cart_COM_vec(3,1) COM_to_Cart_COM_vec(2,1);...
                             COM_to_Cart_COM_vec(3,1) 0 -COM_to_Cart_COM_vec(1,1);...
                             -COM_to_Cart_COM_vec(2,1) COM_to_Cart_COM_vec(1,1) 0];
new_Cart_Inertia_Tensor = Cart_Link_I_COM-Cart_Link_mass*COM_to_Cart_COM_vec_cross^2;
% Relocate Cart-handle's Inertia
COM_to_Cart_Handle_COM_vec = cart_handle_COM_pose_Cart_Origin(1:3,4)-Combined_COM_loc_Cart_Origin(1:3,1); % Point to the COM from the position where we want to identify the Inertia Tensor
COM_to_Cart_Handle_COM_vec_cross = [0 -COM_to_Cart_Handle_COM_vec(3,1) COM_to_Cart_Handle_COM_vec(2,1);...
                                    COM_to_Cart_Handle_COM_vec(3,1) 0 -COM_to_Cart_Handle_COM_vec(1,1);...
                                    -COM_to_Cart_Handle_COM_vec(2,1) COM_to_Cart_Handle_COM_vec(1,1) 0];
% We re-orientate the Inertia matris from its local (handle) frame pose to
% that of the cart frames' pose
new_Cart_Handle_Inertia_Tensor = cart_handle_pose_init(1:3,1:3)*Cart_Handle_Link_I_COM*cart_handle_pose_init(1:3,1:3)'-Cart_Handle_Link_mass*COM_to_Cart_Handle_COM_vec_cross^2;
% Relocate Palm's Inertia
COM_to_Palm_COM_vec = palm_COM_pose_Cart_Origin(1:3,4)-Combined_COM_loc_Cart_Origin(1:3,1); % Point to the COM from the position where we want to identify the Inertia Tensor
COM_to_Palm_COM_vec_cross = [0 -COM_to_Palm_COM_vec(3,1) COM_to_Palm_COM_vec(2,1);...
                             COM_to_Palm_COM_vec(3,1) 0 -COM_to_Palm_COM_vec(1,1);...
                             -COM_to_Palm_COM_vec(2,1) COM_to_Palm_COM_vec(1,1) 0];
new_Palm_Inertia_Tensor = Palm_Link_I_COM-Palm_Link_mass*COM_to_Palm_COM_vec_cross^2;
% Combined Inertia Tensor
Combined_Cart_Handle_Palm_Inertia_Tensor = new_Cart_Inertia_Tensor+new_Cart_Handle_Inertia_Tensor+new_Palm_Inertia_Tensor;

Combined_Inertia_Tensor_Palm_Origin_COM_vec = Combined_COM_loc_Cart_Origin(1:3,1)-palm_origin_pose(1:3,4); % Point to the COM from the position where we want to identify the Inertia Tensor
Combined_Inertia_Tensor_Palm_Origin_COM_vec_cross = [0 -Combined_Inertia_Tensor_Palm_Origin_COM_vec(3,1) Combined_Inertia_Tensor_Palm_Origin_COM_vec(2,1);...
                                                     Combined_Inertia_Tensor_Palm_Origin_COM_vec(3,1) 0 -Combined_Inertia_Tensor_Palm_Origin_COM_vec(1,1);...
                                                     -Combined_Inertia_Tensor_Palm_Origin_COM_vec(2,1) Combined_Inertia_Tensor_Palm_Origin_COM_vec(1,1) 0];
% Generalized Inertia Matrix
Combined_Inertia_Tensor_Palm_GI_Origin = [Cart_Handle_Palm_Combined_mass*eye(3) -Cart_Handle_Palm_Combined_mass*Combined_Inertia_Tensor_Palm_Origin_COM_vec_cross;...
                                          Cart_Handle_Palm_Combined_mass*Combined_Inertia_Tensor_Palm_Origin_COM_vec_cross Combined_Cart_Handle_Palm_Inertia_Tensor-Cart_Handle_Palm_Combined_mass*Combined_Inertia_Tensor_Palm_Origin_COM_vec_cross^2];
% Combined_Inertia_Tensor_Palm_GI_Origin(3,3) = 0;
% Define the Inertia Matrix w.r.t the base (world) frame
% Summit
% Summit_GI_Origin_base = calc_adjoint_inv(mat_w_G_s_init)'*Summit_GI_Origin*calc_adjoint_inv(mat_w_G_s_init);
% Base_Link_GI_Origin_base = calc_adjoint_inv(mat_w_G_0_init)'*Base_Link_GI_Origin*calc_adjoint_inv(mat_w_G_0_init);
% Wrist_Link_GI_Origin_base = calc_adjoint_inv(mat_w_G_1_init)'*Wrist_Link_GI_Origin*calc_adjoint_inv(mat_w_G_1_init);
% Palm_Link_GI_Origin_base = calc_adjoint_inv(mat_w_G_2_init)'*Palm_Link_GI_Origin*calc_adjoint_inv(mat_w_G_2_init);
% Combined_Palm_Link_GI_Origin_base = calc_adjoint_inv(mat_w_G_2_init)'*Combined_Inertia_Tensor_Palm_GI_Origin*calc_adjoint_inv(mat_w_G_2_init);

% Cart
Cart_GI_Origin_base = calc_adjoint_inv(mat_w_G_c0_init)'*Cart_Link_GI_Origin*calc_adjoint_inv(mat_w_G_c0_init);
Cart_Handle_GI_Origin_base = calc_adjoint_inv(mat_w_G_c3_init)'*Cart_Handle_Link_GI_Origin*calc_adjoint_inv(mat_w_G_c3_init);

% Effort limits on joints
%                      N |N |Nm|Nm|N
upper_effort_limits = [inf;inf;40;40;40];
lower_effort_limits = -upper_effort_limits;

% MB Translation effort-limits
effort_limit_MB_trans = 80;

% State limits on joints
%                      m  | m  |rad | rad|  m  | rad| local coordinates. 
upper_state_limits = [+inf;+inf;+inf;+inf;+0.17;+inf; inf ; inf ;  inf  ;  inf  ; inf];
lower_state_limits = [-inf;-inf;-inf;-inf;-0.17;-inf;-inf ;-inf ; -inf  ; -inf  ; -inf];

% MB Translation velocity-limits
vel_limit_MB_trans = 0.75;


%% Define Cart and MB vertices for plotting the viewshed
% Summit vertices
sum_length = summit_length;
sum_width = summit_width;
sum_height = 0;
sum_points = [sum_length/2 sum_width/2 sum_height 1;...
              sum_length/2 -sum_width/2 sum_height 1;...
              -sum_length/2 -sum_width/2 sum_height 1;...
              -sum_length/2 sum_width/2 sum_height 1];
% LiDAR frame deviation from Summit frame
% Front
L_frame_front_wrt_summ_frame = [1 0 0 (sum_length/2+0.03);...
                                0 1 0 0;...
                                0 0 1 0;...
                                0 0 0 1];
L_frame_rear_wrt_summ_frame = [-1 0 0 -(sum_length/2+0.03);...
                               0 -1 0 0;...
                               0 0 1 0;...
                               0 0 0 1];
% Cart vertices
cart_length = cart_Link_length;
cart_width = cart_Link_width;
cart_height = 0;
cart_points = [cart_length/2 cart_width/2 cart_height 1;... % Front-Left
               cart_length/2 -cart_width/2 cart_height 1;... % Front-Right
               -cart_length/2 -cart_width/2 cart_height 1;... % Rear-Right
               -cart_length/2 cart_width/2 cart_height 1]; % Rear-Left

%% Casadi
% addpath('C:\Users\chiku\Documents\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*
% Define Homogeneous Transformation matrix. Useful when we have a point
% through which a rotation axis passes through
theta_ = MX.sym('theta');
axis_ = MX.sym('axis',3,1);
point = MX.sym('point',3,1);
homo_point = MX.sym('homo_point',4,1); % Homogeneous point
trans_mat_ = MX.sym('trans_mat_',4,4); % A Transformation matrix
unit_axis_cross_mat_fun = Function('unit_axis_cross_mat_fun',{axis_},...
                                   {[0 -axis_(3,1) axis_(2,1);...
                                     axis_(3,1) 0 -axis_(1,1);...
                                    -axis_(2,1) axis_(1,1) 0]},...
                                   {'axis'},{'unit_axis_cross_mat_fun'});
rot_mat_fun = Function('rot_mat_fun',{axis_,theta_},...
                       {eye(3)+unit_axis_cross_mat_fun(axis_)*sin(theta_)+unit_axis_cross_mat_fun(axis_)^2*(1-cos(theta_))},...
                       {'axis_','theta'},{'rot_mat_fun'});
trans_mat_0_fun = Function('trans_mat_0_fun',{axis_,theta_,point},...
                           {[rot_mat_fun(axis_,theta_) (eye(3)-rot_mat_fun(axis_,theta_))*point;...
                             0 0 0 1]},...
                           {'axis_','theta','point'},{'trans_mat_0_fun'});

trans_mat_inv_fun = Function('trans_mat_inv_fun',{trans_mat_},...
                             {[trans_mat_(1:3,1:3)' -trans_mat_(1:3,1:3)'*trans_mat_(1:3,4);...
                               zeros(1,3) 1]},...
                             {'trans_mat_'},{'trans_mat_inv_fun'});

% Define Adjoint Matrix
adj_mat_fun = Function('adj_mat_fun',{trans_mat_},...
                       {[trans_mat_(1:3,1:3) [0 -trans_mat_(3,4) trans_mat_(2,4);...
                                              trans_mat_(3,4) 0 -trans_mat_(1,4);...
                                              -trans_mat_(2,4) trans_mat_(1,4) 0]*trans_mat_(1:3,1:3);...
                         zeros(3,3) trans_mat_(1:3,1:3)]},...
                       {'trans_mat_'},{'adj_mat_fun'});

% Define Inverse Adjoint Matrix
trans_mat_ = MX.sym('trans_mat_',4,4);
inv_adj_mat_fun = Function('inv_adj_mat_fun',{trans_mat_},...
                           {[trans_mat_(1:3,1:3)' -trans_mat_(1:3,1:3)'*[0 -trans_mat_(3,4) trans_mat_(2,4);...
                                                                        trans_mat_(3,4) 0 -trans_mat_(1,4);...
                                                                        -trans_mat_(2,4) trans_mat_(1,4) 0];...
                             zeros(3,3) trans_mat_(1:3,1:3)']},...
                           {'trans_mat_'},{'inv_adj_mat_fun'});

% Define Rotation Twist motion. Useful when we know of the rotational and
% linear velocity being experienced by an object
twist = MX.sym('twist',6,1);
trans_mat_1_fun = Function('trans_mat_1_fun',{twist,theta_},...
                           {[rot_mat_fun(twist(4:6,1),theta_) (eye(3)-rot_mat_fun(twist(4:6,1),theta_))*unit_axis_cross_mat_fun(twist(4:6,1))*twist(1:3,1)+...
                           twist(4:6,1)*twist(4:6,1)'*twist(1:3,1)*theta_;...
                             0 0 0 1]},...
                           {'twist','theta'},{'trans_mat_1_fun'});

% Define Pure Displacement Twist motion
trans_mat_2_fun = Function('trans_mat_2_fun',{twist,theta_},...
                           {[eye(3) twist(1:3,1)*theta_;...
                             0 0 0 1]},...
                           {'twist','theta'},{'trans_mat_2_fun'});
%% Define the displacements for each twist
import casadi.*
% Define the displacements for each twist
% Summit+WAM = 6 dof: 3 Summit | 1 Beam Link (swivel) | 1 Wrist Link (extend) | 1 Palm Link (swivel)
% Summit
num_dof_Summit = 6;
% Summit+WAM Joint damping coeffiecinet
damping = eye(num_dof_Summit)*0.1;
% Structure of the system state variable
% 1 <- Summit X-Axis Linear Displacement
% 2 <- Summit Y-Axis Linear Displacement
% 3 <- Summit Z-Axis Angular Displacement
% 4 <- Beam Link (swivel) Angular Displacement
% 5 <- WAM Wrist Link (extend) Linear Displacement
% 6 <- WAM Palm Link (swivel) Angular Displacement
% 7 <- Summit X-Axis Linear Velocity
% 8 <- Summit Y-Axis Linear Velocity
% 9 <- Summit Z-Axis Angular Velocity
% 10 <- Beam Link (swivel) Angular Velocity
% 11 <- WAM Wrist Link (extend) Linear Velocity
% 12 <- WAM Palm Link (swivel) Angular Velocity
% Displacement + Velocity
states = MX.sym('states',num_dof_Summit*2-1,1); % This will be recycled for the modified implementation
% states_accel = MX.sym('states_accel',num_dof_Summit,1);
% Independent Local coordinates of the paffain Nullspace
states_accel_mod = MX.sym('states_accel',num_dof_Summit-1,1);
% are only (num_dof_Summit-1) independent local states to regulate the
% num_dof_Summit d_theta
% Inputs
num_inputs = num_dof_Summit-1;
% 1->4 : Inputs to Active Joints of Summit+WAM
inputs = MX.sym('inputs',num_inputs,1);
% Inputs to Passive Joints
inputs_Summit_passive = 0;%MX(1,1);
% All inputs to Summit-WAM sub-system
inputs_Summit_all = [inputs;inputs_Summit_passive];

% All inputs to Summit-WAM sub-system


% Define the Jacobians for each link
% Summit Link Spatial Jacobian
J_Summit_Link_Spatial_fun = Function('J_Summit_Link_Spatial_fun',{states},...
                                     {[twist_lin_x_summit ...
                                       twist_lin_y_summit ...
                                       adj_mat_fun(trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                                   trans_mat_2_fun(twist_lin_y_summit,states(2,1)))*twist_ang_z_summit]},...
                                      {'states'},{'J_Summit_Link_Spatial_fun'});
J_Summit_Link_Spatial = [twist_lin_x_summit ...
                         twist_lin_y_summit ...
                         adj_mat_fun(trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                     trans_mat_2_fun(twist_lin_y_summit,states(2,1)))*twist_ang_z_summit];
% Summit Link Body Jacobian
J_Summit_Link_Body_0_homo_mat = trans_mat_1_fun(twist_ang_z_summit,states(3,1));
J_Summit_Link_Body_1_homo_mat = trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                trans_mat_2_fun(twist_lin_y_summit,states(2,1))*...
                                J_Summit_Link_Body_0_homo_mat;
J_Summit_Link_Body_2_homo_mat = trans_mat_2_fun(twist_lin_y_summit,states(2,1))*...
                                J_Summit_Link_Body_0_homo_mat;
J_Summit_Link_Body_fun = Function('J_Summit_Link_Body_fun',{states},...
                                  {calc_adjoint_inv(mat_w_G_s_init)*[inv_adj_mat_fun(J_Summit_Link_Body_1_homo_mat)*twist_lin_x_summit ...
                                                                     inv_adj_mat_fun(J_Summit_Link_Body_2_homo_mat)*twist_lin_y_summit ...
                                                                     twist_ang_z_summit ...
                                                                     zeros(6,3)]},...
                                   {'states'},{'J_Summit_Link_Body_fun'});
J_Summit_Link_Body = calc_adjoint_inv(mat_w_G_s_init)*[inv_adj_mat_fun(J_Summit_Link_Body_1_homo_mat)*twist_lin_x_summit ...
                                                       inv_adj_mat_fun(J_Summit_Link_Body_2_homo_mat)*twist_lin_y_summit ...
                                                       twist_ang_z_summit ...
                                                       zeros(6,3)];

% Beam Link Spatial Jacobian
J_Beam_Link_Spatial_fun = Function('J_Beam_Link_Spatial_fun',{states},...
                                   {[J_Summit_Link_Spatial ...
                                     adj_mat_fun(trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                             trans_mat_2_fun(twist_lin_y_summit,states(2,1))*...
                                             trans_mat_1_fun(twist_ang_z_summit,states(3,1)))*twist_0_1_revolute_base]},...
                                   {'states'},{'J_Beam_Link_Spatial_fun'});
J_Beam_Link_Spatial = [J_Summit_Link_Spatial ...
                       adj_mat_fun(trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                   trans_mat_2_fun(twist_lin_y_summit,states(2,1))*...
                                   trans_mat_1_fun(twist_ang_z_summit,states(3,1)))*twist_0_1_revolute_base];
% Beam Link Body Jacobian
J_Beam_Link_Body_0_homo_mat = trans_mat_1_fun(twist_0_1_revolute_base,states(4,1));
J_Beam_Link_Body_1_homo_mat = J_Summit_Link_Body_1_homo_mat*...
                              J_Beam_Link_Body_0_homo_mat;
J_Beam_Link_Body_2_homo_mat = J_Summit_Link_Body_2_homo_mat*...
                              J_Beam_Link_Body_0_homo_mat;
J_Beam_Link_Body_3_homo_mat = J_Summit_Link_Body_0_homo_mat*...
                              J_Beam_Link_Body_0_homo_mat;
J_Beam_Link_Body_fun = Function('J_Beam_Link_Body_fun',{states},...
                                {calc_adjoint_inv(mat_w_G_1_init)*[inv_adj_mat_fun(J_Beam_Link_Body_1_homo_mat)*twist_lin_x_summit ...
                                                                   inv_adj_mat_fun(J_Beam_Link_Body_2_homo_mat)*twist_lin_y_summit ...
                                                                   inv_adj_mat_fun(J_Beam_Link_Body_3_homo_mat)*twist_ang_z_summit ...
                                                                   twist_0_1_revolute_base ...
                                                                   zeros(6,2)]},...
                                {'states'},{'J_Beam_Link_Body_fun'});
J_Beam_Link_Body = calc_adjoint_inv(mat_w_G_1_init)*[inv_adj_mat_fun(J_Beam_Link_Body_1_homo_mat)*twist_lin_x_summit ...
                                                     inv_adj_mat_fun(J_Beam_Link_Body_2_homo_mat)*twist_lin_y_summit ...
                                                     inv_adj_mat_fun(J_Beam_Link_Body_3_homo_mat)*twist_ang_z_summit ...
                                                     twist_0_1_revolute_base ...
                                                     zeros(6,2)];

% Wrist Link Spatial Jacobian
J_Wrist_Link_Spatial_fun = Function('J_Wrist_Link_Spatial_fun',{states},...
                                    {[J_Beam_Link_Spatial ...
                                      adj_mat_fun(trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                              trans_mat_2_fun(twist_lin_y_summit,states(2,1))*...
                                              trans_mat_1_fun(twist_ang_z_summit,states(3,1))*...
                                              trans_mat_1_fun(twist_0_1_revolute_base,states(4,1)))*twist_1_2_linear_base]},...
                                     {'states'},{'J_Wrist_Link_Spatial_fun'});
J_Wrist_Link_Spatial = [J_Beam_Link_Spatial ...
                        adj_mat_fun(trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                    trans_mat_2_fun(twist_lin_y_summit,states(2,1))*...
                                    trans_mat_1_fun(twist_ang_z_summit,states(3,1))*...
                                    trans_mat_1_fun(twist_0_1_revolute_base,states(4,1)))*twist_1_2_linear_base];
% Wrist Link Body Jacobian
J_Wrist_Link_Body_0_homo_mat = trans_mat_2_fun(twist_1_2_linear_base,states(5,1));
J_Wrist_Link_Body_1_homo_mat = J_Beam_Link_Body_1_homo_mat*...
                               J_Wrist_Link_Body_0_homo_mat;
J_Wrist_Link_Body_2_homo_mat = J_Beam_Link_Body_2_homo_mat*...
                               J_Wrist_Link_Body_0_homo_mat;
J_Wrist_Link_Body_3_homo_mat = J_Beam_Link_Body_3_homo_mat*...
                               J_Wrist_Link_Body_0_homo_mat;
J_Wrist_Link_Body_4_homo_mat = J_Beam_Link_Body_0_homo_mat*...
                               J_Wrist_Link_Body_0_homo_mat;
J_Wrist_Link_Body_fun = Function('J_Wrist_Link_Body_fun',{states},...
                                 {calc_adjoint_inv(mat_w_G_2_init)*[inv_adj_mat_fun(J_Wrist_Link_Body_1_homo_mat)*twist_lin_x_summit ...
                                                                    inv_adj_mat_fun(J_Wrist_Link_Body_2_homo_mat)*twist_lin_y_summit ...
                                                                    inv_adj_mat_fun(J_Wrist_Link_Body_3_homo_mat)*twist_ang_z_summit ...
                                                                    inv_adj_mat_fun(J_Wrist_Link_Body_4_homo_mat)*twist_0_1_revolute_base ...
                                                                    twist_1_2_linear_base ...
                                                                    zeros(6,1)]},...
                                 {'states'},{'J_Wrist_Link_Body_fun'});
J_Wrist_Link_Body = calc_adjoint_inv(mat_w_G_2_init)*[inv_adj_mat_fun(J_Wrist_Link_Body_1_homo_mat)*twist_lin_x_summit ...
                                                      inv_adj_mat_fun(J_Wrist_Link_Body_2_homo_mat)*twist_lin_y_summit ...
                                                      inv_adj_mat_fun(J_Wrist_Link_Body_3_homo_mat)*twist_ang_z_summit ...
                                                      inv_adj_mat_fun(J_Wrist_Link_Body_4_homo_mat)*twist_0_1_revolute_base ...
                                                      twist_1_2_linear_base ...
                                                      zeros(6,1)];

% Palm Link Spatial Jacobian
J_Palm_Link_Spatial_fun = Function('J_Palm_Link_Spatial_fun',{states},...
                                   {[J_Wrist_Link_Spatial ...
                                     adj_mat_fun(trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                                 trans_mat_2_fun(twist_lin_y_summit,states(2,1))*...
                                                 trans_mat_1_fun(twist_ang_z_summit,states(3,1))*...
                                                 trans_mat_1_fun(twist_0_1_revolute_base,states(4,1))*...
                                                 trans_mat_2_fun(twist_1_2_linear_base,states(5,1)))*twist_1_2_revolute_z_base]},...
                                   {'states'},{'J_Palm_Link_Spatial_fun'});
J_Palm_Link_Spatial = [J_Wrist_Link_Spatial ...
                       adj_mat_fun(trans_mat_2_fun(twist_lin_x_summit,states(1,1))*...
                                   trans_mat_2_fun(twist_lin_y_summit,states(2,1))*...
                                   trans_mat_1_fun(twist_ang_z_summit,states(3,1))*...
                                   trans_mat_1_fun(twist_0_1_revolute_base,states(4,1))*...
                                   trans_mat_2_fun(twist_1_2_linear_base,states(5,1)))*twist_1_2_revolute_z_base];
% Palm Link Body Jacobian
J_Palm_Link_Body_0_homo_mat = trans_mat_1_fun(twist_1_2_revolute_z_base,states(6,1));
J_Palm_Link_Body_1_homo_mat = J_Wrist_Link_Body_1_homo_mat*...
                              J_Palm_Link_Body_0_homo_mat;
J_Palm_Link_Body_2_homo_mat = J_Wrist_Link_Body_2_homo_mat*...
                              J_Palm_Link_Body_0_homo_mat;
J_Palm_Link_Body_3_homo_mat = J_Wrist_Link_Body_3_homo_mat*...
                              J_Palm_Link_Body_0_homo_mat;
J_Palm_Link_Body_4_homo_mat = J_Wrist_Link_Body_4_homo_mat*...
                              J_Palm_Link_Body_0_homo_mat;
J_Palm_Link_Body_5_homo_mat = J_Wrist_Link_Body_0_homo_mat*...
                              J_Palm_Link_Body_0_homo_mat;
J_Palm_Link_Body_fun = Function('J_Palm_Link_Body_fun',{states},...
                                {calc_adjoint_inv(mat_w_G_3_init)*[inv_adj_mat_fun(J_Palm_Link_Body_1_homo_mat)*twist_lin_x_summit ...
                                                                   inv_adj_mat_fun(J_Palm_Link_Body_2_homo_mat)*twist_lin_y_summit ...
                                                                   inv_adj_mat_fun(J_Palm_Link_Body_3_homo_mat)*twist_ang_z_summit ...
                                                                   inv_adj_mat_fun(J_Palm_Link_Body_4_homo_mat)*twist_0_1_revolute_base ...
                                                                   inv_adj_mat_fun(J_Palm_Link_Body_5_homo_mat)*twist_1_2_linear_base ...
                                                                   twist_1_2_revolute_z_base]},...
                                {'states'},{'J_Palm_Link_Body_fun'});
J_Palm_Link_Body = calc_adjoint_inv(mat_w_G_3_init)*[inv_adj_mat_fun(J_Palm_Link_Body_1_homo_mat)*twist_lin_x_summit ...
                                                     inv_adj_mat_fun(J_Palm_Link_Body_2_homo_mat)*twist_lin_y_summit ...
                                                     inv_adj_mat_fun(J_Palm_Link_Body_3_homo_mat)*twist_ang_z_summit ...
                                                     inv_adj_mat_fun(J_Palm_Link_Body_4_homo_mat)*twist_0_1_revolute_base ...
                                                     inv_adj_mat_fun(J_Palm_Link_Body_5_homo_mat)*twist_1_2_linear_base ...
                                                     twist_1_2_revolute_z_base];

J_Palm_Surface_Body_fun = Function('J_Palm_Surface_Body_fun',{states},...
                               {J_Palm_Link_Body},...
                               {'states'},{'J_Palm_Surface_Body_fun'});
J_Palm_Surface_Body = J_Palm_Link_Body;
J_Cart_Origin_Body_fun = Function('J_Cart_Origin_Body_fun',{states},...
                              {adjoint_Cart_origin_2*J_Palm_Link_Body},...
                              {'states'},{'J_Cart_Origin_Body_fun'});
J_Cart_Origin_Body = adjoint_Cart_origin_2*J_Palm_Link_Body;

% Define the Complete Inertia Matrix
% Summit
Summit_Inertia_Matrix_fun = Function('Summit_Inertia_Matrix_fun',{states},...
                                     {J_Summit_Link_Body'*Summit_GI_Origin*...
                                      J_Summit_Link_Body},...
                                     {'states'},{'Combined_Summit_Inertia_Matrix_fun'});
Summit_Inertia_Matrix = J_Summit_Link_Body'*Summit_GI_Origin*...
                        J_Summit_Link_Body;
% Beam Link
Combined_Beam_Inertia_Matrix_fun = Function('Combined_Beam_Inertia_Matrix_fun',{states},...
                                            {J_Beam_Link_Body'*Combined_Inertia_Tensor_Base_GI_Origin*...
                                             J_Beam_Link_Body},...
                                            {'states'},{'Combined_Beam_Inertia_Matrix_fun'});
Combined_Beam_Inertia_Matrix = J_Beam_Link_Body'*Combined_Inertia_Tensor_Base_GI_Origin*...
                               J_Beam_Link_Body;
% Wrist Link
Wrist_Inertia_Matrix_fun = Function('Wrist_Inertia_Matrix_fun',{states},...
                                    {J_Wrist_Link_Body'*Wrist_Link_GI_Origin*...
                                     J_Wrist_Link_Body},...
                                    {'states'},{'Wrist_Inertia_Matrix_fun'});
Wrist_Inertia_Matrix = J_Wrist_Link_Body'*Wrist_Link_GI_Origin*...
                       J_Wrist_Link_Body;
% Palm Link
Palm_Inertia_Matrix_fun = Function('Palm_Inertia_Matrix_fun',{states},...
                                   {J_Palm_Link_Body'*Palm_Link_GI_Origin*...
                                    J_Palm_Link_Body},...
                                   {'states'},{'Palm_Inertia_Matrix_fun'});
Palm_Inertia_Matrix = J_Palm_Link_Body'*Palm_Link_GI_Origin*...
                      J_Palm_Link_Body;
% Combined Palm Link
Combined_Palm_Inertia_Matrix_fun = Function('Combined_Palm_Inertia_Matrix_fun',{states},...
                                            {J_Palm_Link_Body'*Combined_Inertia_Tensor_Palm_GI_Origin*...
                                             J_Palm_Link_Body},...
                                            {'states'},{'Combined_Palm_Inertia_Matrix_fun'});
Combined_Palm_Inertia_Matrix = J_Palm_Link_Body'*Combined_Inertia_Tensor_Palm_GI_Origin*...
                               J_Palm_Link_Body;
% Total Inertia Matrix
Inertia_Matrix = Summit_Inertia_Matrix+...
                 Combined_Beam_Inertia_Matrix+...
                 Wrist_Inertia_Matrix+...
                 Combined_Palm_Inertia_Matrix;
Inertia_Matrix_fun = Function('Inertia_Matrix_fun',{states},...
                              {Inertia_Matrix},...
                              {'states'},{'Inertia_Matrix_fun'});
% Define a vertical representation of the Inertia Matrix
Inertia_Matrix_vert = reshape(Inertia_Matrix,[num_dof_Summit^2,1]);
% Calculate the Jacobian of the Inertia Matrix
J_Inertia_mat = jacobian(Inertia_Matrix_vert,states);
J_Inertia_mat_fun = Function('J_Inertia_mat_fun',{states},...
                             {J_Inertia_mat},...
                             {'states'},{'J_Inertia_mat_fun'});
% Cannot use this corriolis matrix since the omegas need to be replaced
% with the linear combination of the null-space
% % Define the Coriolis Matrix
% Coriolis_Matrix = MX(num_dof_Summit,num_dof_Summit);
% for row_index=1:num_dof_Summit
%     for column_index=1:num_dof_Summit
%         for k=1:num_dof_Summit
%             Coriolis_Matrix(row_index,column_index) = Coriolis_Matrix(row_index,column_index)+...
%                                                       0.5*(J_Inertia_mat(row_index+(column_index-1)*num_dof_Summit,k)+...
%                                                       J_Inertia_mat(row_index+(k-1)*num_dof_Summit,column_index)-...
%                                                       J_Inertia_mat(k+(column_index-1)*num_dof_Summit,row_index))*states(num_dof_Summit+k,1);
%         end
%     end
% end
% Coriolis_Matrix_fun = Function('Coriolis_Matrix_fun',{states},...
%                                {Coriolis_Matrix},...
%                                {'states'},{'Coriolis_Matrix_fun'});

% Gravitational Force Term
% Define current pose of each link
% Summit
mat_w_G_s_curr_fun = Function('mat_w_G_s_curr_fun',{states},...
                              {J_Summit_Link_Body_1_homo_mat*...
                               mat_w_G_s_init},...
                              {'states'},{'mat_w_G_s_curr_fun'});
mat_w_G_s_curr = J_Summit_Link_Body_1_homo_mat*...
                 mat_w_G_s_init;
% Beam Link
mat_w_G_1_curr_fun = Function('mat_w_G_1_curr_fun',{states},...
                              {J_Beam_Link_Body_1_homo_mat*...
                               mat_w_G_1_init},...
                              {'states'},{'mat_w_G_1_curr_fun'});
mat_w_G_1_curr = J_Beam_Link_Body_1_homo_mat*...
                 mat_w_G_1_init;
% Wrist Link
mat_w_G_2_curr_fun = Function('mat_w_G_2_curr_fun',{states},...
                              {J_Wrist_Link_Body_1_homo_mat*...
                               mat_w_G_2_init},...
                              {'states'},{'mat_w_G_2_curr_fun'});
mat_w_G_2_curr = J_Wrist_Link_Body_1_homo_mat*...
                 mat_w_G_2_init;
% Palm Link
mat_w_G_3_curr_fun = Function('mat_w_G_3_curr_fun',{states},...
                              {J_Palm_Link_Body_1_homo_mat*...
                               mat_w_G_3_init},...
                              {'states'},{'mat_w_G_3_curr_fun'});
mat_w_G_3_curr = J_Palm_Link_Body_1_homo_mat*...
                 mat_w_G_3_init;
% Cart Link (not on the cart link (rotated accordingly), but on the cart
% base)
mat_w_G_c0_curr_fun = Function('mat_w_G_c0_curr_fun',{states},...
                               {J_Palm_Link_Body_1_homo_mat*...
                                mat_w_G_c0_init},...
                               {'states'},{'mat_w_G_c0_curr_fun'});
mat_w_G_c0_curr = J_Palm_Link_Body_1_homo_mat*...
                  mat_w_G_c0_init;
% Cart COM
mat_w_G_c4_curr_fun = Function('mat_w_G_c4_curr_fun',{states},...
                               {mat_w_G_c0_curr/cart_rear_axle_pose*cart_COM_pose},...
                               {'states'},{'mat_w_G_c4_curr_fun'});
mat_w_G_c4_curr = mat_w_G_c0_curr/cart_rear_axle_pose*cart_COM_pose;
% Cart handle Handle Frame
mat_w_G_c5_curr_fun = Function('mat_w_G_c5_curr_fun',{states},...
                               {mat_w_G_c0_curr*(cart_rear_axle_pose\(cart_handle_pose_init*cart_handle_handle_pose_init_1))},...
                               {'states'},{'mat_w_G_c5_curr_fun'});
mat_w_G_c5_curr = mat_w_G_c0_curr*(cart_rear_axle_pose\(cart_handle_pose_init*cart_handle_handle_pose_init_1));

% Define the Potential energy
Potential_Energy = Summit_mass*g'*(mat_w_G_s_curr*[Summit_Origin_COM_vec;1])+...
                   Base_Beam_Combined_mass*g'*(mat_w_G_1_curr*Combined_COM_loc_Base_Origin)+...
                   Wrist_Link_mass*g'*(mat_w_G_2_curr*[Wrist_Link_Origin_COM_vec;1])+...
                   Cart_Handle_Palm_Combined_mass*g'*(mat_w_G_c0_curr*Combined_COM_loc_Cart_Origin);
% Calculate Jacobian of Potential Energy
J_Potential_Energy = jacobian(Potential_Energy,states);


%% All constraint equations
import casadi.*
% Constraints

% We will now setup the Paffian constraint seperately in order to
% stream-line the generation of its nullity basis
paffian_constraint = J_Cart_Origin_Body(2,:);
paffian_constraint_fun = Function('paffian_constraint_fun',{states},...
                                  {paffian_constraint},...
                                  {'states'},{'paffian_constraint_fun'});
% Nullspace to the Paffian constraint
null_space_paffian_constraint = [0 0 0 0 -paffian_constraint(1,6) paffian_constraint(1,5);...
                                 0 0 0 -paffian_constraint(1,6) 0 paffian_constraint(1,4);...
                                 0 0 -paffian_constraint(1,6) 0 0 paffian_constraint(1,3);...
                                 0 -paffian_constraint(1,6) 0 0 0 paffian_constraint(1,2);...
                                 -paffian_constraint(1,6) 0 0 0 0 paffian_constraint(1,1)]';
% Remap the state inputs from 'states' to 'states_mod'
null_space_paffian_constraint_fun = Function('null_space_paffian_constraint_fun',{states},...
                                             {null_space_paffian_constraint},...
                                             {'states'},{'null_space_paffian_constraint_fun'});
% theta_dot = null_space_paffian_constraint*u | u = states((num_dof_Summit+1):(num_dof_Summit*2-1),1)
dt_theta_fun = Function('dt_theta_fun',{states},...
                        {null_space_paffian_constraint*states((num_dof_Summit+1):(num_dof_Summit*2-1),1)},...
                        {'states'},{'dt_theta_fun'});
dt_theta = null_space_paffian_constraint*states((num_dof_Summit+1):(num_dof_Summit*2-1),1);
dt_theta_Xk = null_space_paffian_constraint_fun(states)*states((num_dof_Summit+1):(num_dof_Summit*2-1),1);
% Jacobian of the 'paffian_constraint'
J_paffian_constraint = jacobian(paffian_constraint',states);
% dt of 'paffian_constraint'
dt_paffian_constraint = (J_paffian_constraint(:,1:num_dof_Summit)*dt_theta)';
% Time derivative of Nullspace to the Paffian constraint
dt_null_space_paffian_constraint = [0 0 0 0 0 dt_paffian_constraint(1,5);...
                                    0 0 0 0 0 dt_paffian_constraint(1,4);...
                                    0 0 0 0 0 dt_paffian_constraint(1,3);...
                                    0 0 0 0 0 dt_paffian_constraint(1,2);...
                                    0 0 0 0 0 dt_paffian_constraint(1,1)]';
ddt_theta_fun = Function('ddt_theta_fun',{states,states_accel_mod},...
                         {dt_null_space_paffian_constraint*states((num_dof_Summit+1):(num_dof_Summit*2-1),1)+null_space_paffian_constraint_fun(states)*states_accel_mod},...
                         {'states','states_accel_mod'},{'ddt_theta_fun'});
% The Nullspace spans the space of all possible combinations of theta_dot
% that satisfies the paffian constraint
% Now we re-calculate the modified dt_Summit_State_fun
% Modified Inertia matrix
Inertia_Matrix_mod = null_space_paffian_constraint'*Inertia_Matrix*null_space_paffian_constraint;
Inertia_Matrix_mod_fun = Function('Inertia_Matrix_mod_fun',{states},...
                                  {Inertia_Matrix_mod},...
                                  {'states'},{'Inertia_Matrix_mod_fun'});
% Modified Coriolis matrix
Coriolis_Matrix_mod = MX(num_dof_Summit,num_dof_Summit);
for row_index=1:num_dof_Summit
    for column_index=1:num_dof_Summit
        for k=1:num_dof_Summit
            Coriolis_Matrix_mod(row_index,column_index) = Coriolis_Matrix_mod(row_index,column_index)+...
                                                          0.5*(J_Inertia_mat(row_index+(column_index-1)*num_dof_Summit,k)+...
                                                          J_Inertia_mat(row_index+(k-1)*num_dof_Summit,column_index)-...
                                                          J_Inertia_mat(k+(column_index-1)*num_dof_Summit,row_index))*dt_theta_Xk(k,1);
        end
    end
end
Coriolis_Matrix_mod_fun = Function('Coriolis_Matrix_mod_fun',{states},...
                                   {Coriolis_Matrix_mod},...
                                   {'states'},{'Coriolis_Matrix_mod_fun'});
% Modified Gravity_Damping Term
Gravity_Damping_mod_Vec = J_Potential_Energy(1,1:num_dof_Summit)'+damping*dt_theta_Xk;

Gravity_Damping_mod_Vec_fun = Function('Gravity_Damping_Vec_fun',{states},...
                                       {Gravity_Damping_mod_Vec},...
                                       {'states'},{'Gravity_Damping_Vec_fun'});
% Modified Summit Dynamic Equation
dt_Summit_State_mod = [dt_theta_Xk;...
                       Inertia_Matrix_mod\(null_space_paffian_constraint'*inputs_Summit_all-...
                                           null_space_paffian_constraint'*(Inertia_Matrix*dt_null_space_paffian_constraint+...
                                                                           Coriolis_Matrix_mod*null_space_paffian_constraint)*states((num_dof_Summit+1):(num_dof_Summit*2-1),1)-...
                                           null_space_paffian_constraint'*Gravity_Damping_mod_Vec)];

dt_Summit_State_mod_fun = Function('dt_Summit_State_mod_fun',{states,inputs},...
                                   {[dt_theta_Xk;...
                                     Inertia_Matrix_mod\(null_space_paffian_constraint'*inputs_Summit_all-...
                                                         null_space_paffian_constraint'*(Inertia_Matrix*dt_null_space_paffian_constraint+...
                                                                                         Coriolis_Matrix_mod*null_space_paffian_constraint)*states((num_dof_Summit+1):(num_dof_Summit*2-1),1)-...
                                                         null_space_paffian_constraint'*Gravity_Damping_mod_Vec)]},...
                                   {'states','inputs'},{'dt_Summit_State_mod_fun'});
Inputs_Summit_all_mod_fun = Function('Inputs_Summit_all_mod_fun',{states,states_accel_mod},...
                                 {Inertia_Matrix_mod*states_accel_mod+...
                                  null_space_paffian_constraint'*(Inertia_Matrix*dt_null_space_paffian_constraint+...
                                                                  Coriolis_Matrix_mod*null_space_paffian_constraint)*states((num_dof_Summit+1):(num_dof_Summit*2-1),1)+...
                                  null_space_paffian_constraint'*Gravity_Damping_mod_Vec},...
                                 {'states','states_accel'},{'Inputs_Summit_all_mod_fun'});

%% Constraints on simplified WAM joint displacement and velocity
import casadi.*
% Palm-Link Wrist-Link Joint displacement constraint
% Fore-arm link length
l_1 = sqrt(0.045^2+0.3^2);
Angular_offset_l_1 = atan2(0.045,0.3);
% Upper-arm link length
l_2 = sqrt(0.045^2+0.55^2);
Angular_offset_l_2 = atan2(0.045,0.55);
% Wrist-Link extension
Wrist_len = MX.sym('Wrist_len',1,1);
% Joint 6
Joint_6_ang = MX.sym('Joint_6_ang',1,1);
% The minus sign is present in the acos function bevcause, unlike lengths
% l_1 and l_2, which are positive, l_3 is negative
Joint_6_Config_1_upper_lim_fun = Function('Joint_6_Config_1_upper_lim_fun',{Wrist_len},...
                                          {pi/2+(acos(-(l_1^2+(delta_x+Wrist_len)^2-l_2^2)/(2*l_1*(delta_x+Wrist_len)))-Angular_offset_l_1)},...
                                          {'Wrist_len'},{'Joint_6_Config_1_upper_lim_fun'});
Joint_6_Config_1_lower_lim_fun = Function('Joint_6_Config_1_lower_lim_fun',{Wrist_len},...
                                          {-pi/2+(acos(-(l_1^2+(delta_x+Wrist_len)^2-l_2^2)/(2*l_1*(delta_x+Wrist_len)))+Angular_offset_l_1)},...
                                          {'Wrist_len'},{'Joint_6_Config_1_lower_lim_fun'});
Joint_6_Config_2_upper_lim_fun = Function('Joint_6_Config_2_upper_lim_fun',{Wrist_len},...
                                          {pi/2-(acos(-(l_1^2+(delta_x+Wrist_len)^2-l_2^2)/(2*l_1*(delta_x+Wrist_len)))+Angular_offset_l_1)},...
                                          {'Wrist_len'},{'Joint_6_Config_2_upper_lim_fun'});
Joint_6_Config_2_lower_lim_fun = Function('Joint_6_Config_2_lower_lim_fun',{Wrist_len},...
                                          {-pi/2-(acos(-(l_1^2+(delta_x+Wrist_len)^2-l_2^2)/(2*l_1*(delta_x+Wrist_len)))-Angular_offset_l_1)},...
                                          {'Wrist_len'},{'Joint_6_Config_2_lower_lim_fun'});
Joint_6_upper_lim_fun = Function('Joint_6_upper_lim_fun',{Wrist_len,Joint_6_ang},...
                                 {if_else(Joint_6_ang>=Joint_6_Config_2_upper_lim_fun(Wrist_len),...
                                          Joint_6_Config_1_upper_lim_fun(Wrist_len),...
                                          if_else(Joint_6_ang>=Joint_6_Config_1_lower_lim_fun(Wrist_len),...
                                                  Joint_6_Config_1_upper_lim_fun(Wrist_len),...
                                                  Joint_6_Config_2_upper_lim_fun(Wrist_len)))},...
                                 {'Wrist_len','Joint_6_ang'},{'Joint_6_upper_lim_fun'});
Joint_6_lower_lim_fun = Function('Joint_6_lower_lim_fun',{Wrist_len,Joint_6_ang},...
                                 {if_else(Joint_6_ang<Joint_6_Config_1_lower_lim_fun(Wrist_len),...
                                          Joint_6_Config_2_lower_lim_fun(Wrist_len),...
                                          if_else(Joint_6_ang<Joint_6_Config_2_upper_lim_fun(Wrist_len),...
                                                  Joint_6_Config_2_lower_lim_fun(Wrist_len),...
                                                  Joint_6_Config_1_lower_lim_fun(Wrist_len)))},...
                                 {'Wrist_len','Joint_6_ang'},{'Joint_6_lower_lim_fun'});
% Joint 4
Joint_4_ang = MX.sym('Joint_4_ang',1,1);
Joint_4_Config_1_upper_lim_fun = 2.6;
Joint_4_Config_1_lower_lim_fun = Function('Joint_4_Config_1_lower_lim_fun',{Wrist_len},...
                                          {-2.6+(acos(-(l_2^2+(delta_x+Wrist_len)^2-l_1^2)/(2*l_2*(delta_x+Wrist_len)))+Angular_offset_l_2)},...
                                          {'Wrist_len'},{'Joint_4_Config_1_lower_lim_fun'});
Joint_4_Config_2_upper_lim_fun = Function('Joint_4_Config_2_upper_lim_fun',{Wrist_len},...
                                          {2.6-(acos(-(l_2^2+(delta_x+Wrist_len)^2-l_1^2)/(2*l_2*(delta_x+Wrist_len)))+Angular_offset_l_2)},...
                                          {'Wrist_len'},{'Joint_4_Config_2_upper_lim_fun'});
Joint_4_Config_2_lower_lim_fun = -2.6;
% Wrist Link Extension velocity constraint
Joint_5_vel_upper_lim_fun = Function('Joint_5_vel_upper_lim_fun',{Wrist_len},...
                                     {sqrt((2*l_1*l_2)^2-(l_1^2+l_2^2-(-delta_x-Wrist_len)^2)^2)/(2*(-delta_x-Wrist_len))},...
                                     {'Wrist_len'},{'Joint_5_vel_upper_lim_fun'});
% Beam Link Revolute velocity constraint
Joint_4_vel_upper_lim_fun = Function('Joint_4_vel_upper_lim_fun',{Wrist_len},...
                                     {-((-delta_x-Wrist_len)^2-l_2^2+l_1^2)/((-delta_x-Wrist_len)*sqrt((2*l_2*(-delta_x-Wrist_len))^2-((-delta_x-Wrist_len)^2+l_2^2-l_1^2)^2))},...
                                     {'Wrist_len'},{'Joint_4_vel_upper_lim_fun'});
% Palm Link Revolute velocity constraint
Joint_6_vel_upper_lim_fun = Function('Joint_6_vel_upper_lim_fun',{Wrist_len},...
                                     {-((-delta_x-Wrist_len)^2-l_1^2+l_2^2)/((-delta_x-Wrist_len)*sqrt((2*l_1*(-delta_x-Wrist_len))^2-(l_1^2+(-delta_x-Wrist_len)^2-l_2^2)^2))},...
                                     {'Wrist_len'},{'Joint_6_vel_upper_lim_fun'});

%% Cost Function
Q_input = eye(num_inputs)*Q(1,1);
Q_input(1,1) = 0.0005;
Q_input(2,2) = 0.0005;
Q_input(3,3) = 0.0005;
% Location of Summit's origin
Link_s_center_loc_curr = mat_w_G_s_curr*[0;0;0;1];
% Terminal location of Summit's origin
Link_s_Center_loc_term = [Summit_origin_goal_point;1];
% Objective function (integral term)
h = 1/3*(inputs'*Q_input*inputs); % Input should be minimal. Multiply it by 1/3 as inputs are once per knot point and this component will be repeated as many times as 'd=3'

% All Constraints, Local space velocity, and Cost Function
All_constraints_and_local_space_vel_and_Cost_fun = Function('All_constraints_and_local_space_vel_and_Cost_fun',{states,inputs},...
                                                            {dt_Summit_State_mod,h},...
                                                            {'collocation states kj','knot point inputs k'},{'x_dot','Cost function'});


%% Set up Collocation and NLP Constraints
import casadi.*
% Generate Legendre Polynomial
% Degree of interpolating polynomial
d = 1;

% Get collocation points
tau_root = [0 collocation_points(d, 'legendre')];

% Coefficients of the collocation equation
C = zeros(d+1,d+1);

% Coefficients of the continuity equation
D = zeros(d+1, 1);

% Coefficients of the quadrature function
B = zeros(d+1, 1);

% Construct polynomial basis
for j=1:d+1
    % Construct Lagrange polynomials to get the polynomial basis at the collocation point
    coeff = 1;
    for r=1:d+1
        if r ~= j
            coeff = conv(coeff, [1, -tau_root(r)]);
            coeff = coeff / (tau_root(j)-tau_root(r));
        end
    end
    % Evaluate the polynomial at the final time to get the coefficients of the continuity equation
    D(j) = polyval(coeff, 1.0);
    
    % Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
    pder = polyder(coeff);
    for r=1:d+1
        C(j,r) = polyval(pder, tau_root(r));
    end
    
    % Evaluate he integral of the polynomial to get the coefficients of the quadrature function
    pint = polyint(coeff);
    B(j) = polyval(pint, 1.0);
end

% create CasADi function for discrete-time dynamics using collocation 
% (it should speed up jitting. Adapted from 
% https://gist.github.com/jaeandersson/9bf6773414f30539daa32d9c7deeb441)
% Empty generalized state variable list for only within a time step k
X_j = MX.sym('X_j', (num_dof_Summit*2-1), d); % Collocation state variables

U_k_init = MX.sym('U_k', num_inputs, 1); % action at the start of collocation interval
U_kp = MX.sym('U_kp', num_inputs, 1); % action at the end of collocation interval
X_0_j = MX.sym('X_0_j', (num_dof_Summit*2-1), 1); % state at the beginning of the collocation

% Generalized state at the end of the collocation interval
X_f_j = D(1)*X_0_j;
for j=1:d    
    % add contribution to the end state
    X_f_j = X_f_j + D(j+1)*X_j(:,j);
       
end

% nonlinear equations and cost contribution for collocation interval
eq = {};
% Delta time as a variable
delta_t_var = MX.sym('delta_t_var',1);
% Cost function
J_j = 0;
% collocation and algebraic equations
for j = 1:d
    % expression for the state derivative at the collocation point
    xp = C(1,j+1)*X_0_j;
    for r=1:d
        xp = xp + C(r+1,j+1)*X_j(:,r);
    end
    
    % evaluate the function at the collocation points
    Uc = U_k_init + tau_root(j+1)*(U_kp-U_k_init);
    [x_dot_j, h_j] = All_constraints_and_local_space_vel_and_Cost_fun(X_j(:,j), ...
                                                                      Uc);

    % append collocation equations
    eq = {eq{:}, delta_t_var*x_dot_j - xp};
    % Add contribution to quadrature function
    J_j = J_j + B(j+1)*h_j*delta_t_var;
end

% implicit discrete-time dynamics
Implicit_discrete_time_dynamics_fun = Function('Implicit_discrete_time_dynamics_fun', {X_0_j, X_j, U_k_init, U_kp, delta_t_var}, ...
                                               {vertcat(eq{:}), X_f_j, J_j},...
                                               {'Initial x','Collocation x','Initial input','Final input','Time scaling'},{'NLP constraints','Terminal state','Cost function'});
%% Seed values
x_seed = [-1	0.500000000000000	0	0	0	0	0	0	0	0	0.750000000000000;...
          -0.706113550553128	0.488770416600682	0.0859532478860483	-0.196026578016327	-0.00343481968603493	0.177883568458727	-0.0175245902348721	-1.00013560212412	0.438536979010450	-0.0572937928536611	0.749420660443224;...
          -0.412663923693123	0.467805975790556	0.141566782475204	-0.396026580710382	-0.00343482035110860	0.377883568737341	0.0175245868416391	-0.0202725748863680	-0.154794455596391	-0.0496676398510653	0.747771313332312;...
          -0.119248291641101	0.442673381044179	0.113111590706656	-0.467017408431077	-0.00504512017597474	0.475571251804960	-0.0257404022746296	-0.341925525729421	0.00961490575686060	-0.0785598843651452	0.749247217545352;...
          0.175228516155969	0.409062005369241	0.139943175046233	-0.617922886882673	-0.0240478802965183	0.616292004050014	-0.0712124554832458	-0.428000384737905	0.127280932710368	-0.0929267262212719	0.753185475296838;...
          0.471857009664556	0.375677158491026	0.122508691744233	-0.657853293057954	-0.0491699749308031	0.648621692460099	-0.0569614967324929	0.224273822619123	-0.216232378128737	-0.0774041251981926	0.760225205869422;...
          0.770097881962189	0.345393211553123	0.0412974695937526	-0.563985208595596	-0.0714763638725648	0.573762505284946	-0.0568466101132301	0.254644975658214	-0.198110592026775	-0.0771058081584567	0.761411897689932;...
          1.06830728372456	0.314492783454210	-0.00679520245454266	-0.493551849824282	-0.0915219087246772	0.500665864295848	-0.0454265779077513	0.104708895624000	-0.0472601837298336	-0.0805494372441575	0.760064641914009;...
          1.36607507083847	0.282668240839365	-0.0113263750138915	-0.466023886852242	-0.107922822636927	0.441180318751796	-0.0382515542976048	0.0357398950496737	0.0241419563862170	-0.0818206781377060	0.759158761728393;...
          1.66357501195539	0.250709615939622	-0.00262170073731692	-0.450679813486878	-0.122424415766454	0.389039181446723	-0.0357361657510018	0.0425461935491249	0.0202696470656942	-0.0812335305344524	0.758698080704839;...
          1.96091056252758	0.219012762612063	0.00179016153491005	-0.430588853408811	-0.136379990420210	0.341893387811451	-0.0354657457477566	0.0599587048083589	0.00223985432321888	-0.0804851088918676	0.758320034459415;...
          2.25809406403659	0.187519294169008	0.00131774834453752	-0.406486508693121	-0.150332113484043	0.300719345172104	-0.0357185555983268	0.0630124417002653	-0.00465012570267059	-0.0801958525522899	0.757922320178406;...
          2.55512226366118	0.156086020098235	-0.000392673340820735	-0.382822066363590	-0.164380823488548	0.266048714347646	-0.0359585362613953	0.0577245089606046	-0.00407651554915722	-0.0801779947475763	0.757527677906239];

x_1_seed = [-0.853056775276564	0.494385208300341	0.0429766239430241	-0.0980132890081635	-0.00171740984301746	0.0889417842293636	-0.00876229511743604	-0.500067801062059	0.219268489505225	-0.0286468964268306	0.749710330221612;...
          -0.559388737123125	0.478288196195619	0.113760015180626	-0.296026579363355	-0.00343482001857176	0.277883568598034	-1.69661650195573e-09	-0.510204088505243	0.141871261707030	-0.0534807163523632	0.748595986887768;...
          -0.265956107667112	0.455239678417367	0.127339186590930	-0.431521994570730	-0.00423997026354167	0.426727410271150	-0.00410790771649526	-0.181099050307895	-0.0725897749197650	-0.0641137621081053	0.748509265438832;...
           0.0279901122574341	0.425867693206710	0.126527382876445	-0.542470147656875	-0.0145465002362465	0.545931627927487	-0.0484764288789377	-0.384962955233663	0.0684479192336144	-0.0857433052932085	0.751216346421095;...
           0.323542762910262	0.392369581930134	0.131225933395233	-0.637888089970314	-0.0366089276136607	0.632456848255056	-0.0640869761078693	-0.101863281059391	-0.0444757227091842	-0.0851654257097322	0.756705340583130;...
           0.620977445813373	0.360535185022074	0.0819030806689927	-0.610919250826775	-0.0603231694016840	0.611192098872522	-0.0569040534228615	0.239459399138668	-0.207171485077756	-0.0772549666783246	0.760818551779677;...
           0.919202582843376	0.329942997503667	0.0172511335696050	-0.528768529209939	-0.0814991362986210	0.537214184790397	-0.0511365940104907	0.179676935641107	-0.122685387878304	-0.0788276227013071	0.760738269801970;...
           1.21719117728152	0.298580512146788	-0.00906078873421709	-0.479787868338262	-0.0997223656808021	0.470923091523822	-0.0418390661026780	0.0702243953368369	-0.0115591136718083	-0.0811850576909317	0.759611701821201;...
           1.51482504139693	0.266688928389494	-0.00697403787560422	-0.458351850169560	-0.115173619201690	0.415109750099260	-0.0369938600243033	0.0391430442993993	0.0222058017259556	-0.0815271043360792	0.758928421216616;...
           1.81224278724148	0.234861189275843	-0.000415769601203435	-0.440634333447844	-0.129402203093332	0.365466284629087	-0.0356009557493792	0.0512524491787419	0.0112547506944566	-0.0808593197131600	0.758509057582127;...
           2.10950231328209	0.203266028390536	0.00155395493972378	-0.418537681050966	-0.143356051952127	0.321306366491778	-0.0355921506730417	0.0614855732543121	-0.00120513568972586	-0.0803404807220788	0.758121177318911;...
           2.40660816384889	0.171802657133622	0.000462537501858391	-0.394654287528355	-0.157356468486296	0.283384029759875	-0.0358385459298610	0.0603684753304350	-0.00436332062591391	-0.0801869236499331	0.757724999042322];

u_seed = [0.582107204596483	-8.61413095866382	10.0031147501271	-12.0646943553579	-1.30859344819985;...
          0.241916674125068	-6.52839187421806	-5.88053509327737	4.04813609922467	1.93530509079400;...
          0.482226469650588	-6.11208251619724	-3.76852547737875	2.26638873494528	0.709787240567364;...
          0.849726804275269	-5.11188161408639	5.81692063843318	-4.46204802555925	-1.23270497916690;...
          0.413762359688787	-1.73442935694635	-2.89724259020819	4.51397303212777	0.236855489368150;...
         -0.0666856653612975	-0.663873455744643	-0.708857627925171	1.18083665901306	-0.344757690843841;...
         -0.202935614375348	-0.202969135250833	2.01360480784957	-0.0820382334492542	0.164553208032328;...
         -0.0556410007899801	-0.0651179545341719	1.46731150463140	-0.0847380544232876	0.169725105392447;...
         -0.0232879419106694	-0.0117751577463536	0.255879239295205	0.0851738064783907	0.114047978596967;...
         -0.0146830262713373	0.00350594837355318	-0.189374073243527	0.113828242538293	0.0703836192348586;...
         -0.0108630111414368	0.00374661217803482	-0.114499918042396	0.0573723299342005	0.0439448749935899;...
         -0.00805771729388762	0.00175154643455024	-0.00388583392421855	0.0135712769970544	0.0258795759904169;...
         -0.00750356180580547	0.00115722640060813	0.0216121021333427	0.00212742804031680	0.0178773094438470];
%% Initialize arguments for NLP Solver
import casadi.*
% We want the discrete-time system equation to give us the final states after 0.1 secs worth of constant inputs 
delta_t = 0.1*4;              
% Now, we start generating the Continuous NLP Solver
% Number of Control Intervals
num_steps = size(x_seed,1)-1;
% Here, we will be having both the initial state and the input to the
% system at each time step be a variable. This prevents concatenation of
% the Integrator, increasing convergence rates. This is known as 'Lifting'

% By having the initial states to the discrete-time system as a variable,
% one of the objectives would be to reduce the deviation between the
% realised final state of the previous set of inputs and initial state and

% Start with an empty NLP
% Start with an empty list to contain both the inputs and the initial
% state variables of the system that will be tuned
w={}; % A list of variables
% Initial Guess
w0 = [];
% Start with an empty array of lower limit values for said variable (for
% corresponding variable in list 'w')
lbw = [];
% Start with an empty array of upper limit values for said variable (for
% corresponding variable in list 'w')
ubw = [];
% Start with 0 cost for the cummulative cost function. We will be adding
% the cost function at every time-step
J = 0;
% Start with an empty array to contain the constraints
G={}; % A list of constraints
% Start with an empty array of lower limit values for said constraint (for
% corresponding constraints in list 'G')
lbg = [];
% Start with an empty array of upper limit values for said constraint (for
% corresponding constraints in list 'G')
ubg = [];
% Boolean indicator helping structure detection for Fatrop
equality = [];

% "Lift" initial conditions

% Time scaling factor
var_delta_t = delta_t;

% Generalized coordinates at k
X_k_init = MX.sym('X_0_init', num_dof_Summit*2-1, 1);
w = {w{:}, X_k_init};
lbw = [lbw; [-0.8 0.49 x_seed(1,3:end) zeros(1,(num_dof_Summit-2)) 0.75]'];
ubw = [ubw; [-0.8 0.49 x_seed(1,3:end) zeros(1,(num_dof_Summit-2)) 0.75]'];
w0 = [w0; [-0.8 0.49 x_seed(1,3:end) zeros(1,(num_dof_Summit-2)) 0.75]']; % 
% size(w0)
% size(lbw)

% New NLP variable for control at the begining of the time-step k
U_k_init = MX.sym(['U_0_init'],num_inputs,1);
w = {w{:}, U_k_init};
lbw = [lbw; lower_effort_limits];
ubw = [ubw; upper_effort_limits];
w0 = [w0; u_seed(1,:)'];
% Add constraints on the force inputs
% MB frame effort constraint
G = {G{:},sqrt(U_k_init(1,1)^2+U_k_init(2,1)^2)};
lbg = [lbg;0];
ubg = [ubg; 80];
equality = [equality; false(1, 1)];
% size(w0)
% size(lbw)

% Formulate the NLP
import casadi.*
for k=0:(num_steps-1)
    % Generalized State at collocation points kj
    X_kj =  MX.sym(['X_' num2str(k) '_' num2str(j)], num_dof_Summit*2-1, d);
    w = {w{:}, reshape(X_kj,((num_dof_Summit*2-1)*d),1)};
    lbw = [lbw; repmat(lower_state_limits,d,1) ];
    ubw = [ubw; repmat(upper_state_limits,d,1) ];
%     size(lbw)
    temp_x_w0_collo = zeros((num_dof_Summit*2-1)*d,1);
    
    for j=1:d
        % Generalized state
        temp_x_w0_collo((1+(j-1)*(num_dof_Summit*2-1)):(j*(num_dof_Summit*2-1))) = x_1_seed(k*d+j,:)';

        temp_Xkj = X_kj(:,j);
        % Impose limits on Joint 6 displacement
        % Upper limit
        G = {G{:},Joint_6_Config_2_upper_lim_fun(temp_Xkj(5,1))-temp_Xkj(6,1)};
        lbg = [lbg; 0];
        ubg = [ubg; inf];
        equality = [equality; false(1, 1)];
        % lower limit
        G = {G{:},Joint_6_Config_1_lower_lim_fun(temp_Xkj(5,1))-temp_Xkj(6,1)};
        lbg = [lbg; -inf];
        ubg = [ubg; 0];
        equality = [equality; false(1, 1)];
        % Impose limits on Joint 4 displacement
        % Upper limit
        G = {G{:},Joint_4_Config_2_upper_lim_fun(temp_Xkj(5,1))-temp_Xkj(4,1)};
        lbg = [lbg; 0];
        ubg = [ubg; inf];
        equality = [equality; false(1, 1)];
        % lower limit
        G = {G{:},Joint_4_Config_1_lower_lim_fun(temp_Xkj(5,1))-temp_Xkj(4,1)};
        lbg = [lbg; -inf];
        ubg = [ubg; 0];
        equality = [equality; false(1, 1)];
        % Determine Joint angle velocites
        dt_theta_Xkj = dt_theta_fun(temp_Xkj);
        % Place bounds on the velocity of Joint 5 in order to keep the IK 
        % system solutions valid
        % Upper bound
        G = {G{:}, Joint_5_vel_upper_lim_fun(temp_Xkj(5,1))*0.5-dt_theta_Xkj(5,1)};
        lbg = [lbg; 0];
        ubg = [ubg; inf];
        equality = [equality; false(1, 1)];
        % lower bound
        G = {G{:}, -Joint_5_vel_upper_lim_fun(temp_Xkj(5,1))*0.5-dt_theta_Xkj(5,1)};
        lbg = [lbg; -inf];
        ubg = [ubg; 0];
        equality = [equality; false(1, 1)];
        % Place bounds on the velocity of Joint 4 in order to keep the IK 
        % system solutions valid
        % bound
        % Config 1
        G = {G{:}, -Joint_4_vel_upper_lim_fun(temp_Xkj(5,1))*dt_theta_Xkj(5,1)+dt_theta_Xkj(4,1)};
        lbg = [lbg; -0.5];
        ubg = [ubg; 0.5];
        equality = [equality; false(1, 1)];
        % Config 2
        G = {G{:}, Joint_4_vel_upper_lim_fun(temp_Xkj(5,1))*dt_theta_Xkj(5,1)+dt_theta_Xkj(4,1)};
        lbg = [lbg; -0.5];
        ubg = [ubg; 0.5];
        equality = [equality; false(1, 1)];
        % Place bounds on the velocity of Joint 6 in order to keep the IK 
        % system solutions valid
        % bound
        % Config 1
        G = {G{:}, -Joint_6_vel_upper_lim_fun(temp_Xkj(5,1))*dt_theta_Xkj(5,1)+dt_theta_Xkj(6,1)};
        lbg = [lbg; -0.5];
        ubg = [ubg; 0.5];
        equality = [equality; false(1, 1)];
        % Config 2
        G = {G{:}, Joint_6_vel_upper_lim_fun(temp_Xkj(5,1))*dt_theta_Xkj(5,1)+dt_theta_Xkj(6,1)};
        lbg = [lbg; -0.5];
        ubg = [ubg; 0.5];
        equality = [equality; false(1, 1)];
        
    end
    w0 = [w0; temp_x_w0_collo];

    % New NLP variable for Generalized state at end of interval
    X_k_fin = MX.sym(['X_' num2str(k+1) '_fin'], (num_dof_Summit*2-1), 1);
    w = {w{:}, X_k_fin};
    lbw = [lbw; lower_state_limits];
    ubw = [ubw; upper_state_limits];
    % Add initial values for the Xk variable
    w0 = [w0; x_seed(2+k,:)'];
%     size(w0)
%     size(lbw)

    % New NLP variable for control at the end of the time-step k
    U_k_fin = MX.sym(['U_' num2str(k+1) '_fin'],num_inputs,1);
    w = {w{:}, U_k_fin};
    lbw = [lbw; lower_effort_limits];
    ubw = [ubw; upper_effort_limits];
    w0 = [w0;  u_seed(2+k,:)'];
%     size(w0)
%     size(lbw)

    % values for detected point
    % Points_seen_kp = MX.sym(['Points_seen_' num2str(k+1)],num_points_length*num_points_width,1);
    % w = {w{:}, Points_seen_kp};
    % lbw = [lbw; -inf(num_points_length*num_points_width,1)];
    % ubw = [ubw; inf(num_points_length*num_points_width,1)];
    % w0 = [w0;  zeros(num_points_length*num_points_width,1)];

    % Get Collocation constraints
    [eq_k,X_k_end,J_k] = Implicit_discrete_time_dynamics_fun(X_k_init,... % Initial x_k
                                                             X_kj,... % Y_kj
                                                             U_k_init,... % Initial input
                                                             U_k_fin,... % Final input
                                                             var_delta_t); % Delta time
    % Append the collocation constraints
    G = {G{:},eq_k};
    lbg = [lbg; zeros((num_dof_Summit*2-1)*d,1)];
    ubg = [ubg; zeros((num_dof_Summit*2-1)*d,1)];
    equality = [equality; true((num_dof_Summit*2-1)*d, 1)];
%     size(lbg)
    
    % Continuity conditions at the terminal point of a collocation
    % polynomial with the start of the next collocation polynomial
    G = {G{:}, X_k_fin-X_k_end};
    lbg = [lbg; zeros((num_dof_Summit*2-1),1)];
    ubg = [ubg; zeros((num_dof_Summit*2-1),1)];
    equality = [equality; true((num_dof_Summit*2-1), 1)];
%     size(lbg)

    % Cart
    cart_frame_curr = mat_w_G_c4_curr_fun(X_k_fin);

    
    % Add constraints on the linear velocity
    % MB frame velocity constraint
    % Joint velocity
    dt_theta_X_kp = null_space_paffian_constraint_fun(X_k_fin)*X_k_fin((num_dof_Summit+1):(num_dof_Summit*2-1),1);
    G = {G{:},dt_theta_X_kp(1,1)^2+dt_theta_X_kp(2,1)^2};
    lbg = [lbg; 0];
    ubg = [ubg; vel_limit_MB_trans^2];
    equality = [equality; false(1, 1)];

    % Add constraints on the force inputs
    % MB frame effort constraint
    G = {G{:},sqrt(U_k_fin(1,1)^2+U_k_fin(2,1)^2)};
    lbg = [lbg;  0];
    ubg = [ubg; effort_limit_MB_trans];
    equality = [equality; false(1, 1)];

    % Add constraint on Cart-MB collision avoidance
    % MB Point-of-interest
    mat_w_G_s_curr_X_kp = mat_w_G_s_curr_fun(X_k_fin);
    Point_Summit_1_curr_X_kp = mat_w_G_s_curr_X_kp*[Point_Summit_1;1];
    Point_Summit_2_curr_X_kp = mat_w_G_s_curr_X_kp*[Point_Summit_2;1];
    % Cart Point-of-interest
    mat_w_G_c0_curr_X_kp = mat_w_G_c0_curr_fun(X_k_fin);
    Point_Cart_1_curr_X_kp = mat_w_G_c0_curr_X_kp*[Point_Cart_1;1];
    Point_Cart_2_curr_X_kp = mat_w_G_c0_curr_X_kp*[Point_Cart_2;1];
    Point_Cart_3_curr_X_kp = mat_w_G_c0_curr_X_kp*[Point_Cart_3;1];
    % difference
    diff_X_kp = [Point_Summit_1_curr_X_kp(1:2,1)';...
                 Point_Summit_1_curr_X_kp(1:2,1)';...
                 Point_Summit_1_curr_X_kp(1:2,1)';...
                 Point_Summit_2_curr_X_kp(1:2,1)';...
                 Point_Summit_2_curr_X_kp(1:2,1)';...
                 Point_Summit_2_curr_X_kp(1:2,1)']-...
                [Point_Cart_1_curr_X_kp(1:2,1)';...
                 Point_Cart_2_curr_X_kp(1:2,1)';...
                 Point_Cart_3_curr_X_kp(1:2,1)';...
                 Point_Cart_3_curr_X_kp(1:2,1)';...
                 Point_Cart_2_curr_X_kp(1:2,1)';...
                 Point_Cart_1_curr_X_kp(1:2,1)'];
    % MB 1 vs Cart 1
    G = {G{:},sqrt(diff_X_kp(1,1)^2+diff_X_kp(1,2)^2)};
    lbg = [lbg;  (Radius_Summit+Radius_Cart)];
    ubg = [ubg; inf];
    equality = [equality; false(1, 1)];
    % MB 1 vs Cart 2
    G = {G{:},sqrt(diff_X_kp(2,1)^2+diff_X_kp(2,2)^2)};
    lbg = [lbg;  (Radius_Summit+Radius_Cart)];
    ubg = [ubg; inf];
    equality = [equality; false(1, 1)];
    % MB 1 vs Cart 3
    G = {G{:},sqrt(diff_X_kp(3,1)^2+diff_X_kp(3,2)^2)};
    lbg = [lbg;  (Radius_Summit+Radius_Cart)];
    ubg = [ubg; inf];
    equality = [equality; false(1, 1)];
    % MB 2 vs Cart 3
    G = {G{:},sqrt(diff_X_kp(4,1)^2+diff_X_kp(4,2)^2)};
    lbg = [lbg;  (Radius_Summit+Radius_Cart)];
    ubg = [ubg; inf];
    equality = [equality; false(1, 1)];
    % MB 2 vs Cart 2
    G = {G{:},sqrt(diff_X_kp(5,1)^2+diff_X_kp(5,2)^2)};
    lbg = [lbg;  (Radius_Summit+Radius_Cart)];
    ubg = [ubg; inf];
    equality = [equality; false(1, 1)];
    % MB 2 vs Cart 1
    G = {G{:},sqrt(diff_X_kp(6,1)^2+diff_X_kp(6,2)^2)};
    lbg = [lbg;  (Radius_Summit+Radius_Cart)];
    ubg = [ubg; inf];
    equality = [equality; false(1, 1)];

    % Sum up the cost function
    J = J + var_delta_t/3*(U_k_init'*Q_input*U_k_init+U_k_fin'*Q_input*U_k_fin+U_k_init'*Q_input*U_k_fin) + ...
            var_delta_t/2*(U_k_fin-U_k_init)'*Q_input*(U_k_fin-U_k_init) + ... Q(2,1)*var_delta_t^2 + ... % large Time step penalty 
            Q(3,1)*X_k_fin(3,1)^2 + ... % MB orientation deviation penalty
            Q(6,1)*(cart_frame_curr(2,4)-0.5)^2; % Cart lateral deviation penalty
       
    % The initial observation state to the subsequent time-step
%     Points_seen_k = Points_seen_kp;
    if(k==(num_steps-1))
        Link_s_center_loc_curr = mat_w_G_s_curr_fun(X_k_fin)*[0;0;0;1];
%         J = J + 10^3*(Link_s_Center_loc_term(1:2,1) - Link_s_center_loc_curr(1:2,1))'*(Link_s_Center_loc_term(1:2,1) - Link_s_center_loc_curr(1:2,1));
        % Terminate the trajectory at the terminal position
        G = {G{:},Link_s_Center_loc_term(1,1) - Link_s_center_loc_curr(1,1)};
        lbg = [lbg; [-inf]];
        ubg = [ubg; zeros(1,1)];
        equality = [equality; false(1, 1)];
        % Terminate the trajectory with the cart directly behind the summit
        % G = {G{:},X_k(4,1)};
        % lbg = [lbg; zeros(1,1)];
        % ubg = [ubg; zeros(1,1)];
        % Set the terminal linear and angular velocity of the summit to zero
        % G = {G{:},dt_theta_X_kp(1:3,1)};
        % lbg = [lbg; zeros(3,1)];
        % ubg = [ubg; zeros(3,1)];
    else
        % New NLP variable for Generalized state at end of interval
        X_kp_init = MX.sym(['X_' num2str(k+1) '_init'], (num_dof_Summit*2-1), 1);
        w = {w{:}, X_kp_init};
        lbw = [lbw; lower_state_limits];
        ubw = [ubw; upper_state_limits];
        % Add initial values for the Xk variable
        w0 = [w0; x_seed(2+k,:)'];
    %     size(w0)
    %     size(lbw)
    
        % New NLP variable for control at the end of the time-step k
        U_kp_init = MX.sym(['U_' num2str(k+1) '_init'],num_inputs,1);
        w = {w{:}, U_kp_init};
        lbw = [lbw; lower_effort_limits];
        ubw = [ubw; upper_effort_limits];
        w0 = [w0;  u_seed(2+k,:)'];
    %     size(w0)
    %     size(lbw)
        
        % Continuity conditions at the terminal point of a collocation
        % polynomial with the start of the next collocation polynomial
        G = {G{:}, X_kp_init-X_k_fin};
        lbg = [lbg; zeros((num_dof_Summit*2-1),1)];
        ubg = [ubg; zeros((num_dof_Summit*2-1),1)];
        equality = [equality; true((num_dof_Summit*2-1), 1)];
    %     size(lbg)
        % Continuity conditions for the terminal input for the last
        % time-step and the initial input for the new time-step.
        G = {G{:}, U_kp_init-U_k_fin};
        lbg = [lbg; zeros(num_inputs,1)];
        ubg = [ubg; zeros(num_inputs,1)];
        equality = [equality; true(num_inputs, 1)];
    %     size(lbg)

        % The initial state of the subsequent time-step 
        X_k_init = X_kp_init;
        % The initial input to the subsequent time-step
        U_k_init = U_kp_init;

        
    end
end
%% Solve the NLP
import casadi.*
% % Load seed w0
% load("Summit_Cart_12_dof_NLP_V5_6_unbounded_projection_jump.mat","w_opt")
% w0 = w_opt;
% Options
opts = struct;
opts.structure_detection = 'auto';
opts.debug = true;
opts.equality = equality;
% opts.ipopt.print_level = 6;
%opts.ipopt.max_iter = 400;
% opts.ipopt.tol = 1e-8;
% opts.ipopt.constr_viol_tol = 1e-8;
% opts.ipopt.acceptable_tol = 1e-2;
% opts.ipopt.acceptable_obj_change_tol = 10;
% Create an NLP solver
if(optimize_and_record)
    prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(G{:}));
    solver = nlpsol('solver', 'fatrop', prob,opts);
    sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                'lbg', lbg, 'ubg', ubg);
    w_opt = full(sol.x);
end
%% Post Processing
if(optimize_and_record)
    delta = ((num_dof_Summit*2-1)+num_inputs+(num_dof_Summit*2-1)*d);
    initial_index = 0;
    % Time scaling
    delta_t_opt = delta_t;%[w_opt(1)];
    % State at Knot point
    x_opt = [w_opt(1:delta:end) ...
             w_opt(2:delta:end) ...
             w_opt(3:delta:end) ...
             w_opt(4:delta:end) ...
             w_opt(5:delta:end) ...
             w_opt(6:delta:end) ...
             w_opt(7:delta:end) ...
             w_opt(8:delta:end) ...
             w_opt(9:delta:end) ...
             w_opt(10:delta:end) ...
             w_opt(11:delta:end)];
    % Input at Collocation point          
    u_opt = [w_opt(12:delta:end) ...
             w_opt(13:delta:end) ...
             w_opt(14:delta:end) ...
             w_opt(15:delta:end) ...
             w_opt(16:delta:end)];
    
    % State at Collocation point
    x1_opt = [w_opt(17:delta:end) ...
              w_opt(18:delta:end) ...
              w_opt(19:delta:end) ...
              w_opt(20:delta:end) ...
              w_opt(21:delta:end) ...
              w_opt(22:delta:end) ...
              w_opt(23:delta:end) ...
              w_opt(24:delta:end) ...
              w_opt(25:delta:end) ...
              w_opt(26:delta:end) ...
              w_opt(27:delta:end)];
else
    % Load previous results, if they exist
    load(strcat(file_name,'.mat'))
end

























