#! /usr/bin/env python
from casadi import *
import numpy as np

class Cart_MPC(object):
    def __init__(self, k_traj=10, k_input=0.1, k_input_var=2, delta_t=0.5, num_steps=15):
        # Number of time-steps
        self.num_steps = num_steps
        # Time step-size
        self.delta_t = delta_t # (sec)
        # Cart Width
        cart_Link_width = 0.6 # (m)
        # Number of Cart poses
        num_cart_poses = 3
        # Number of Cart wrench inputs
        num_cart_inputs = 2
        # Number of dof when using hand: position, and velocity
        num_dof = 2
        self.delta = num_dof+num_cart_poses+num_cart_inputs+num_cart_inputs

        # An initial-guess of cart dynamic-properties
        # Cart Length
        cart_Link_length = 1 # (m)
        # Dynamics parameters
        m = 4 # (kg) mass
        I_com_z = 1/12*m*(cart_Link_width**2+cart_Link_length**2)
        I_c0 = I_com_z+1/2*m*(cart_Link_length/2)**2
        # Wheel roller damping
        c_1 = 0.1 # (N*sec/m) left-wheel dampening
        c_2 = 0.2 # (N*sec/m) right-wheel dampening
        # Cart way-points to track
        cart_pos_waypoints = np.zeros((self.num_steps,num_cart_poses))
        cart_pos_waypoints[:,0] = np.arange(0,self.num_steps)/10
        # An initial-guess of cart positions over MPC-window
        cart_pos_mpc_window = np.zeros((self.num_steps+1,num_cart_poses))
        cart_pos_mpc_window[1:,0] = np.arange(0,self.num_steps)/10
        # An initial-guess of cart velocities over MPC-window
        cart_vel_mpc_window = np.zeros((self.num_steps+1,num_cart_inputs))
        cart_vel_mpc_window[1:,0] = np.arange(0,self.num_steps)/10
        # An initial-guess of cart inputs over MPC-window
        cart_input_mpc_window = np.zeros((self.num_steps+1,num_cart_inputs))
        cart_input_mpc_window[1:,0] = np.ones(self.num_steps)
        # An initial-guess of hand-states over MPC-window
        hand_state_mpc_window = np.zeros((self.num_steps+1,num_dof))

        # Looping parameter-estimates back into the MPC
        # Number of system parameters to be given
        num_sys_param = 8
        # Number of unknowns
        num_unkw = 4

        # Cost-component weights
        Q = np.array([[k_traj],[k_input],[k_input_var]])
        # Hand mass
        mass_hand = 1 # (kg)
        # Force and torque limits
        force_lim = 20 # (N)
        torque_lim = 5 # (Nm)
        # Hand-exerted lateral, normal, and friction-force limits
        lat_force_max_lim = 2.0 # (N)
        norm_force_max_lim = 15.0 # (N)
        # Hand lateral velocity limit
        lat_vel_lim = 0.1 # (m/sec)
        # Effort limits                          N           |         N         |
        lower_effort_limits = np.array([[-norm_force_max_lim],[-lat_force_max_lim]])
        upper_effort_limits = np.array([[ norm_force_max_lim],[lat_force_max_lim]])
        # Hand State limits                      m         |        m/sec
        lower_hand_state_limits = np.array([[-cart_Link_width/2],[-lat_vel_lim]])
        upper_hand_state_limits = np.array([[ cart_Link_width/2],[ lat_vel_lim]])

        # Set up cost function
        # State: Hand position, and Hand velocity
        # Initial
        l_k = MX.sym('states_k',num_dof,1)
        # Final
        l_kp = MX.sym('states_kp',num_dof,1)
        # Inputs: Grip normal-reaction, Grip friction, Lateral-force for moving hand
        # Initial
        u_k = MX.sym('inputs_k',num_cart_inputs,1)
        # Final
        u_kp = MX.sym('inputs_kp',num_cart_inputs,1)
        # Next cart-rear-axle mid-point position (x,y) and orientation: 
        cart_pose_nxt = MX.sym('cart_pose_nxt',num_cart_poses,1)
        # Previous cart-rear-axle mid-point position (x,y) and orientation: 
        cart_pose_prv = MX.sym('cart_pose_prv',num_cart_poses,1)
        # Next cart-rear-axle mid-point linear and angular velocity:
        cart_vel_nxt = MX.sym('cart_vel_nxt',num_cart_inputs,1)
        # Previous cart-rear-axle mid-point linear and angular velocity:
        cart_vel_prv = MX.sym('cart_vel_prev',num_cart_inputs,1)
        # Next Demanded cart-rear-axle mid-point position and orientation
        cart_pose_d = MX.sym('cart_pose_d',num_cart_poses,1)
        # Cart-handle collision impulse
        cart_handle_col_imp = MX.sym('cart_handle_col_imp',1,1)

        # System parameters
        # eigen-vector (4) | eigen-values (2) | mass (1) | inertia (1) 
        sys_param = MX.sym('sys_param',num_sys_param,1)
        # Eigen-vector matrix
        eig_vec_mat = MX.zeros(num_cart_inputs,num_cart_inputs)
        eig_vec_mat[0,0] = sys_param[0,0]
        eig_vec_mat[0,1] = sys_param[2,0]
        eig_vec_mat[1,0] = sys_param[1,0]
        eig_vec_mat[1,1] = sys_param[3,0]
        inv_eig_vec_mat = solve(eig_vec_mat, MX.eye(num_cart_inputs))
        # Eigen-values
        eig_val = sys_param[4:6,0]
        # Inverted-Mass-inertia matrix
        inv_m_i_mat =  MX.zeros(num_cart_inputs,num_cart_inputs)
        inv_m_i_mat[0,0] = 1/sys_param[6,0]
        inv_m_i_mat[1,1] = 1/sys_param[7,0]

        # Initial wrench at the Cart rear-axile mid-point
        #                 Normal-Force, Torque as a result of Normal-force
        init_wrench = vertcat(u_k[0,0],l_k[0,0]*u_k[0,0])
        # Final wrench at the Cart rear-axile mid-point
        #                 Normal-Force, Torque as a result of Normal-force
        final_wrench = vertcat(u_kp[0,0],l_kp[0,0]*u_kp[0,0])
        # Final Cart Velocity
        lambda_1_exp_1 = exp(eig_val[0,0]*delta_t)
        lambda_2_exp_1 = exp(eig_val[1,0]*delta_t)
        lambda_exp_1_mat = MX.zeros(num_cart_inputs,num_cart_inputs)
        lambda_exp_1_mat[0,0] = lambda_1_exp_1
        lambda_exp_1_mat[1,1] = lambda_2_exp_1
        lambda_1_exp_2 = lambda_1_exp_1-1
        lambda_2_exp_2 = lambda_2_exp_1-1
        lambda_exp_2_mat = MX.zeros(num_cart_inputs,num_cart_inputs)
        lambda_exp_2_mat[0,0] = lambda_1_exp_2/eig_val[0,0]
        lambda_exp_2_mat[1,1] = lambda_2_exp_2/eig_val[1,0]
        lambda_1_exp_3 = lambda_1_exp_2-eig_val[0,0]*delta_t
        lambda_2_exp_3 = lambda_2_exp_2-eig_val[1,0]*delta_t
        lambda_exp_3_mat = MX.zeros(num_cart_inputs,num_cart_inputs)
        lambda_exp_3_mat[0,0] = lambda_1_exp_3/(eig_val[0,0]**2)
        lambda_exp_3_mat[1,1] = lambda_2_exp_3/(eig_val[1,0]**2)
        lambda_1_exp_4 = lambda_1_exp_3-(eig_val[0,0]**2)*(delta_t**2)/2
        lambda_2_exp_4 = lambda_2_exp_3-(eig_val[1,0]**2)*(delta_t**2)/2
        lambda_exp_4_mat = MX.zeros(num_cart_inputs,num_cart_inputs)
        lambda_exp_4_mat[0,0] = lambda_1_exp_4/(eig_val[0,0]**3)
        lambda_exp_4_mat[1,1] = lambda_2_exp_4/(eig_val[1,0]**3)
        lambda_1_exp_5 = lambda_1_exp_4-(eig_val[0,0]**3)*(delta_t**3)/6
        lambda_2_exp_5 = lambda_2_exp_4-(eig_val[1,0]**3)*(delta_t**3)/6
        temp_mult_0_0 = mtimes(inv_eig_vec_mat,cart_vel_prv)
        temp_mult_0_1 = mtimes(inv_eig_vec_mat,mtimes(inv_m_i_mat,init_wrench))
        temp_mult_0_2 = mtimes(inv_eig_vec_mat,mtimes(inv_m_i_mat,(final_wrench-init_wrench)/delta_t))
        temp_summation_0 = mtimes(lambda_exp_1_mat,temp_mult_0_0)+mtimes(lambda_exp_2_mat,temp_mult_0_1)+mtimes(lambda_exp_3_mat,temp_mult_0_2)
        final_vel = mtimes(eig_vec_mat,temp_summation_0)

        final_vel_fun = Function('final_vel_fun',[cart_vel_prv,u_k,l_k,u_kp,l_kp,sys_param],
                                 [final_vel],
                                 ['Cart vel prev','Hand forces prev','Hand state prev',
                                  'Hand forces next','Hand state next','System parameters'],
                                 ['Final velocity'])
        # Final Cart displacement
        temp_summation_1 = mtimes(lambda_exp_2_mat,temp_mult_0_0)+mtimes(lambda_exp_3_mat,temp_mult_0_1)+mtimes(lambda_exp_4_mat,temp_mult_0_2)
        final_disp = mtimes(eig_vec_mat,temp_summation_1)
        
        final_disp_fun = Function('final_disp_fun',[cart_vel_prv,u_k,l_k,u_kp,l_kp,sys_param],
                                  [final_disp],
                                  ['Cart vel prev','Hand forces prev','Hand state prev',
                                   'Hand forces next','Hand state next','System parameters'],
                                  ['Final displacement'])
        # System matrix
        sys_mat = vertcat(horzcat(MX.eye(num_cart_inputs), mtimes(eig_vec_mat,mtimes(lambda_exp_2_mat,inv_eig_vec_mat))),
                          horzcat(MX.zeros(num_cart_inputs,num_cart_inputs), mtimes(eig_vec_mat,mtimes(lambda_exp_1_mat,inv_eig_vec_mat))))
        sys_mat_fun = Function('sys_mat_fun',[sys_param],
                               [sys_mat],
                               ['System parameters'],
                               ['System matrix'])
        # Cart position constraint
        # Previous Cart pose
        w_G_cart_prev = MX.eye(3)
        w_G_cart_prev[0,0] = cos(cart_pose_prv[2,0])
        w_G_cart_prev[0,1] = -sin(cart_pose_prv[2,0])
        w_G_cart_prev[0,2] = cart_pose_prv[0,0]
        w_G_cart_prev[1,0] = sin(cart_pose_prv[2,0])
        w_G_cart_prev[1,1] = cos(cart_pose_prv[2,0])
        w_G_cart_prev[1,2] = cart_pose_prv[1,0]
        # Delta Cart pose
        cart_prev_G_cart_nxt = MX.eye(3)
        cart_prev_G_cart_nxt[0,0] = cos(final_disp[1,0])
        cart_prev_G_cart_nxt[0,1] = -sin(final_disp[1,0])
        cart_prev_G_cart_nxt[0,2] = final_disp[0,0]
        cart_prev_G_cart_nxt[1,0] = sin(final_disp[1,0])
        cart_prev_G_cart_nxt[1,1] = cos(final_disp[1,0])

        # Next Cart pose
        w_G_cart_nxt = mtimes(w_G_cart_prev,cart_prev_G_cart_nxt)
        # Cart pose constraints
        #                                       position (x,Y)     , Prev-state yaw-orientation + yaw-displacement
        cart_pose_const = cart_pose_nxt-vertcat(w_G_cart_nxt[0:2,2],cart_pose_prv[2,0]+final_disp[1,0])

        # Cart velocity constraint
        cart_vel_const = cart_vel_nxt - final_vel

        # Cost of deviating from planned cart trajectory
        cost_traj_dev = mtimes(transpose(cart_pose_d - cart_pose_nxt),(cart_pose_d - cart_pose_nxt))
        # Cost of high input
        input_mag_cost = mtimes(transpose(u_k),u_k) + mtimes(transpose(u_kp),u_kp) # High-input cost
        input_var_cost = mtimes(transpose(u_kp-u_k),(u_kp-u_k))*Q[2,0]      # High-variation cost 
        cost_input = input_mag_cost+input_var_cost

        # Total cost
        total_cost = Q[0,0]*cost_traj_dev+Q[1,0]*cost_input
        # Total cost function
        Cost_fun = Function('Cost_fun',[cart_pose_nxt,cart_pose_d,u_k,u_kp],
                            [total_cost],
                            ['Cart pose','Demanded cart pose','Initial input','Final input'],['Cost function'])
        # Hand velocity
        # With Impulse
        final_hand_vel_w_imp = (u_kp[1,0]-u_k[1,0])/(2*delta_t*mass_hand)*(delta_t**2)+u_k[1,0]*delta_t/mass_hand+l_k[1,0]-2*l_k[0,0]*cart_handle_col_imp
        # Without Impulse
        final_hand_vel_wo_imp = (u_kp[1,0]-u_k[1,0])/(2*delta_t*mass_hand)*(delta_t**2)+u_k[1,0]*delta_t/mass_hand+l_k[1,0]
        # Hand displacement
        # With Impulse
        final_hand_disp_w_imp = (u_kp[1,0]-u_k[1,0])/(6*delta_t*mass_hand)*(delta_t**3)+u_k[1,0]/(2*mass_hand)*(delta_t**2)+l_k[1,0]*delta_t+l_k[0,0]+(-2*l_k[0,0])*cart_handle_col_imp*delta_t
        # Without Impulse
        final_hand_disp_wo_imp = (u_kp[1,0]-u_k[1,0])/(6*delta_t*mass_hand)*(delta_t**3)+u_k[1,0]/(2*mass_hand)*(delta_t**2)+l_k[1,0]*delta_t+l_k[0,0]
        # Hand constraint
        # With Impulse
        hand_const_w_imp = vertcat(l_kp[1,0]-final_hand_vel_w_imp,
                                   l_kp[0,0]-final_hand_disp_w_imp,
                                   ((cart_Link_width/2)**2-final_hand_disp_w_imp)*cart_handle_col_imp)
        # Without Impulse
        hand_const_wo_imp = vertcat(l_kp[1,0]-final_hand_vel_wo_imp,
                                    l_kp[0,0]-final_hand_disp_wo_imp)
        # Constraints
        # With Impulse
        const_w_imp = vertcat(cart_pose_const,
                              cart_vel_const,
                              hand_const_w_imp)
        # Without Impulse
        const_wo_imp = vertcat(cart_pose_const,
                               cart_vel_const,
                               hand_const_wo_imp)
        # Constraint and cost function
        # With Impulse
        Const_and_Cost_w_imp_fun = Function('Const_and_Cost_w_imp_fun',[l_k,l_kp,u_k,u_kp,cart_pose_prv,cart_pose_nxt,cart_vel_prv,cart_vel_nxt,cart_handle_col_imp,cart_pose_d,sys_param],
                                            [const_w_imp,total_cost],
                                            ['Hand state prev','Hand state next',
                                             'Hand forces prev','Hand forces next',
                                             'Cart pose prev','Cart pose next',
                                             'Cart vel prev','Cart vel next','Impulse',
                                             'Cart pose Demanded','System parameters'],['Motion constraint', 'Cost function'])
        # Without Impulse
        Const_and_Cost_wo_imp_fun = Function('Const_and_Cost_w_imp_fun',[l_k,l_kp,u_k,u_kp,cart_pose_prv,cart_pose_nxt,cart_vel_prv,cart_vel_nxt,cart_pose_d,sys_param],
                                             [const_wo_imp,total_cost],
                                             ['Hand state prev','Hand state next',
                                              'Hand forces prev','Hand forces next',
                                              'Cart pose prev','Cart pose next',
                                              'Cart vel prev','Cart vel next',
                                              'Cart pose Demanded','System parameters'],['Motion constraint', 'Cost function'])
        
        # Initialize arguments for NLP Solver
        # Now, we start generating the Continuous NLP Solver
        # Here, we will be having both the initial state and the input to the system at each time step be a variable. This prevents concatenation of
        # the Integrator, increasing convergence rates. This is known as 'Lifting'.

        # By having the initial states to the discrete-time system as a variable, one of the objectives would be to reduce the deviation between the
        # realised final state of the previous set of inputs and initial state and

        # Start with an empty NLP
        # Start with an empty list to contain both the inputs and the initial
        # state variables of the system that will be tuned
        w = [] # A list of variables
        # Initial Guess
        w0 = []
        # Start with an empty array of lower limit values for said variable (for corresponding variable in list 'w')
        self.lbw = []
        # Start with an empty array of upper limit values for said variable (for corresponding variable in list 'w')
        self.ubw = []
        # Start with 0 cost for the cummulative cost function. We will be adding the cost function at every time-step
        J = 0
        # Start with an empty array to contain the constraints
        G = [] # A list of constraints
        # Start with an empty array of lower limit values for said constraint (for corresponding constraints in list 'G')
        self.lbg = []
        # Start with an empty array of upper limit values for said constraint (for corresponding constraints in list 'G')
        self.ubg = []
        # A set of position reference values to follow. This will be provided by the user
        P = []
        # A set of system-parameters and an initial set of reference points
        p0 = []

        # A reference point to follow
        P_0 = MX.sym('P_'+str(0), num_sys_param, 1)
        P = P_0
        # Calculate system parameters
        A_mat = np.array([[-(c_1+c_2)/m, (c_1-c_2)*cart_Link_width/(2*m)],
                          [(c_1-c_2)*cart_Link_width/(2*I_c0), -(c_1+c_2)*(cart_Link_width**2)/(4*I_c0)]])
        a_mat_eig_val, a_mat_eig_vec = np.linalg.eig(A_mat)
        lambda_1 = a_mat_eig_val[0]
        lambda_2 = a_mat_eig_val[1]
        # Add set of system-parameters
        p0 = p0 + [a_mat_eig_vec[0,0],a_mat_eig_vec[1,0],a_mat_eig_vec[0,1],a_mat_eig_vec[1,1], # Eigen-vectors
                   lambda_1,lambda_2, # Eigen values
                   m,I_c0] # mass, inertia 
        
        # "Lift" initial conditions
        # Collision Impulse
        Col_imp_0 = MX.sym('Col_imp_0', 1, 1)
        w = Col_imp_0
        self.lbw.append(0.0)
        self.ubw.append(np.inf)
        # Add initial values for the Col_imp_0 variable
        w0.append(0.0)
        # print(len(w0))

        # Generalized coordinates at k
        # Hand state: position and velocity
        L_k = MX.sym('L_0', num_dof, 1)
        w = vertcat(w, L_k)
        self.lbw = self.lbw + hand_state_mpc_window[0,:].tolist()
        self.ubw = self.ubw + hand_state_mpc_window[0,:].tolist()
        w0 = w0 + hand_state_mpc_window[0,:].tolist()
        # print(len(w0))
        # print(len(lbw))

        # Cart pose
        C_k = MX.sym('C_0', num_cart_poses, 1)
        w = vertcat(w, C_k)
        self.lbw = self.lbw + cart_pos_mpc_window[0,:].tolist()
        self.ubw = self.ubw + cart_pos_mpc_window[0,:].tolist()
        w0 = w0 + cart_pos_mpc_window[0,:].tolist()
        # print(len(w0))
        # print(len(lbw))

        # Cart velocity
        C_vel_k = MX.sym('C_vel_0', num_cart_inputs, 1)
        w = vertcat(w, C_vel_k)
        self.lbw = self.lbw + cart_vel_mpc_window[0,:].tolist()
        self.ubw = self.ubw + cart_vel_mpc_window[0,:].tolist()
        w0 = w0 + cart_vel_mpc_window[0,:].tolist()
        # print(len(w0))
        # print(len(lbw))

        # New NLP variable for control at the begining of the time-step k
        U_k = MX.sym('U_0',num_cart_inputs,1)
        w = vertcat(w, U_k)
        self.lbw = self.lbw + cart_input_mpc_window[0,:].tolist()
        self.ubw = self.ubw + cart_input_mpc_window[0,:].tolist()
        w0 = w0 + cart_input_mpc_window[0,:].tolist()
        # print(len(w0))
        # print(len(lbw))

        # Formulate the NLP
        for k in range(0,num_steps):
            # A reference point to follow
            P_k = MX.sym('P_'+str(k+1), num_cart_poses, 1)
            P = vertcat(P, P_k)
            # Add initial values for the P_k variable
            p0 = p0 + cart_pos_waypoints[k,:].tolist()

            # Hand state: position and velocity
            L_kp = MX.sym('L_'+str(k+1), num_dof, 1)
            w = vertcat(w, L_kp)
            if(k==0):
                self.lbw = self.lbw + [-np.inf,lower_hand_state_limits[1,0]]
                self.ubw = self.ubw + [np.inf,upper_hand_state_limits[1,0]]
            else:
                self.lbw = self.lbw + lower_hand_state_limits[:,0].tolist()
                self.ubw = self.ubw + upper_hand_state_limits[:,0].tolist()
            
            # Add initial values for the L_kp variable
            w0 = w0 + hand_state_mpc_window[k+1,:].tolist()
            # print(len(w0))
            # print(len(lbw))
            
            # Cart pose
            C_kp = MX.sym('C_'+str(k+1), num_cart_poses, 1)
            w = vertcat(w, C_kp)
            temp_inf_array_cp = np.ones((num_cart_poses))*np.inf
            self.lbw = self.lbw + (-temp_inf_array_cp).tolist()
            self.ubw = self.ubw + temp_inf_array_cp.tolist()
            # Add initial values for the C_kp variable
            w0 = w0 + cart_pos_mpc_window[k+1,:].tolist()
            # print(len(w0))
            # print(len(lbw))

            # Cart velocity
            C_vel_kp = MX.sym('C_vel_'+str(k+1), num_cart_inputs, 1)
            w = vertcat(w, C_vel_kp)
            temp_inf_array_cv = np.ones((num_cart_inputs))*np.inf
            self.lbw = self.lbw + (-temp_inf_array_cv).tolist()
            self.ubw = self.ubw + temp_inf_array_cv.tolist()
            w0 = w0 + cart_vel_mpc_window[k+1,:].tolist()
            # print(len(w0))
            # print(len(lbw))

            # New NLP variable for control at the end of the time-step k
            U_kp = MX.sym('U_'+str(k+1),num_cart_inputs,1)
            w = vertcat(w, U_kp)
            self.lbw = self.lbw + lower_effort_limits[:,0].tolist()
            self.ubw = self.ubw + upper_effort_limits[:,0].tolist()
            w0 = w0 + cart_input_mpc_window[k+1,:].tolist()
            # print(len(w0))
            # print(len(lbw))

            # Get Collocation constraints
            if(k==0):
                eq_k,J_k = Const_and_Cost_w_imp_fun(L_k, # Initial hand state
                                                    L_kp, # Final hand state
                                                    U_k, # Initial hand force input
                                                    U_kp, # Final hand force input
                                                    C_k, # Initial cart pose
                                                    C_kp, # Final cart pose
                                                    C_vel_k, # Initial cart velocity
                                                    C_vel_kp, # Final cart velocity
                                                    Col_imp_0, # Impulse component
                                                    P_k, # Demanded cart pose
                                                    P_0)   # System parameters
                # Append the collocation constraints
                G = eq_k
                self.lbg = self.lbg + np.zeros((num_cart_poses+num_dof+num_dof+1)).tolist()
                self.ubg = self.ubg + np.zeros((num_cart_poses+num_dof+num_dof+1)).tolist()
            else:
                eq_k,J_k = Const_and_Cost_wo_imp_fun(L_k, # Initial hand state
                                                     L_kp, # Final hand state
                                                     U_k, # Initial hand force input
                                                     U_kp, # Final hand force input
                                                     C_k, # Initial cart pose
                                                     C_kp, # Final cart pose
                                                     C_vel_k, # Initial cart velocity
                                                     C_vel_kp, # Final cart velocity
                                                     P_k, # Demanded cart pose
                                                     P_0) # System parameters
                # Append the collocation constraints
                G = vertcat(G,eq_k)
                self.lbg = self.lbg + np.zeros((num_cart_poses+num_dof+num_dof)).tolist()
                self.ubg = self.ubg + np.zeros((num_cart_poses+num_dof+num_dof)).tolist()
            
            # print(len(lbg))

            # Sum up the cost function
            J = J + J_k
            # The initial state of the subsequent time-step 
            L_k = L_kp
            C_k = C_kp
            C_vel_k = C_vel_kp
            # The initial input to the subsequent time-step
            U_k = U_kp
        
        # Solve the NLP
        # Options
        # IPOPT
        opts = {'ipopt.print_level':0, 
                'print_time':0,
                'ipopt.sb':'yes'}
        # FATROP
        # opts = {'structure_detection':'auto',
        #         'debug':True,
        #         'equality': True}

        # Create an NLP solver
        prob = {'f': J, 
                'x': w,
                'g': G, 
                'p': P}
        
        self.solver = nlpsol('solver', 'ipopt', prob, opts)
        # self.solver = nlpsol('solver', 'fatrop', prob, opts)

        sol = self.solver(x0 = w0,
                          lbx = self.lbw, 
                          ubx = self.ubw,
                          lbg = self.lbg, 
                          ubg = self.ubg, 
                          p = p0)
        w_opt = sol['x'].full()
        # print(w_opt[9:-1:self.delta,:])

        # Post Processing
        # Initial Collision impulse
        Col_imp_opt = w_opt[0,0]
        # print(Col_imp_opt)
        # Hand State
        L_opt = np.concatenate((w_opt[1::self.delta,:], # Hand position
                                w_opt[2::self.delta,:]),   # Hand velocity
                               axis=1)
        # print(L_opt)
        # Cart State
        C_opt = np.concatenate((w_opt[3::self.delta,:], # Cart x-position
                                w_opt[4::self.delta,:], # Cart y-position
                                w_opt[5::self.delta,:]),   # Cart orientation
                               axis=1)
        # print(C_opt)
        # Cart velocity State
        C_vel_opt = np.concatenate((w_opt[6::self.delta,:], # Cart linear velocity
                                    w_opt[7::self.delta,:]),   # Cart angular velocity
                                   axis=1)
        # print(C_vel_opt)
        # Input at Collocation point          
        u_opt = np.concatenate((w_opt[8::self.delta,:], # Grip normal-reaction
                                w_opt[9::self.delta,:]), # Lateral-force for moving hand
                               axis=1)

if __name__ == '__main__':
    cart_mpc = Cart_MPC()
    print("Hello World!")