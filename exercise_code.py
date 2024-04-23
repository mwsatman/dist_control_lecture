import numpy as np
from qpsolvers import Problem, solve_problem

def compute_controller(goal_pos, robot_state, obstacles):

    state = robot_state[:2].copy() # only use the position information

    # CALCULATE POINT AHEAD OF THE ROBOT
    # TODO
    # ell = 0.1
    # theta = robot_state[2]
    # state[0] = ???
    # state[1] = ???

    # COMPUTE GO-TO-GOAL CONTROLLER (nominal)
    kP = 1.
    u_nom = kP*(goal_pos - state)

    # Saturate nominal input if needed
    # TODO
    # u_max = 0.15

    # initiate default solution (to stop)
    vx_star, vy_star = 0., 0.

    obst_num = 0 # skip the optimization in the template
    # obst_num = len(obstacles)
    if obst_num > 0: 
        # APPLY QP-BASED COLLISION AVOIODANCE 
        P, q = 2 * np.eye(2), -2*u_nom
        H = np.zeros([obst_num, 2])
        b = np.zeros([obst_num, 1])

        i = 0
        for key in obstacles:
            gamma, pow = 10., 1

            obst_i = obstacles[key]['pos']
            obst_rad = obstacles[key]['rad']

            # TODO
            # H[i] = ???
            # b[i] = ???

            i += 1

        # Provide bounds for the solution (if needed)
        # TODO
        # u_max = np.inf
        # lb = np.ones(2)*(-u_max)
        # ub = np.ones(2)*(u_max)

        # solve the qp problem
        qp_problem = Problem(P, q, H, b) # solving without box bounds
        # qp_problem = Problem(P, q, H, b, lb = lb, ub = ub) # solving with box bounds

        opt_tolerance = 1e-8
        solution = solve_problem(qp_problem, solver="daqp", 
            dual_tol=opt_tolerance, primal_tol=opt_tolerance)
        sol = solution.x

        # check the solution
        if sol is None:
            print('WARNING QP SOLVER [no solution] stopping instead')
        else:
            if not solution.is_optimal(opt_tolerance):
                print('WARNING QP SOLVER [not optimal] stopping instead')
            else:
                vx_star, vy_star = sol[0], sol[1]
    
    else: vx_star, vy_star = u_nom[0], u_nom[1]

    # return safe control input
    return vx_star, vy_star, state



def update_robot_state(robot_state, Ts, vx, vy):

    prev_state = robot_state.copy()

    # using discrete-time model of SINGLE INTEGRATOR dynamics for omnidirectional robot
    # ----------------------------------------------------------------------------------
    current_input = np.array([vx, vy, 0])
    # Update new state of the robot at time-step t+1
    robot_state = prev_state + Ts*current_input # will be used in the next iteration
    robot_state[2] = ( (robot_state[2] + np.pi) % (2*np.pi) ) - np.pi # ensure theta within [-pi pi]

    # # using discrete-time model of UNICYCLE model
    # # ----------------------------------------------------------------------------------
    # ell = 0.1
    # theta = prev_state[2]
    # # Do conversion from vx vy to V and omega (based on point ahead strategy)
    # # TODO
    # # current_input = ???

    # # Check / Apply saturation limit
    # v_lin_max, omega_max = 0.22, 2.84
    # # TODO
    # # current_input[0] = ???
    # # current_input[1] = ???

    # # Update new state of the robot at time-step t+1
    # B = np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0, 1]])
    # robot_state = prev_state + Ts*(B @ current_input) # will be used in the next iteration
    # robot_state[2] = ( (robot_state[2] + np.pi) % (2*np.pi) ) - np.pi # ensure theta within [-pi pi]

    return robot_state

