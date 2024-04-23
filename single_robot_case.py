import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from exercise_code import compute_controller, update_robot_state
from visualize_mobile_robot import sim_mobile_robot

class simulate_single_robot():
    def __init__(self):
        # SIMULATION PARAMETER
        t_max = 40. # total simulation time,
        self.Ts = 0.02 # time sampling
        self.sim_iter = round(t_max/self.Ts) # Total Step for simulation

        # INITIAL POSITION AND GOAL
        self.robot_state = np.array([-1.5, -1.5, 0]) # numpy array for [px, py, theta]
        self.desired_state = np.array([1.5, 1.5, 0.]) # numpy array for goal / the desired [px, py, theta]
        # array to store trajectory
        self.state_history = np.zeros( (self.sim_iter, len(self.robot_state)) ) 

        # Static obstacles
        self.obstacles = {
            0: {'pos': np.array([0., 0.1]), 'rad': 0.7}
        }

        # Visualizer
        # self.sim_visualizer = sim_mobile_robot( 'omnidirectional' ) # Omnidirectional Icon
        self.sim_visualizer = sim_mobile_robot( 'unicycle' ) # Unicycle Icon
        self.sim_visualizer.set_field( (-2.5, 2.5), (-2, 2) ) # set plot area
        self.sim_visualizer.show_goal(self.desired_state)
        # Plot obstacles
        for key in self.obstacles:
            obst_pos, obst_rad = self.obstacles[key]['pos'], self.obstacles[key]['rad']
            self.sim_visualizer.ax.add_patch( plt.Circle( (obst_pos[0], obst_pos[1]), obst_rad, color='r' ) )
            self.sim_visualizer.ax.add_patch( plt.Circle( (obst_pos[0], obst_pos[1]), obst_rad+0.2, color='r', fill=False) )
        
        self.pl_ps, = self.sim_visualizer.ax.plot(0, 0, 'r.', marker='x')


    def loop(self, it = 0):
        current_time = it*self.Ts
        if (it > 0) and (it % round(1 / self.Ts) == 0): print('simulating t = {}s.'.format(current_time))
        if it >= self.sim_iter: exit() # Exit at the end of simulation time

        self.state_history[it] = self.robot_state

        # COMPUTE CONTROL INPUT
        #------------------------------------------------------------  
        goal_pos = self.desired_state[:2]
        vx, vy, state = compute_controller(goal_pos, self.robot_state, self.obstacles)
        #------------------------------------------------------------

        # update visualization data
        self.sim_visualizer.update_trajectory( self.state_history[:it+1] ) # up to the latest data
        self.pl_ps.set_data(state[0], state[1])


        # UPDATE NEW STATE FOR NEXT ITERATION
        #------------------------------------------------------------        
        self.robot_state = update_robot_state(self.robot_state, self.Ts, vx, vy)
        #------------------------------------------------------------        

        # update visualization data
        self.sim_visualizer.update_visualization( current_time )


if __name__ == '__main__':

    sim = simulate_single_robot() # Run the single robot case
    # sim = simulate_multi_robot() # Run the multi robot case

    ani = animation.FuncAnimation( sim.sim_visualizer.fig, sim.loop, save_count=sim.sim_iter, interval = sim.Ts*1000)

    IS_SAVE_INTO_ANIMATION = False
    if IS_SAVE_INTO_ANIMATION:
        print('saving animation ...')
        ani.save('simulation.gif', fps=round(0.5/sim.Ts)) # Half speed simulation
    else:
        plt.show()

