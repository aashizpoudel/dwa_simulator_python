import math
import numpy as np
import matplotlib.pyplot as plt
import asyncio
import random


def toDegree(radian):
    return math.degrees(radian)

class Robot:
    def __init__(self, x, y, theta, v_max, w_max, dt):
        self.x = x 
        self.y = y
        self.v = 0 
        self.w = 0
        self.theta = theta
        self.v_max = v_max
        self.w_max = w_max
        self.a_max = 5.0
        self.alpha_max = 3.2
        self.v_samples = 15
        self.w_samples = 10
        self.dt = dt
        self.time = 0
        self.stopped = True
        self.parameters = [1,2,1,0.1] # weights of [distance_cost, obstacle_cost, heading cost, velocity cost ]

    def motion_model(self,state,input,dt): #The model of robot motion.
        x,y,theta,v,w = state
        u1,u2 = input 
        x_new  = x + u1 * math.cos(theta) * dt
        y_new = y + u1 * math.sin(theta) * dt
        theta_new = theta + u2*dt 
        v = u1 
        w = u2 
        return [x_new,y_new,theta_new,v,w]

    
    def generate_trajectory(self,state,input,time): # What will happen to robot after particular time with given input?
        for i in np.arange(0,time,self.dt):
            state = self.motion_model(state,input,self.dt)
        return state  
        

    def calculate_obstacle_cost(self, state, obstacles, radius):
        x,y,_ ,_,_= state 
        cost = 0
        for obstacle in obstacles:
            dist = math.sqrt((x - obstacle[0])**2 + (y - obstacle[1])**2)
            if dist < radius:
                cost += 1.0 / dist
        return cost

    def heading_cost(self,state, goal):
        x,y,theta,_,_ = state
        theta=toDegree(theta)
        if theta < 0:
            theta = theta + 360

        goalTheta=toDegree(math.atan2(goal[1]-y,goal[0]-x))#Goal direction
        if goalTheta<0:
            goalTheta = goalTheta + 360

        targetTheta = goalTheta - theta 
        cost = abs(min(targetTheta, 360 - targetTheta))

        return cost 
    

    def dynamic_window_approach(self, goal, obstacles, radius): #Algorithm implementation 
        vs = np.array([-self.v_max, self.v_max, -self.w_max, self.w_max  ])
        vd = np.array([self.v - self.a_max*self.dt, self.v + self.a_max*self.dt, 
                       self.w - self.alpha_max*self.dt,self.w + self.alpha_max*self.dt])
        temp = np.vstack([vs,vd])
        vr = [temp[:,0].max(),temp[:,1].min(), temp[:,2].max(),temp[:,3].min()]

        v_opt, w_opt = 0.0, 0.0
        v_samples = np.linspace(vr[0],vr[1],self.v_samples)
        w_samples = np.linspace(vr[2],vr[3],self.w_samples)
        state = [self.x,self.y,self.theta,self.v,self.w]
        evaluations = np.empty(0).reshape(0,6)
        for v in v_samples:
            for w in w_samples:
                new_state = self.generate_trajectory(state,[v,w],2)
                x_new,y_new,_,_,_ = new_state
                distance_cost = math.sqrt((x_new - goal[0])**2 + (y_new - goal[1])**2)
                obstacle_cost = self.calculate_obstacle_cost(new_state, obstacles, radius)
                heading_cost = self.heading_cost(new_state,goal)
                velocity_cost = self.v_max - v
                evaluations = np.vstack([evaluations,[v,w,distance_cost,obstacle_cost,heading_cost,velocity_cost]])
        for i in range(2,6):
            evaluations[:,i] = (evaluations[:,i] - evaluations[:,i].min()) / (evaluations[:,i].max() - evaluations[:,i].min() + 1e-6)
        cost = (evaluations[:,2:6]*np.array(self.parameters)).sum(axis=1)
        optimum = cost.argmin()
        v_opt,w_opt = evaluations[optimum,0], evaluations[optimum,1]
        return v_opt, w_opt
    

    def step(self,input,dt):
        state = [self.x,self.y,self.theta,self.v,self.w]
        new_state = self.motion_model(state,input,dt)
        self.x,self.y,self.theta,self.v,self.w = new_state 
        self.time += dt
        return


    async def follow_waypoints(self, waypoints, obstacles,radius):
        print("Starting robot")
        self.stopped = False
        self.trajectory = [[self.x,self.y,self.theta,self.v,self.w,self.time]] 
        for goal in waypoints:
            start_time = self.time 
            distance = 0
            start_point = [self.x,self.y]
            while True:
                # print("Working")
                # print(goal)
                v,w = self.dynamic_window_approach(goal,obstacles,radius)
                self.step([v,w],self.dt)
                x,y,theta,v,w,time = self.x,self.y,self.theta,self.v,self.w,self.time
                self.trajectory.append([x,y,theta,v,w,time])
                distance += math.sqrt( (start_point[0] - x)**2 + (start_point[1] - y)**2 )
                start_point = [x,y]
                if math.sqrt((goal[0] - x)**2 + (goal[1] - y)**2) < 0.5:
                    time_taken = self.time - start_time
                    
                    print("Goal reached:",goal, f"distance travelled: {distance:.2f}m ,time taken:{time_taken:.2f}s ")
                    break 
                await asyncio.sleep(self.dt)
        self.stopped = True
    
   
    async def get_trajectory(self):
        return self.trajectory


# Plotter
async def plot_trajectory(robot,line,quiver,text):
    while True:
        if robot.stopped:
            plt.text(10,4,f"Total time: {robot.time:.2f} s")
            plt.draw()
            plt.show()
            return
        robot_trajectory = await robot.get_trajectory()
        robot_trajectory = np.array(robot_trajectory)
        line.set_data(robot_trajectory[:,0], robot_trajectory[:,1])
        quiver.set_offsets([robot.x,robot.y])
        quiver.set_UVC(1 * np.cos(robot.theta), 1 * np.sin(robot.theta))
        text.set_text(f"v,w = {robot.v:.2f},{robot.w:.2f}")
        line.figure.canvas.draw()
        quiver.figure.canvas.flush_events()
        await asyncio.sleep(0.01)


def load_waypoints(filename):
    coordinates = []
    with open(filename,"r") as fp:
        lines = fp.readlines()
        coordinates = [ [int(l) for l in line.split(',')] for line in lines ]
    return np.array(coordinates)


async def main():
    num_obstacles = 20
    robot = Robot(0, 0, 0, 1.0, 1.0, 0.1)
    waypoints  = load_waypoints('waypoints.txt')[:2,:]
    obstacles = np.array([(random.uniform(0,30), random.uniform(0,30)) for _ in range(num_obstacles)])

    radius = 1.0
    _, ax = plt.subplots(figsize=(8,5))
  
   
    ax.set_title("Autonomous Robot Simulator")
    ax.plot(waypoints[:,0], waypoints[:,1], 'kx', markersize=5,label="Goals")  # Plot goal

    ax.plot(obstacles[:,0], obstacles[:,1], 'ro', markersize=5,label='obstacles')  # Plot obstacles
    trajectory, = ax.plot([],[],'b-',label='Trajectory')
    robot_arrow = ax.quiver(robot.x, robot.y, 0.5 * math.cos(robot.theta), 0.5 * math.sin(robot.theta),
                    angles='xy', scale_units='xy', scale=1,label='Robot')
    text = plt.text(-5,-5,"v,w")
   
    plt.legend(loc='upper right')
    plt.ioff()
    plt.show(block=False)
    await asyncio.gather(robot.follow_waypoints(waypoints,obstacles,radius),
                         plot_trajectory(robot,trajectory,robot_arrow,text),
                         )

    


if __name__=='__main__':
    asyncio.run(main())