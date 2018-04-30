"""

Path tracking simulation with pure pursuit steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

"""


import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from car_gar import *
import numpy
from matplotlib.patches import Circle
        
    
 


k = 0.07  # look forward gain
Kp = 10.0  # speed proportional gain
dt = 0.1 # [s]
L = 1.6# [m] wheel base of vehicle



show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, delta = 0.0, delta_1 =0, delta_2 = 0, delta_3 = 0, delta_4 = 0, delta_5=0, delta_6 = 0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.delta = delta
        self.delta_1 = delta_1
        self.delta_2 = delta_2
        self.delta_3 = delta_3
        self.delta_4 = delta_4
        self.delta_5 = delta_5
        self.delta_6 = delta_6


def update(state, a, delta):

    if delta > math.pi/5:
        delta = math.pi/5
    if delta < -math.pi/5:
        delta = -math.pi/5

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(state.delta) * dt
    state.v = state.v + a * dt
    state.delta = (0.1*state.delta_1+0.2*state.delta_2+0.5*state.delta_3+0.1*state.delta_4+0.1*state.delta_5+0.00*state.delta_6)
    #state.delta = delta
    state.delta_1 = state.delta_2
    state.delta_2 = state.delta_3
    state.delta_3 = state.delta_4
    state.delta_4 = state.delta_5
    state.delta_5 = state.delta_6
    state.delta_6 = delta
    return state


def PIDControl(target, current): # PID for acceleration
    a = Kp * (target - current)
    print(a)
    return a


def pure_pursuit_control(state, cx, cy, pind, Lfc):

    ind = calc_target_index(state, cx, cy, Lfc)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = k * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return delta, ind


def calc_target_index(state, cx, cy, Lfc):

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d)) # find the closest point in the trajectory
    L = 0.0

    Lf = k * state.v + Lfc# Lf is like the 'maximum'

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind


def generate_semi_random_point(r, curr_goal_dot):
    """
    Semi random dot generator for simulation purposes
    """
    phi_min=-0.3*math.pi
    phi_max=0.3*math.pi
    phi=np.random.rand()*(phi_max-phi_min)-(phi_max+phi_min)/2
    #rip
    goal_x=math.cos(phi)+curr_goal_dot.x+1
    goal_y=math.sin(phi)+curr_goal_dot.y+1
    return [goal_x,goal_y]
    
    
    
    
def pure_pursuit_sim(x_0, y_0, yaw_0, v_0, cx=0, cy=0):
    #  target course
    coordinates_x = []
    coordinates_y = []
    with open("circuit_points.dat") as circuit:
        for line in circuit:
            line_split = line.split(",")
            coordinates_x.append(float(line_split[0]))
            coordinates_y.append(float(line_split[1]))


    point_1 = dots(x=-5, y=0, v=0.1, a=0.0, yaw=math.pi / 4)
    point_2 = dots(x=-4, y=0, v=0.1, a=0.2, yaw=0.8 * math.pi / 4)
    point_3 = dots(x=-3, y=0, v=0.1, a=0.0, yaw=math.pi/4)
    waypoints = [point_1, point_2, point_3]
    # Generate the car and initial trajectory
    x_0 = -10
    y_0 = 0
    yaw_0 = 0
    v_0 = -0.1
    car_1 = car(x=-10, y=y_0, v=v_0, a=0,yaw=0, dist_goal=0.8, dotlist=waypoints)
    # Generate first trajectory
    traj = car_1.traj
    cx = traj.x_ref
    cy = traj.y_ref
    # Initial lookahead
    Lfc = 13.2
    # Initial target speed
    target_speed = 20.0/3.6   # [m/s]

    # initial state
    state = State(x=x_0, y=y_0, yaw=yaw_0, v=v_0)

    di_vec = []
    time = 0.0
    x_vec = []
    y_vec = []
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy, Lfc)
    # Other lists for storing stuff
    provola = 0
    goal_x = 0
    goal_y = 0
    goal_x_vec = [goal_x]
    goal_y_vec = [goal_y]
    delta_vec = []
    rx = []
    a = []
    while time<30:
        print(time)
        point_list = []
        ai = PIDControl(target_speed, state.v) # acceleration command
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind, Lfc)
        di_vec.append(di)
        # update states with the control signals computed above
        state = update(state, ai, di)
        car_1.carstate.update_state(x= state.x, y= state.y, v=state.v, a=0, yaw=state.yaw)
        new_points = car_1.traj.update_start(car_1.carstate, [])
        car_1.move_car(x =cx[target_ind], y=cy[target_ind], v = state.v, yaw = 0, new_points = new_points, point_list = point_list)
        max_radial_acc = 9.8*0.5

        if len(rx)!=0:
            if (rx[target_ind]) != 0:
                old = target_speed
                target_speed = math.sqrt(max_radial_acc / abs(rx[target_ind]))
                #print(target_speed)
                if target_speed > 5:
                    target_speed = old

                Lfc = state.v/0.8*(1-rx[target_ind]*0.5)


        if new_points:
            r = 1

            goal_x = coordinates_x[provola]
            goal_x_vec.append(goal_x)
            goal_y = coordinates_y[provola]
            goal_y_vec.append(goal_y)

            provola = provola +1

            car_1.traj.update_goal(dots(x = goal_x, y = goal_y, v =0, yaw =0))
            car_1.traj.gen_trajectory()
            cx = traj.x_ref
            cy = traj.y_ref
            old_len = len(rx)

            rx = traj.kref
            new_len = len(rx)

            #print('lookahead')
            #print(Lfc)


            target_ind = calc_target_index(state, cx, cy, Lfc)
            di, target_ind = pure_pursuit_control(state, cx, cy, target_ind, Lfc)



        time = time + dt

        x_vec.append(state.x)
        y_vec.append(state.y)
        yaw.append(state.yaw)
        delta_vec.append(state.delta)
        a.append(state.v)
        t.append(time)
        #print(time)
        if show_animation:
            plt.cla()
            fig = plt.figure(1)

            # initialize axis, important: set the aspect ratio to equal
            ax = fig.add_subplot(111, aspect='equal')


#
            plt.plot(cx, cy, ".r")  # , label="course")
            #print(goal_x)
            for x, y in zip(goal_x_vec, goal_y_vec):
                ax.add_artist(Circle(xy=(x, y),
                                     radius=0.5))

            #plt.show()

            plt.plot(state.x, state.y, ".g", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.01)

            #plt.figure(2)
            #plt.plot(delta_vec)
            #lt.cla()
            #plt.plot(time, di)
            #plt.pause(0.01)
        #print('time elapsed')
        #print(T >= time)
        #print('last index')
        #print(lastIndex > target_ind)

    # Test
    #assert lastIndex >= target_ind, "Cannot goal"

    if show_animation:

        #plt.plot(cx, cy, ".r", label="course")
        #plt.plot(x, y, "-b", label="trajectory")
        #plt.legend()
        #plt.xlabel("x[m]")
        #plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        t = t[:-1]
        #print(len(t))
        #print(len(yaw))
        flg, ax = plt.subplots(1)
        plt.plot(x_vec, y_vec, "-r")
        #plt.plot(x_vec,y_vec, "-r")

        plt.xlabel("Time[s]")
        plt.ylabel("Input to steering")
        plt.grid(True)
        plt.show()

        #plt.pause(0.01)
    plt.figure(3)
    #plt.plot(x_vec, y_vec)

    plt.plot(t, [iv for iv in di_vec], "-r")
    plt.show()


