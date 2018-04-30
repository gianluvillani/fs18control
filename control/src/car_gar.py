import math
from scipy.interpolate import splev, splrep
import quintic_polynomials_planner as qp
from pycubicspline import *



class car:
    def __init__(self, x, y, v, a, yaw, dotlist, dist_goal, point_list = []):
        self.carstate = dots(x, y, v, a, yaw)
        self.traj = trajectory(dotlist, dist_goal)
        self.traj.update_start(self.carstate, point_list)
        self.traj.gen_trajectory()

    def move_car(self, x, y, v, yaw, new_points, point_list =[]):
        self.x = x
        self.y = y
        self.v = v
        self.yaw = yaw
        self.traj.update_start(self.carstate, [])  # updated only when the car is close enough to the closest waypoint
        if new_points:
            self.traj.gen_trajectory()  # recompute the trajectory to the actual state

    def reached_goal():
        return traj.reached_final_goal()

    def add_goal(next_goal):
        self.trajectory.update_goal(next_goal)


class dots:
    def __init__(self, x, y, v, yaw, a=0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        # self.a = a
        self.vx = v * math.cos(yaw)
        self.vy = v * math.sin(yaw)
        # self.ax = a*math.cos(yaw)
        # self.ay = a*math.sin(yaw)

    def update_state(self, x, y, v, a, yaw):
        """
        Update coordinates for the car
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.a = a
        self.vx = v * math.cos(yaw)
        self.vy = v * math.sin(yaw)
        self.ax = a * math.cos(yaw)
        self.ay = a * math.sin(yaw)


class trajectory:
    def __init__(self, dotlist, close_dist):
        self.dotlist = dotlist  # Dots to be trajectorized
        self.dist = close_dist  # Maximum distance
        # between the car state and the closest waypoint

    def update_goal(self, dot_goal):
        """
        Called each time a new goal point is added
        """
        self.dotlist.append(dot_goal)

    def close_enough(self, car_state, curr_dot):
        """
        Check if car has passed a dot in the trajectory
        """
        #print(curr_dot.x)
        #print(curr_dot.y)
        #print(car_state.x)
        #print(car_state.y)
        dist = math.sqrt((curr_dot.x - car_state.x) ** 2 + (curr_dot.y - car_state.y) ** 2)
        return dist < self.dist

    def update_start(self, car_state, point_list):

        # NEW
        # Recomputing close. keep continuity in some way
        #self.dotlist[0] = car_state

        # NEW

        if self.close_enough(car_state, self.dotlist[1]):
            # for i in self.dotlist:
            # print(i.x)
            # print(i.y)
            # print('ja')

            temp = self.dotlist[1]
            del self.dotlist[1]
            self.dotlist[0] = temp
            #point_list = []
            ''''
            if len(point_list)!=0:
                for i in len(point_list):
                    self.dotlist.insert(1, dots(x=point_list[i+1][0],
                                            y=point_list[i+1][1], v=0, yaw = 0) )
                self.dotlist.insert(1, dots(x=point_list[0][0],
                                            y=point_list[0][1], v=0, yaw = 0))
            '''




            return True

        return False


    def gen_trajectory(self):
        """
        Trajectory generator
        """
        # do stuff here
        # x_trajectory = linspace(self.start, self.)
        dt = 0.1
        max_accel = 1
        self.x_ref = []
        self.y_ref = []
        self.v_ref = []
        self.a_ref = []
        self.yaw_ref = []
        mode = 1
        if mode == 0:
            for i in range(len(self.dotlist) - 1):
                curr_dot = self.dotlist[i]

                goal_dot = self.dotlist[i + 1]
                #print(goal_dot.x)
                # Get the point coordinate, the first one is the car position

                [time, rx, ry, ryaw, rv, ra] = qp.quinic_polynomials_planner(sx=curr_dot.x, sy=curr_dot.y,
                                                                             sv=curr_dot.v,
                                                                             syaw=curr_dot.yaw, sa=curr_dot.a,
                                                                             gx=goal_dot.x,
                                                                             gy=goal_dot.y, gv=goal_dot.v,
                                                                             ga=goal_dot.a,
                                                                             gyaw=goal_dot.yaw, dt=dt,
                                                                             max_accel=max_accel)

                self.x_ref.extend(rx)
                self.y_ref.extend(ry)
                self.v_ref.extend(rv)
                self.a_ref.extend(ra)
                self.yaw_ref.extend(ryaw)
        if mode == 1:
            x = []
            y = []
            #print(self.dotlist[0].x)
            #print(self.dotlist[0].y)
            for i in range(len(self.dotlist)):
                x.append(self.dotlist[i].x)
                y.append(self.dotlist[i].y)

            sp = Spline2D(x, y)
            s = np.arange(0, sp.s[-1], 0.1)
            rx, ry, ryaw, rk = [], [], [], []
            for i_s in s:
                ix, iy = sp.calc_position(i_s)
                rx.append(ix)
                ry.append(iy)
                ryaw.append(sp.calc_yaw(i_s))
                rk.append(sp.calc_curvature(i_s))

            self.x_ref = rx
            self.y_ref = ry
            self.yaw_ref = ryaw
            self.kref = rk

    def get_next_traj(self):
        return dots(x_vec[0], y_vec[0], 0)

    def reached_final_goal(self):
        return close_enough(dotlist[0], dotlist[-1])

    def return_goal(self):
        return self.dotlist[-1]
