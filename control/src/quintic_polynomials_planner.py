"""

Quinitc Polynomials Planner

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Local Path Planning And Motion Control For Agv In Positioning](http://ieeexplore.ieee.org/document/637936/)

"""

import numpy as np
import matplotlib.pyplot as plt
import math

# parameter
MAX_T = 100.0
MIN_T = 5.0

show_animation = True


class quinic_polynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, T):
        # xs = x_start
        # vxs = vx_start
        # axs = ax_start
        # xe = x_end
        # vxe = vx_end
        # axe = ax_end
        # T = 'period', final time instant




        # calc coefficient of quinic polynomial
        self.xs = xs
        self.vxs = vxs
        self.axs = axs
        self.xe = xe
        self.vxe = vxe
        self.axe = axe

        self.a0 = xs   # a_o = starting x
        self.a1 = vxs  # a_1 = starting velocity
        self.a2 = axs / 2.0 # a_2 = starting acceleration divided by two

        # For this part, look at Elisa's thesis:

        # Anyway, ok it should be easy! Just find all the constraints.
        A = np.array([[T**3, T**4, T**5],
                      [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                      [6 * T, 12 * T ** 2, 20 * T ** 3]])
        b = np.array([xe - self.a0 - self.a1 * T - self.a2 * T**2,
                      vxe - self.a1 - 2 * self.a2 * T,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]
        #̈́ Ok, we computed the



    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t**2 + \
            self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5
        # Calculate the reference and the given instant of time t

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
            3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4
        # Calculate first derivative


        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3
        # Calculate second derivative

        return xt


def quinic_polynomials_planner(sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, dt):
    """
    quinic polynomial planner

    input
        sx: start x position [m]
        sy: start y position [m]
        syaw: start yaw angle [rad]
        sa: start accel [m/ss]
        gx: goal x position [m]
        gy: goal y position [m]
        gyaw: goal yaw angle [rad]
        ga: goal accel [m/ss]
        max_accel: maximum accel [m/ss]
        dt: time tick [s] - it is the time step

    return
        time: time result
        rx: x position result list
        ry: y position result list
        ryaw: yaw angle result list
        rv: velocity result list
        ra: accel result list

    """

    # Here we trasform the initial conditions
    vxs = sv * math.cos(syaw)   # In our case, maybe, we should consider the yaw angle
    vys = sv * math.sin(syaw)           # The starting yaw gives the starting orientation
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)   # The final yaw gives the final orientation
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    for T in np.arange(MIN_T, MAX_T, MIN_T):
        # Define polynomial in x
        xqp = quinic_polynomial(sx, vxs, axs, gx, vxg, axg, T)
        # Define polynomial in t
        yqp = quinic_polynomial(sy, vys, ays, gy, vyg, ayg, T)

        time, rx, ry, ryaw, rv, ra = [], [], [], [], [], []

        for t in np.arange(0.0, T + dt, dt):    # iterate for each dt
            time.append(t)
            rx.append(xqp.calc_point(t))        # calculate next step along x
            ry.append(yqp.calc_point(t))        # calculate next step along y

            vx = xqp.calc_first_derivative(t)   # calculate velocity along x
            vy = yqp.calc_first_derivative(t)   # calculate velocity along y
            v = np.hypot(vx, vy)                # calculate speed (mod velocity)
            yaw = math.atan2(vy, vx)            # compute yaw
            rv.append(v)                        # append to rv
            ryaw.append(yaw)

            ax = xqp.calc_second_derivative(t)  # Compute acceleration.
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
                pass
            ra.append(a)

        if max([abs(i) for i in ra]) <= max_accel:  # Just have to understand this! How does it ensure maximum acceleration?
            print("find path!!")
            break

    if show_animation:
        for i in range(len(rx)):
            plt.cla()
            plt.axis("equal")
            plot_arrow(sx, sy, syaw)
            plot_arrow(gx, gy, gyaw)
            plot_arrow(rx[i], ry[i], ryaw[i])
            plt.title("Time[s]:" + str(time[i])[0:4] +
                      " v[m/s]:" + str(rv[i])[0:4] +
                      " a[m/ss]:" + str(ra[i])[0:4])
            plt.pause(0.01)

    return time, rx, ry, ryaw, rv, ra


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    u"""
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    print(__file__ + " start!!")

    sx = 0.0  # start x position [m]
    sy = 0.0  # start y position [m]
    syaw = math.radians(math.pi/4)  # start yaw angle [rad]
    sv = 1.0  # start speed [m/s]
    sa = 0.1  # start accel [m/ss]
    gx = 30.0  # goal x position [m]
    gy = -10.0  # goal y position [m]
    gyaw = math.radians(0.0)  # goal yaw angle [rad]
    gv = 1.0  # goal speed [m/s]
    ga = 0.1  # goal accel [m/ss]
    max_accel = 10  # max accel [m/ss]
    dt = 0.1  # time tick [s]

    time, x, y, yaw, v, a = quinic_polynomials_planner(
        sx, sy, syaw, sv, sa, gx, gy, gyaw, gv, ga, max_accel, dt)
    # Baiscally, here we obtan the referecne signals for:
    # x = x position
    # y = y position
    # v = speed
    # yaw = yaw
    # a = acceleration
    # Thisc


    if show_animation:
        plt.plot(x, y, "-r")

        plt.subplots()
        plt.plot(time, [math.degrees(i) for i in yaw], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Yaw[deg]")
        plt.grid(True)

        plt.subplots()
        plt.plot(time, v, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[m/s]")
        plt.grid(True)

        plt.subplots()
        plt.plot(time, a, "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("accel[m/ss]")
        plt.grid(True)

        plt.show()


if __name__ == '__main__':
    main()
