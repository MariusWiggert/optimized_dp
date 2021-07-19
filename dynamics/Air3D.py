import heterocl as hcl

""" Air3D DYNAMICS IMPLEMENTATION 
Details about Air3D can be found in Ian Mitchells PhD Thesis Section 3.1
 x_dot = -v_a + v_b* cos(theta) + a*y
 y_dot = v_a * sin(theta) - a*x
 theta_dot = b-a
 """
class Air3D:
    def __init__(self, plane_speeds, x=[0,0,0], uMin = [-1], uMax = [1], dMin = [-1],
                 dMax=[1], uMode="min", dMode="max"):
        """Creates an Air3D with the following states:
           X position, Y position, relative heading
           The user control is turn_rate, the disturbance is the turn_rate of the other airplane
        Args:
            plane_speed (scalar) : forward velocity of the two planes (assumes they are the same)
            x (list, optional)   : Initial state . Defaults to [0,0,0].
            uMin (list, optional): Lowerbound of user control. Defaults to [-1].
            uMax (list, optional): Upperbound of user control.Defaults to [1].
            dMin (list, optional): Lowerbound of disturbance to user control, . Defaults to [-1].
            dMax (list, optional): Upperbound of disturbance to user control. Defaults to [1].
            uMode (str, optional): Accepts either "min" or "max".
                                   * "min" : have optimal control reach goal
                                   * "max" : have optimal control avoid goal
                                   Defaults to "min".
            dMode (str, optional): Accepts whether "min" or "max" and should be opposite of uMode.
                                   Defaults to "max".
        """
        self.x = x
        self.plane_speeds = plane_speeds
        self.uMax = uMax
        self.uMin = uMin
        self.dMax = dMax
        self.dMin = dMin
        assert(uMode in ["min", "max"])
        self.uMode = uMode
        if uMode == "min":
            assert(dMode == "max")
        else:
            assert(dMode == "min")
        self.dMode = dMode

    def opt_ctrl(self, t, state, spat_deriv):
        """
        :param t: time t
        :param state: tuple of coordinates
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return:
        """

        # Graph takes in 3 possible inputs, by default, for now
        opt_ctrl = hcl.scalar(self.uMax, "opt_ctrl")
        # Just create and pass back, even though they're not used
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")

        # determinand for sign of control
        det = hcl.scalar(0, "det")
        det[0] = spat_deriv[0]*state[1] - spat_deriv[1]*state[0] - spat_deriv[2]

        if self.uMode == "min":
            with hcl.if_(det >= 0):
                opt_ctrl[0] = self.uMin[0]
            with hcl.elif_(det < 0):
                opt_ctrl[0] = self.uMax[0]
        else:
            with hcl.if_(det >= 0):
                opt_ctrl[0] = self.uMax[0]
            with hcl.elif_(det < 0):
                opt_ctrl[0] = self.uMin[0]

        return (opt_ctrl[0], in3[0], in4[0])

    def optDstb(self, spat_deriv):
        """
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return: a tuple of optimal disturbances
        """
        # Graph takes in 3 possible inputs, by default, for now
        opt_dist = hcl.scalar(self.dMax, "opt_dist")
        # Just create and pass back, even though they're not used
        in3 = hcl.scalar(0, "in3")
        in4 = hcl.scalar(0, "in4")

        if self.uMode == "min":
            with hcl.if_(spat_deriv[2] >= 0):
                opt_dist[0] = self.dMin[0]
            with hcl.elif_(spat_deriv[2] < 0):
                opt_dist[0] = self.dMax[0]
        else:
            with hcl.if_(spat_deriv[2] >= 0):
                opt_dist[0] = self.dMax[0]
            with hcl.elif_(spat_deriv[2] < 0):
                opt_dist[0] = self.dMin[0]

        return (opt_dist[0], in3[0], in4[0])

    def dynamics(self, t, state, uOpt, dOpt):
        # x_dot = -v_a + v_b * cos(theta) + a * y
        # y_dot = v_a * sin(theta) - a * x
        # theta_dot = b - a
        x_dot = hcl.scalar(0, "x_dot")
        y_dot = hcl.scalar(0, "y_dot")
        theta_dot = hcl.scalar(0, "theta_dot")

        x_dot[0] = -self.plane_speeds + self.plane_speeds * hcl.cos(state[2]) + uOpt*state[1]
        y_dot[0] = self.plane_speeds * hcl.sin(state[2]) - uOpt*state[0]
        theta_dot[0] = dOpt - uOpt

        return (x_dot[0], y_dot[0] ,theta_dot[0])