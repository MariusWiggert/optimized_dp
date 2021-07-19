import numpy as np
# Utility functions to initialize the problem
from Grid.GridProcessing import Grid
from Shapes.ShapesFunctions import *
# Specify the  file that includes dynamic systems
from dynamics.Air3D import *
# Plot options
from plot_options import *
# Solver core
from solver import HJSolver

import math

""" USER INTERFACES
- Define grid
- Generate initial values for grid using shape functions
- Time length for computations
- Initialize plotting option
- Call HJSolver function
"""


# Scenario 1
g = Grid(np.array([-6., -10., 0.]), np.array([20.0, 10.0, 2.*math.pi]), 3, np.array([51,40,50]), [2])
# gnore_dims (List) : List  specifing axis where cylindar is aligned (0-indexed)
# center (List) :  List specifing the center of cylinder
# radius (float): Radius of cylinder
Initial_value_f = CylinderShape(g, [], np.zeros(3), 5)

# Look-back lenght and time step
lookback_length = 2.8
t_step = 0.2

small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)
my_plane = Air3D(plane_speeds=5)

"""
Assign one of the following strings to `compMethod` to specify the characteristics of computation
"none" -> compute Backward Reachable Set
"minVWithV0" -> compute Backward Reachable Tube
"maxVWithVInit" -> compute max V over time
"minVWithVInit" compute min V over time
"""
po2 = PlotOptions("3d_plot", [0,1,2], [])
# HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
HJSolver(my_plane, g, Initial_value_f, tau, "minVWithV0", po2)