#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['control_node','hand_controller','robot_controller'],
   package_dir={'':'src'},
)

setup(**d)