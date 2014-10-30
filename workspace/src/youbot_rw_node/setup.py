#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['youbot_rw_node'],
    package_dir={'': 'scripts'}
    ##  don't do this unless you want a globally visible script
    # scripts=['bin/myscript'],
    # requires=[]
)

setup(**d)