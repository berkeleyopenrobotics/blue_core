#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages = ['blue_bringup'],
    package_dir = {'': 'scripts'},
    install_requires = ['']
)

setup(**setup_args)
