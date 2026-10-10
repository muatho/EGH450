#!/usr/bin/env python3

"""
Catkin packaging configuration for the spar_node Python modules.

Catkin reads this file when building the workspace. It makes the Python code
under src/spar_node importable by executable ROS scripts such as
scripts/autonomous_mission.

Do not run this file directly. Build the workspace through catkin instead.
"""

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup


# Describe the Python package using metadata already declared in package.xml.
# `package_dir` states that the importable `spar_node` package is located
# beneath this ROS package's src directory.
setup_args = generate_distutils_setup(
    packages=["spar_node"],
    package_dir={"": "src"},
)

setup(**setup_args)