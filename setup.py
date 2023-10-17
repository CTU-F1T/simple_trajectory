#!/usr/bin/env python
# setup.py
"""Install script for ROS1 catkin / ROS2 ament_python."""

from setuptools import setup


package_name = 'simple_trajectory'


setup(
 name=package_name,
 version='0.5.1',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=[
    'setuptools',
    'numpy',
    'scipy',
 ],
 zip_safe=True,
 maintainer='F1tenth CTU Community',
 maintainer_email='f1tenth@rtime.felk.cvut.cz',
 description='This node is used to create a custom path using "Publish Point" feature that is available in rViz.',
 license='GPLv3',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'run = simple_trajectory.node:main'
     ],
   },
)


# Should do the same work for ROS1 as:
#from distutils.core import setup
#from catkin_pkg.python_setup import generate_distutils_setup

#d = generate_distutils_setup(
#    packages=[package_name],
#    package_dir={"": "module"}
#)

#setup(**d)
