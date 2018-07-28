## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    scripts=['scripts/fixed_wing_controller.py',\
    'scripts/quadrotor_controller.py',\
    'scripts/keyboard_controller.py',\
    'scripts/uav_controller.py'],
    packages=['sample_team'],
    package_dir={'': 'src'})

setup(**setup_args)
