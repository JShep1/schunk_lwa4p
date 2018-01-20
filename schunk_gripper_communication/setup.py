from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    #scripts=['bin/set_gripper'],
    packages=['schunk_gripper_communication'],
    package_dir={'': 'src'}
)

setup(**setup_args)
