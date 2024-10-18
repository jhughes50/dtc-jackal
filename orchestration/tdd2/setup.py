from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tdd2'],
    scripts=['scripts/detection_and_gps.py'],
    package_dir={'': 'src'}
)

setup(**d)
