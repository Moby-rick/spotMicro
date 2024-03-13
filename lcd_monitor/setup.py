## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD\
## 어디까지나 테스트용이므로, 이것 말고 CATKIN을 쓰세요.

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['lcd_monitor'],
    package_dir={'': 'src'})

setup(**setup_args)
