from __future__ import absolute_import
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

distutils_config = generate_distutils_setup(
    packages=['sr_grasp'],
    scripts=[],
    package_dir={'': 'src'}
)

setup(** distutils_config)
