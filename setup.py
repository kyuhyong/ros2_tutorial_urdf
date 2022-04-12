# Added imports
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

from setuptools import setup

package_name = 'ros2_tutorial_urdf'

setup(
  name=package_name,
  version='0.0.0',
  packages=[package_name],
  data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # Added
    (os.path.join('share', package_name), glob('launch/*.py')),
    (os.path.join('share', package_name), glob('urdf/*'))
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='kyu2',
  maintainer_email='kyuhyong@gmail.com',
  description='TODO: Package description',
  license='TODO: License declaration',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      #to run ‘state_publisher’ from a console
      'r2d2_state_publisher = ros2_tutorial_urdf.r2d2_state_publisher:main',
      'diff2_state_publisher = ros2_tutorial_urdf.diff2_state_publisher:main'
    ],
  },
)
