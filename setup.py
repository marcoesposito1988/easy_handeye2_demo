import os
from glob import glob

from setuptools import setup

package_name = 'easy_handeye2_demo'

setup(
 name=package_name,
 version='0.5.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (
         os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*')))
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Marco Esposito',
 maintainer_email='esposito@imfusion.com',
 description='Playground for a hand-eye calibration with easy_handeye2, no hardware required.',
 license='BSD',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'tracking_simulator = easy_handeye2_demo.tracking_simulator:main'
     ],
   },
)