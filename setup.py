from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pc2image'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files if you have any
        (os.path.join('share', package_name, 'launch'),
            glob('launch/launch_*.py')),
        # Include config files if you have any
        #(os.path.join('share', package_name, 'config'),
        #    glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'scipy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Kevin Wu',
    maintainer_email='wu.kevi@northeastern.edu',
    description='Package for projecting LiDAR point clouds onto RGB images',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'launch_projector = pc2image.PointCloudProjector:main',
        ],
    },
)
