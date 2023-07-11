from setuptools import setup
import os
from glob import glob

package_name = 'unit5_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='somebody very awesome',
    maintainer_email='user@user.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exercise51 = unit5_pkg.exercise51:main',
            'exercise52 = unit5_pkg.exercise52:main',
            'exercise53 = unit5_pkg.exercise53:main',
            'callback_groups_examples = unit5_pkg.callback_groups_examples:main',
        ],
    },
)