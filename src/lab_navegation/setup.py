from setuptools import find_packages, setup

package_name = 'lab_navegation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fbr',
    maintainer_email='felipebr@uc.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dead_reckoning = lab_navegation.ros2_dead_reckoning:main',
            'pose_loader = lab_navegation.ros2_pose_loader:main',
        ],
    },
)