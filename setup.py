from setuptools import find_packages, setup

package_name = 'myrobot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
 data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/myrobot_controller']),
    ('share/myrobot_controller', ['package.xml']),
    ('share/myrobot_controller/launch',
        ['launch/my_robot.launch.py']),
    ('share/myrobot_controller/urdf',
        ['urdf/myrobot.urdf']),
],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vaibhav',
    maintainer_email='vaibhav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
