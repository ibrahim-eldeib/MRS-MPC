from setuptools import setup

package_name = 'turtlebot_dual_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim',
    maintainer_email='hiiimaazz@gmail.com',
    description='ROS 2 package to control two TurtleBots simultaneously.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_controller = turtlebot_dual_controller.dual_controller:main',
            'dual_mpc = turtlebot_dual_controller.dual_mpc:main',
            'dual_mpc_EKF = turtlebot_dual_controller.dual_mpc_EKF:main',
            'dual_mpc_V1 = turtlebot_dual_controller.dual_mpc_V1:main',
            'dual_mpc_V2 = turtlebot_dual_controller.dual_mpc_V2:main',
            'dual_mpc_V3 = turtlebot_dual_controller.dual_mpc_V3:main',
            'single_mpc = turtlebot_dual_controller.Single_MPC:main',
            'acados = turtlebot_dual_controller.ACADOS:main',
        ],
    },
)
