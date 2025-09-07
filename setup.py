from setuptools import setup, find_packages

package_name = 'bldc_car_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', ['launch/sim_car.launch.py']),
        ('share/' + package_name + '/models/bldc_car', [
            'models/bldc_car/model.sdf', 'models/bldc_car/model.config'
        ]),
        ('share/' + package_name + '/models/worlds', [
            'models/worlds/flat_world.sdf'
        ]),
        ('share/' + package_name + '/params', ['bldc_car_demo/params/car.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    description='4-wheel car with rear BLDC motors controlled via voltage from /cmd_vel',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_voltage = bldc_car_demo.nodes.cmd_vel_to_voltage:main',
            'state_bridge = bldc_car_demo.nodes.state_bridge:main',
        ],
    },
)
