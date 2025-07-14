import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'control_motor', 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_test_dual = control_motor.pid_test_dual:main',
            'motores_semPID = control_motor.motores_semPID:main',
            'mapear_velocidade = control_motor.mapear_velocidade:main',
            'simple_dual_motor_control = control_motor.simple_dual_motor_control:main',
            'pid_motor_control = control_motor.pid_motor_control:main',
            'encoder_publisher = control_motor.encoder_publisher:main',
            'velocity_computation = control_motor.velocity_computation:main',
        ], 
    },
)
