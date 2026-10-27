from glob import glob

from setuptools import setup

setup(
    name='control_test',
    version='0.1.0',
    packages=['control_test'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/control_test']),
        ('share/control_test', ['package.xml']),
        ('share/control_test/launch', glob('launch/*.py')),
        ('share/control_test/config', glob('config/*.yaml')),
        ('share/control_test/config/sine', glob('config/sine/*.yaml')),
        ('share/control_test/config/grip', glob('config/grip/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'set_center = control_test.set_center_node:main',
        'sin_test = control_test.sin_test_node:main',
        'profile_test = control_test.profile_test_node:main',
        'grip_test = control_test.grip_test_node:main',
    ]},
)
