from glob import glob

from setuptools import setup

setup(
    name='sine_test',
    version='0.1.0',
    packages=['sine_test'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/sine_test']),
        ('share/sine_test', ['package.xml']),
        ('share/sine_test/launch', glob('launch/*.py')),
        ('share/sine_test/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'set_center = sine_test.set_center_node:main',
        'sin_test = sine_test.sin_test_node:main',
    ]},
)
