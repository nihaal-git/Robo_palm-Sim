from setuptools import setup
import os
from glob import glob

package_name = 'Ust_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'admittance_controller = Ust_description.admittance_controller:main',
            'test_data_sender = Ust_description.test_data_sender:main' ,
            'synthetic_emg_publisher = Ust_description.synthetic_emg_publisher:main',
            'synthetic_fsr_publisher = Ust_description.synthetic_fsr_publisher:main',
            'force_assist_controller = Ust_description.force_assist_controller:main',
            'joint_plotter = Ust_description.joint_plotter:main',
        ],
    },
)
