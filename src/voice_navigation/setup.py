from setuptools import setup

package_name = 'voice_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/keypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arjun Girish',
    maintainer_email='arjungirish2009@email.com',
    description='Voice-controlled navigation using Vosk and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_command_node = voice_navigation.voice_command_node:main',
        ],
    },
)

