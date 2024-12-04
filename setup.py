from setuptools import find_packages, setup

package_name = 'rqt_chat'

setup(
    name=package_name,
    version='1.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
        ('share/' + package_name + '/resource', ['resource/chat.ui',
                                                 'resource/send-circle.svg',
                                                 'resource/robot.svg',
                                                 'resource/face.svg',
                                                 'resource/intent.svg',
                                                 'resource/waiting-icon.svg',
                                                 ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='severin.lemaignan@pal-robotics.com',
    description='A ROS4HRI-compatible chat interface for RQt',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
