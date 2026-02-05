from setuptools import find_packages, setup

package_name = 'voicevox_ros2'

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
    maintainer='roboworks',
    maintainer_email='okdhryk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tts_node = voicevox_ros2.tts_node:main',
            'tts_client_example = voicevox_ros2.tts_client_example:main',
        ],
    },
)
