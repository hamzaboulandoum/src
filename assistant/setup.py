from setuptools import setup

package_name = 'assistant'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hamza Boulandoum',
    maintainer_email='hamzaboulandoum@gmail.com',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'talker = assistant.publisher_member_function:main',
        'listener = assistant.subscriber_member_function:main',
        'driver = assistant.driver:main',
        'broadcaster = assistant.broadcaster:main'
        ],
    },
)
