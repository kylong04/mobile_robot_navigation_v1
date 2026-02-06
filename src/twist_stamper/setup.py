from setuptools import setup

package_name = 'twist_stamper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/twist_stamper.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bluedragon-lab',
    maintainer_email='example@example.com',
    description='Convert geometry_msgs/Twist to geometry_msgs/TwistStamped',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_stamper = twist_stamper.twist_stamper:main',
        ],
    },
)
