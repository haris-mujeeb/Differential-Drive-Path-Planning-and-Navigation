from setuptools import find_packages, setup

package_name = 'bumperbot_py_examples'

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
    maintainer='haris',
    maintainer_email='harrismuib12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'simple_tf_kinematics = bumperbot_py_examples.simple_tf_kinematics:main',
          'simple_service_server = bumperbot_py_examples.simple_service_server:main',
          'simple_lifecycle_node = bumperbot_py_examples.simple_lifecycle_node:main',
          'simple_qos_publisher = bumperbot_py_examples.simple_qos_publisher:main',
          'simple_qos_subscriber = bumperbot_py_examples.simple_qos_subscriber:main',
        ],
    },
)
