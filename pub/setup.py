from setuptools import find_packages, setup

package_name = 'pub'

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
    maintainer='marton',
    maintainer_email='marciszanda@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = pub.publisher_member_function:main',
            'listener = pub.subscriber_member_function:main',
            'listener_joint_states = pub.subscriber_member_function_joint_states:main',
            'talker_backup = pub.publisher_member_function_backup:main',
        ],
    },
)
