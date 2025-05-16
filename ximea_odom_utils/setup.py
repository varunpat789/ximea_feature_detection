from setuptools import find_packages, setup

package_name = 'ximea_odom_utils'

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
    maintainer='biorobotics',
    maintainer_email='varunpat789@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_frame_extractor = ximea_odom_utils.bag_frame_extractor:main'
        ],
    },
)
