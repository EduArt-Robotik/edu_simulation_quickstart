from setuptools import find_packages, setup

package_name = 'edu_template'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hannes Duske',
    maintainer_email='hannes.duske@eduart-robotik.com',
    description='Package with template nodes for the EduArt robots',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dont_hit_the_wall_node = edu_template.dont_hit_the_wall:main',
            'dont_hit_the_wall_lidar_node = edu_template.dont_hit_the_wall_lidar:main'
        ],
    },
)
