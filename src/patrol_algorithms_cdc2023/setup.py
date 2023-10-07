from setuptools import setup

package_name = 'patrol_algorithms_cdc2023'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, "patrol_algorithms_cdc2023.utils"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anthony Goeckner',
    maintainer_email='anthony.goeckner@northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = patrol_algorithms_cdc2023.Monitor:main',
            'BasePatrolAgent = patrol_algorithms_cdc2023.BasePatrolAgent:main',
            'AHPA = patrol_algorithms_cdc2023.AhpaAgent:main',
            'MARL = patrol_algorithms_cdc2023.PzAgent:main'
        ],
    },
)
