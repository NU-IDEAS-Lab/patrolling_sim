from setuptools import setup

package_name = 'patrol_algorithms_ahpa'

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
    maintainer='Anthony Goeckner',
    maintainer_email='anthony.goeckner@northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = patrol_algorithms_ahpa.Monitor:main',
            'BasePatrolAgent = patrol_algorithms_ahpa.BasePatrolAgent:main',
            'AHPA = patrol_algorithms_ahpa.AhpaAgent:main'
        ],
    },
)
