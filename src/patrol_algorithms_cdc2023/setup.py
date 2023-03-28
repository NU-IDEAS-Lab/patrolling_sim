from setuptools import setup

package_name = 'patrol_algorithms_cdc2023'

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
    maintainer='anthony',
    maintainer_email='anthony.goeckner@northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'BasePatrolAgent = patrol_algorithms_cdc2023.BasePatrolAgent:main'
        ],
    },
)
