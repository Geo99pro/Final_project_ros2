from setuptools import find_packages, setup

package_name = 'gpg_fproject_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['gpg_fproject_controller/controller.py']),
        ('share/' + package_name, ['gpg_fproject_controller/reach_goal.py']),
        ('share/' + package_name, ['launch/launch.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lci',
    maintainer_email='lhodonou349@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'reach_goal = gpg_fproject_controller.reach_goal:main',
                            'controller = gpg_fproject_controller.controller:main'
        ],
    },
)
