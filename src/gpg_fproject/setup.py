from setuptools import find_packages, setup

package_name = 'gpg_fproject'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch.py']),
        ('share/' + package_name, ['launch/gpg_fproject_launch.py']),
        ('share/' + package_name, ['gpg_fproject/controllers.yaml']),
        ('share/' + package_name, ['gpg_fproject/robot.urdf']),
        ('share/' + package_name, ['gpg_fproject/image_node.py']),
        ('share/' + package_name, ['gpg_fproject/user_node.py']),
        ('share/' + package_name, ['gpg_fproject/utils.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lci',
    maintainer_email='lhodonou349@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'image_node = gpg_fproject.image_node:main',
                                'user_node = gpg_fproject.user_node:main'
        ],
    },
)
