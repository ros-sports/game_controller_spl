from setuptools import find_packages, setup

package_name = 'game_controller_spl'

setup(
    name=package_name,
    version='4.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ijnek',
    maintainer_email='kenjibrameld@gmail.com',
    description='GameController-Robot communication in RoboCup SPL',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_controller_spl = game_controller_spl.game_controller_spl:main',
        ],
    },
)
