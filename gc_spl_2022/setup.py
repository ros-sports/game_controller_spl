from setuptools import setup

package_name = 'gc_spl_2022'

setup(
    name=package_name,
    version='4.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ijnek',
    maintainer_email='kenjibrameld@gmail.com',
    description='GameController-Robot communication in RoboCup SPL at RoboCup2022',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gc_spl = gc_spl_2022.gc_spl:main',
        ],
    },
)
