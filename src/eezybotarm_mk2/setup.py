from setuptools import setup

package_name = 'eezybotarm_mk2'
submodules = "eezybotarm_mk2/lib"

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raul Bezares',
    maintainer_email='rbezares5@uma.es',
    description='Control of the eezybotarm_mk2 and communication via arduino',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker1 = eezybotarm_mk2.talker1:main',
            'talker3 = eezybotarm_mk2.talker3:main',
            'listener1 = eezybotarm_mk2.listener1:main',
            'listener2 = eezybotarm_mk2.listener2:main',
            'client_move = eezybotarm_mk2.client1:main',
            'service_move = eezybotarm_mk2.service1:main',
        ],
    },
)
