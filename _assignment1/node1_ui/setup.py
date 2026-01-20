from setuptools import find_packages, setup

package_name = 'node1_ui'  

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
    maintainer='cm',
    maintainer_email='chiara.masa@hotmail.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node_ui = script.node_ui:main',
            'turtle_spawn = script.turtle_spawn:main',
        ],
    },
)
