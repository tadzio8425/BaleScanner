from setuptools import find_packages, setup

package_name = 'bale_scanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'open3d'],
    zip_safe=True,
    maintainer='tatan',
    maintainer_email='tatan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test = bale_scanner.test:main',
            'bale_reconstructor = bale_scanner.bale_reconstructor:main',
            'bale_isolator = bale_scanner.bale_isolator:main',
            'trigger_static = bale_scanner.trigger_static:main',
            'static_map_publisher = bale_scanner.static_map_publisher:main'
        ],
    },
)
