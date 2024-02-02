from setuptools import find_packages, setup

package_name = 'graph_search_pkg'

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
    maintainer='adiprash',
    maintainer_email='adiprash@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = graph_search_pkg.publisher:main',
            'subscriber = graph_search_pkg.subscriber:main',
        ],
    },
)
