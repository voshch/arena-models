from setuptools import find_packages, setup

package_name = 'arena_models'

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
    maintainer='kuro',
    maintainer_email='giang.nht108201@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'build = arena_models.build_db:main',
            'query = arena_models.query_db:main',
            'get = arena_models.get_db:main',
            'merge = arena_models.merge_db:main',
            'down = arena_models.down_db:main'
        ],
    },
)
