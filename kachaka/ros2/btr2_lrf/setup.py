from setuptools import find_packages, setup

package_name = 'btr2_lrf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['btr2_lrf.btr2_lrf'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wataru UEMURA',
    maintainer_email='wataru@kdel.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'btr2_lrf = btr2_lrf.btr2_lrf:main'
        ],
    },
)
