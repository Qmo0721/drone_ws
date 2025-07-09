from setuptools import setup

package_name = 'drone_pkg'

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
    maintainer='shengzi',
    maintainer_email='lizhengyu19980105@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "control0 = drone_pkg.control0:main",
            "control1 = drone_pkg.control1:main",
            "control_can = drone_pkg.control_can:main",
            "control_bt = drone_pkg.control_bt:main",
            "view0 = drone_pkg.view0:main",
            "view1 = drone_pkg.view1:main",
        ],
    },
)
