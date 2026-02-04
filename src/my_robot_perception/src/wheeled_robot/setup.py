from setuptools import setup

package_name = 'wheeled_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL',
    description='Wheeled robot control package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'diff_drive_publisher = wheeled_robot.diff_drive_publisher:main',
        ],
    },
)
