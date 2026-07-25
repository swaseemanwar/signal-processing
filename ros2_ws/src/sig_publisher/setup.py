from setuptools import setup

package_name = 'sig_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Waseem Anwar',
    maintainer_email='swaseemanwar@gmail.com',
    description='Sensor publisher node — synthetic and replay modes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sig_publisher = sig_publisher.publisher_node:main',
        ],
    },
)
