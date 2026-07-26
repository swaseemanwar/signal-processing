from setuptools import setup

package_name = 'sig_proc_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Waseem Anwar',
    maintainer_email='swaseemanwar@gmail.com',
    description='Python processing node using siglib_py pybind11 bindings',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sig_proc_py = sig_proc_py.proc_node:main',
            'parity_check = sig_proc_py.parity_check:main',
        ],
    },
)
