#!/usr/bin/env python3
import setuptools

with open('README.md', 'r') as fh:
    long_description = fh.read()

setuptools.setup(
    name='PythonForMecademic',
    version='0.1.0',
    author='Nick Armenta',
    author_email='nick@advin.io',
    license='MIT',
    description='A package to control the Mecademic Robots through Python3',
    long_description=long_description,
    long_description_content_type='markdown',
    url='https://github.com/nickarmenta/PythonForMecademic',
    packages=setuptools.find_packages(),
    data_files=[('',['LICENSE', 'README.md'])],
    include_package_data=True,
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: MIT License',
        'Operating System :: OS Independent',
    ],
    entry_points={
        'console_scripts': [
            'FirmwareUpdate = FirmwareUpdate:main',
        ],
    }
)
