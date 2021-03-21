# Python control for Mecademic

A python module designed for Meca500 robots. The module offers tools that give access to most features of the Mecademic Robots with useful movement and settings functionality.

### Supported Robots

 * Meca500 R3

## Getting Started

These instructions will allow you to control the Robots from Mecademic through a python package and use the package for development and deployment purposes. [The user programming manual](https://mecademic.com/resources/documentation) or the documentation in the module is sufficiant to be able to make the Robot perform actions and control the robot.

## Prerequisites

To be able to use the module without unexpected errors, the user must have a copy of python installed on their machine and it is recommended to use python version 3.6 or higher. [Python](https://www.python.org/) can be installed from its main website (a reboot will be require after the installation to complete the setup). 

## Installing the package

You can use pip to install the library:

```sh
user@host:~$ pip install git+https://github.com/nickarmenta/PythonForMecademic
``` 

And/or build off the provided Dockerfile

```sh
user@host:~$ sudo apt-get -y install git
user@host:~$ git clone https://github.com/nickarmenta/PythonForMecademic
user@host:~$ cd ./PythonForMecademic
user@host:~/PythonForMecademic$ docker build mecademic:latest .
``` 

## Using the library

See [OVERVIEW.md](OVERVIEW.md) for a guide to how to use the module. 

## License

All packages in this repository are licensed under the MIT license.

## Authors 

* **Mecademic** - *Continuous work*
* Nick Armenta
