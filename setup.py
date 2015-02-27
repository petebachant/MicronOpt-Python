#!/usr/bin/env python
# coding=utf-8

from distutils.core import setup
from micronopt import __version__

setup(
    name='MicronOpt',
    version=__version__,
    author='Pete Bachant',
    author_email='petebachant@gmail.com',
    py_modules=['micronopt'],
    scripts=[],
    url='https://github.com/petebachant/MicronOpt-Python.git',
    license='MIT',
    description='Python module for working with Micron Optics sensors and interrogators.',
    long_description=open('README.md').read(),
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "Intended Audience :: Science/Research",
        "Natural Language :: English",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python",
        "Programming Language :: Python :: 2",
        "Programming Language :: Python :: 2.6",
        "Programming Language :: Python :: 2.7",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.3",
        "Programming Language :: Python :: 3.4"]
)
