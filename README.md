## Overview

The _dalai-suite_ is dedicated to the gathering and processing of geographical 3-dimensional information. It allows to considers the most common file formats and to convert them in a standardised and simple format.

This standardised format allows to use the suite tools for color edition, model cleaning and model hashing. In addition, the standard format is also expected by the _eratosthene-suite_ implementing the EPFL CDH DHLAB indexation server and its geographical 3-dimensional data injection tools.

## Copyright and License

**dalai-suite** - Nils Hamel <br >
Copyright (c) 2016-2018 DHLAB, EPFL

This program is licensed under the terms of the GNU GPLv3.

## Dependencies

The _dalai-suite_ comes with the following package dependencies (Ubuntu 16.04 LTS) :

* build-essential
* liblas-c-dev
* mesa-common-dev
* libsdl2-dev
* libeigen3-dev

and the following external dependencies provided as sub-modules :

* liberatosthene 1.1

The code documentation is built using Doxygen.

## Compilation

To build the project, including the sub-modules, use make with the following targets :

    $ make clean-all && make all

To rebuild the binaries without rebuilding sub-modules, use the make targets :

    $ make clean && make build

To only rebuild sub-modules, use the make command with the targets :

    $ make clean-module && make module

To generate the documentation of the project, use the targets :

    $ make clean-doc && make doc

and the targets :

    $ make clean-all-doc && make all-doc

to generate the documentation of the project and its sub-modules.