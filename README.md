## Overview

The _dalai-suite_ is part of the Eratosthene Project and is dedicated to 3D models gathering, processing and format conversion. It allows to consider the most common formats and to convert them into the project common format. The _dalai-suite_ is mainly used for data gathering and processing, allowing to make them compatible with remote Eratosthene servers injection format.

In addition, the project common format allows to apply the _dalai-suite_ solutions for 3D models processing. The suite offers solution for massive 3D models cleaning, hashing and assisted geographical registration. The suite also provides solution for 3D models display and analysis.

## Suite Solutions

This section gives an overview of the tools available in the _dalai-suite_. Its purpose is to provide examples of the suite solutions applied on actual data.

In the first place, the _dalai-vision_ solution allows to read and display the content of the project common format. It allows to view models through a simple graphical interface to analyse point-based models, polygonal models and mixed models.

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-1a.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-1b.jpg?raw=true" width="384">
<br />
<i>Vision interface showing models of the City of Geneva - Data : SITG and DHLAB</i>
</p>
<br />

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-2a.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-2b.jpg?raw=true" width="384">
<br />
<i>Vision interface showing models of the City of Geneva in 1850 and 2018 - Data : Ville de Genève and SITG</i>
</p>
<br />


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