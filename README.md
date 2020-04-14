## Overview

The _dalai-suite_ is part of the Eratosthene Project and is dedicated to 3D models gathering, processing and format conversion. It allows to consider the most common formats and to convert them into the project common format. The _dalai-suite_ is mainly used for data gathering and processing, allowing to make them compatible with remote Eratosthene servers injection format.

In addition, the project common format allows to apply the _dalai-suite_ solutions for 3D models processing. The suite offers solution for massive 3D models cleaning, hashing and assisted geographical registration. The suite also provides solution for 3D models display and analysis.

## Solutions Presentation

This section gives an overview of the tools available in the _dalai-suite_. Its purpose is to provide examples of the suite solutions applied on actual data.

In the first place, it allows the _dalai-vision_ solution to read and display the content of the project common format. It allows it to view models through a simple graphical interface in both case of point-based models :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-1a.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-1b.jpg?raw=true" width="384">
<br />
<i>Vision interface showing models of the City of Geneva - Data : SITG and DHLAB</i>
</p>
<br />

and polygonal models as illustrated with models of the City of Geneva :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-2a.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-2b.jpg?raw=true" width="384">
<br />
<i>Vision interface showing models of the City of Geneva in 1850 and 2018 - Data : Ville de Genève and SITG</i>
</p>
<br />

In addition to models visualization, the _dalai-vision_ also implements an assisted geographical registration tool mainly used for point-based models registration in a specific coordinate system. The tool allows the user to extrapolate surfaces of the models to allow the determination of precise intersections able to be used a control points during registration process :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-3a.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/vision-3b.jpg?raw=true" width="384">
<br />
<i>Example of ideal intersection extraction through surfaces extrapolation - Data : SITG and DHLAB</i>
</p>
<br />

Model filtering is also available through the _dalai-filter_ tool that allows to deduce clean representation of dense point-based models. It is especially adapted for dense models deduced from image sequences. The tool using the project common format and taking advantage of the model hashing, it allows to filter arbitrary large point-based models :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/filter-1a.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/filter-1b.jpg?raw=true" width="384">
<br />
<i>Original model (left) and its filtered counterpart (right) - Data : SITG and DHLAB</i>
</p>
<br />

As a last illustration, the _suite_ also provides conversion tools allowing to bring data from standard format, such as polygon file format (PLY) and ASPRS (LAS) model, to the project common format. The conversion from LAS format allows to convert the element using their provided color, their classification or their intensities :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/convert-1a.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/doc/image/convert-1b.jpg?raw=true" width="384">
<br />
<i>ASPRS LAS converted using classification (left) and intensities (right) - Data : SITG and New York City (US-GOV)</i>
</p>
<br />

The _dalai-suite_ offer the required tools to collect, convert, process large amount of 3D data making them available for injection in remote Eratosthene servers.

## Dalai-Suite

Each implemented tool comes with its own documentation that explains the implemented solution and gives examples of usages and results :

_Display tools_

* [UV3 file visualization and registration](src/dalai-vision)
* [UV3 file content display](src/dalai-cat)

_Models automated filtering and cleaning_

* [UV3 file statistical filtering](src/dalai-filter)

_Models automated transformation and editing_

* [UV3 file translation](src/dalai-shift)
* [UV3 file geoid heights conversion](src/dalai-geoid)
* [UV3 file height-based coloring](src/dalai-color)
* [UV3 file hashing](src/dalai-hash)

_Models format conversion_

* [INTERLIS to UV3](src/dalai-interlis-uv3)

A detailed documentation of specific file formats used by the tools of this suite can be found of the [format page](FORMAT.md).

## Copyright and License

**dalai-suite** - Nils Hamel <br >
Copyright (c) 2016-2019 DHLAB, EPFL

This program is licensed under the terms of the GNU GPLv3.

## Dependencies

The _dalai-suite_ comes with the following package dependencies (Ubuntu 16.04 LTS) :

* build-essential
* liblas-c-dev
* mesa-common-dev
* libsdl2-dev
* libeigen3-dev
* libgeographic-dev

and the following external dependencies provided as sub-modules :

* liberatosthene 1.2

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