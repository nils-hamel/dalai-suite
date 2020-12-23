## Overview

The _dalai-suite_ offers a toolbox for large scale 3D models manipulation, processing, conversion and geographical registration. It is mainly designed to offers solutions suitable for models that can weight hundreds of gigabytes and that are used in the domain of environment survey and land registering. It was initially developed to offer the required tools needed to allows massive data processing and preparation for the [_eratosthene_](https://github.com/nils-hamel/eratosthene-suite) framework.

The _dalai-suite_ uses a common and very simple format for all the implemented tools that is also used for the _Eratosthene Project_. This format offers a simple way of storing massive amount of graphical primitives. This allows the _dalai-suite_ to manipulate all sort of 3D models, from large scale point-based models to more refined vector models.

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/vision-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/vision-2.jpg?raw=true" width="384">
<br />
<i>Illustration of point-based models - Data : SITG and DHLAB</i>
<br />
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/vision-3.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/vision-4.jpg?raw=true" width="384">
<br />
<i>Illustration of vector-based models - Data : Ville de Genève and SITG</i>
</p>
<br />

The _dalai-suite_ comes also with tools dedicated to models visualization and large scale models processing such as automatic segmentation, cleaning, geographical transformation and assisted geographical registration.

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/register-2.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-las-uv3/doc/las-2.jpg?raw=true" width="384">
<br />
<i>Illustration of registration tool and data conversion (las) - Data : SITG and New York City (US-GOV)</i>
</p>
<br />

The _dalai-suite_ also comes with a set of tools dedicated to file format conversion that allows data coming in their specific formats to be converted in the _suite_ format.

## dalai-suite

Each implemented tool of the suite comes with its own documentation that explains the implemented solution and gives examples of usages and results :

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
* [LAS to UV3](src/dalai-las-uv3)
* [PLY to UV3](src/dalai-ply-uv3)
* [UF3 to UV3](src/dalai-uf3-uv3)
* [UV3 to PLY](src/dalai-uv3-ply)

A detailed documentation of specific file formats used by the tools of this suite can be found of the [format page](FORMAT.md).

## Copyright and License

**dalai-suite** - Nils Hamel <br >
Copyright (c) 2016-2020 DHLAB, EPFL <br />
Copyright (c) 2020 Republic and Canton of Geneva

This program is licensed under the terms of the GNU GPLv3. Documentation and illustrations are licensed under the terms of the CC BY 4.0.

## Dependencies

The _dalai-suite_ comes with the following package (Ubuntu 16.04 LTS) dependencies ([Instructions](DEPEND.md)) :

* build-essential
* *liblas-c-dev*
* mesa-common-dev
* libsdl2-dev
* libeigen3-dev
* libgeographic-dev

and the following external dependencies provided as sub-modules ([Instructions](DEPEND.md)) :

* liberatosthene 1.3

The code documentation is built using Doxygen.

It's important to state that when working with Ubuntu 20.04 LTS or superior, the *liblas-c-dev* package is missing. Therefore the dalai-las-uv3 tool won't be working and it will break your installation. This way, in order to be able to work with all the other tools from dalai-suite, remove the dalai-las-uv3 folder in /dalai-suite/src/ (https://github.com/nils-hamel/dalai-suite/tree/master/src/dalai-las-uv3) before compilation (next step).

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

## Possible issues

If some issues are present whilst compiling dalai-suite, you could try the following:

* Make sure you deleted dalai-las-uv3 folder if you're unable to install liblas-c-dev (usually for Ubuntu 20.04 or superior users)

* try recompiling with 
```
    $ git clone https://github.com/nils-hamel/dalai-suite.git --recursive
```
* try the following before recompiling
```
    $ git submodule update --remote 
```
