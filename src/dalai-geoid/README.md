## Overview

This tool allows to convert the heights of models expressed in a geographic coordinates system from _mean sea level_ to _geoid_ heights. The models are then expected to be correctly aligned in a coordinates frame and that a geoid model is available for such frame.

As this tool considers _uv3_ file as input and output, it is expected that the coordinates associate to longitude come in the first place (_x_), the ones associated to latitude in the second place (_y_) and the height in the last place (_z_). In addition, all angles are expected in radian (not in decimal degrees).

## Usage

Considering an _uv3_ file containing a model of the _Chateau de Sceaux_ near _Paris_ and aligned in the _WGS84_ coordinates frame with heights above the _WGS84_, the command :

    ./dalai-geoid -i /path/to/file.uv3 -o /path/to/converted.uv3 -p /path/to/geoid -n egm96-5 --to-msl

allows to convert the heights from geoid to MSL heights. The following gives a dump of the eight first vertex of the original _uv3_ file :

    4.014597e-02 8.512707e-01 1.351008e+02 01 1b 37 23
    4.014597e-02 8.512707e-01 1.350700e+02 01 3a 4c 52
    4.014598e-02 8.512707e-01 1.350375e+02 01 20 3a 34
    4.014599e-02 8.512707e-01 1.350093e+02 01 16 34 25
    4.014377e-02 8.512749e-01 1.569343e+02 01 29 35 51
    4.014376e-02 8.512749e-01 1.569403e+02 01 2d 38 51
    4.014377e-02 8.512749e-01 1.568863e+02 01 26 34 4f
    4.014376e-02 8.512749e-01 1.569000e+02 01 2d 37 53
    ...

and the following dump show the result of the conversion :

    4.014597e-02 8.512707e-01 9.050670e+01 01 1b 37 23
    4.014597e-02 8.512707e-01 9.047588e+01 01 3a 4c 52
    4.014598e-02 8.512707e-01 9.044335e+01 01 20 3a 34
    4.014599e-02 8.512707e-01 9.041513e+01 01 16 34 25
    4.014377e-02 8.512749e-01 1.123403e+02 01 29 35 51
    4.014376e-02 8.512749e-01 1.123463e+02 01 2d 38 51
    4.014377e-02 8.512749e-01 1.122922e+02 01 26 34 4f
    4.014376e-02 8.512749e-01 1.123059e+02 01 2d 37 53
    ...

In this conversion, the _EGM96-5_ geoid model was used (see illustration below). Available geoid models can be downloaded from [this page](https://geographiclib.sourceforge.io/html/geoid.html). Of course, the used geoid model has to be adapted to the coordinates system of the vertex of the converted _uv3_ file.

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-geoid/doc/egm96-5.jpg?raw=true" width="512">
<br />
<i>Illustration of the EGM96-5 geoid model - Source : S. Th. Kouzeleas</i>
</p>
<br />

The tool allows also batch processing of _uv3_ files. If the _--input/-i_ parameter points to a directory, all the _uv3_ files it contains are converted. In such a case, the __--output/-o_ parameter has to point to a different directory as the resulting files are name identically to the original files.
