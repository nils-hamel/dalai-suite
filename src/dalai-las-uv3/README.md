## Overview

This tool allows to convert an _las_ (ASPRS)_ file into a _uv3_ file. It allows to switch through different extraction switches to obtain the colors, the classification or the intensities of the model stored in _las_ format.

## Usage

Considering a _las_ file containing a model with _RGB_ colors, the following command allows to convert the _las_ file into a color _uv3_ file :

    ./dalai-las-uv3 -i /path/to/file.las -o /path/to/converted.uv3 --color

The _--color_ switch indicates the process to consider the _las_ _RGB_ colors during the conversion. The colors have to be available and in the correct format (_16_ bits) in the input file. The following command :

    ./dalai-las-uv3 -i /path/to/file.las -o /path/to/converted.uv3 --intensity

allows to create an _uv3_ file with colors corresponding to the _las_ model intensities. Finally, the following command :

    ./dalai-las-uv3 -i /path/to/file.las -o /path/to/converted.uv3 --classification

allows to create an _uv3_ file with colors corresponding to the _las_ model classification. A default color mapping is used to assign a color to each class. If no extraction switch is provided, the classification switch is considered by default. In case multiple extraction switch are specified, only the first one is considered.

The following images give an illustration of the conversion of two _las_ files : the left image showing the result of the classification switch when the right one shows the result of the intensities switch :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-las-uv3/doc/las-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-las-uv3/doc/las-2.jpg?raw=true" width="384">
<br />
<i>Conversion using classification (left) and intensities (right) - Data : SITG and New York City (US-GOV)</i>
</p>
<br />

In some cases, the _las_ color information or intensities can be coded in way that are not understood by this tool. In such case, unexpected results can occur.
