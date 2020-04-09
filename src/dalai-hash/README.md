## Overview

This tool is made to handle very large 3D models contained in _uv3_ files. As models can reach tens of gigabytes and more, it is impracticable to work on them as a single monolithic model. This tool allows to split the models into smaller pieces based on an hashing process made on the coordinates of the _uv3_ file vertex.

This process can be used to view, analyze and work on model too large to fit on volatile memory. It is also used for large scale models alignment in geographical frames by working on homologous point found in the different pieces of the hashed models.

## Usage

Considering an model contained in a _uv3_ file, the command :

    ./dalai-hash -i /path/to/file.uv3 -o /path/to/directory -c 64 -p 500

allows to hash the contained model into smaller pieces.

The hashing process is based on analysis of the model vertex coordinates. A floating point congruence condition is applied to send the primitives in the different pieces. Considered the first spatial coordinates, the condition applies as follows :

    piece_x = floor( vertex_x / distance )

In other words, the space is divided into cells resulting of the rounding of coordinates on the distance :

    distance = ( --parameter/-p ) * ( minimum distance mean value )

The exportation file are named using the rounded value in each dimension of space.

The _minimum distance mean value_ is computed considering a sample of the model vertex and by determining the distance to their closest point of the model. The mean of these minimum distances is called the _minimum distance mean value_. The size of the sample can be modified using the _--count/-c_ parameter. Increasing the sample size allows to get a more accurate value but increases the computation load (non linear).

The parameter _--parameter/-p_ is used as a multiplier of the _minimum distance mean value_ to determine the size of the hashed pieces.

Applying the example command above on a model of _Geneva_ city, the following result can be obtained : on the right image, only half of the obtained pieces are displayed to give an idea of the nature of the obtained pieces :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-hash/doc/hash-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-hash/doc/hash-2.jpg?raw=true" width="384">
<br />
<i>Original model (left) and half of its pieces (right) </i>
</p>
<br />

To modulate the size of the hashed pieces, on has to vary the value provided as _--parameter/-p_ parameter. In addition, on could consider to shift or rotate the coordinates of the model before to apply the hashing process to obtain the desired result.
