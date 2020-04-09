## Overview

This tools allows to apply a standard color maps to the provided model based on the height of its vertex. The height is here understood simply as the last vertex coordinates value.

## Usage

Considering a model contain in an _uv3_ file, the following command :

    ./dalai-color -i /path/to/file.uv3 -o /path/to/colored.uv3 -m 0.8 -x 0.9

allows to apply the color map on each vertex of the model based on their height values. The minimum (_--minimum/-m_) and maximum (_--maximum/-x_) height values are used to determine the starting and ending point of the color map. The values of height outside of this defined range are colored using a cyclic condition.

The following illustrations shows the result obtained using this tool :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-color/doc/color-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-color/doc/color-2.jpg?raw=true" width="384">
<br />
<i>Original model (left) and its colored counterpart (right)</i>
</p>
<br />

One can see how the application of an height-based color mapping can improve the readability of the model. On this example, one can much better see the variation of the topography on such sparse model.