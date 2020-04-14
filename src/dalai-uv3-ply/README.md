## Overview

This tool allows to convert an _uv3_ file back into a _ply_ file. It also gives the possibility to only export the _uv3_ vertex, even if lines and triangles are present. The little endian binary format is always considered for _ply_ files.

## Usage

To convert an _uv3_ file into a _ply_ file, use the command :

    ./dalai-uv3-ply -i /path/to/file.uv3 -o /path/to/convert.ply

Adding the _--vertex-only_ flag allows to only convert the vertex :

    ./dalai-uv3-ply -i /path/to/file.uv3 -o /path/to/convert.ply --vertex-only

which leads to a _ply_ file containing only points with color.