## Overview

This tool allows to convert specific types of _PLY_ files into _uv3_ files. The conversion is limited to binary _PLY_ files (little endian) and always uses the vertex colors to assign polygon and line colors.

This tool also implements a fast conversion process in case the _PLY_ file contains only point primitives. In such case, the _PLY_ file can be arbitrary large.

## Usage

The following command allows to convert a _PLY_ file into a _uv3_ file :

    ./dalai-ply-uv3 -i /path/to/file.ply -o /path/to/convert.uv3

If the format of the _PLY_ file is not supported, the tool fails and display an error in the terminal.
