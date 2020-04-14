## Overview

This tool allows to convert an _uf3_ file into an _uv3_ file. It is proposed to ensure ascending compatibility for the previous _uf3_ format.

## Usage

To promote an _uf3_ file into an _uv3 file, use the command :

    ./dalai-uf3-uv3 -i /path/to/file.uf3 -o /path/to/convert.uv3

As the _uf3_ only contains points, the created _uv3_ file also contains only point primitives.
