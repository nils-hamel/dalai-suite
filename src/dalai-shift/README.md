## Overview

This tool allows to apply a translation on the vertex coordinates of the specified _uv3_ files. The translation is provided component-wise.

## Usage

The following command :

    ./dalai-shift -i /path/to/file.uv3 -o /path/to/shifted.uv3 -x 10 -y -10

allows to apply on the vertex coordinates the transformation :

    x = x + 10
    y = y - 10
    z = z

If a component of the translation is not provided by the user, it is assumed to be zero.

Applying the previous command on the following _uv3_ file content :

    +3.142440e-01 +1.277200e+00 +1.642900e+00 01 6d 69 6e
    +4.337270e-01 +1.403630e+00 +1.683750e+00 01 ce bc 95
    +3.554590e-01 +1.382040e+00 +1.646040e+00 01 67 71 78
    +3.531210e-01 +1.405240e+00 +1.649870e+00 01 82 77 76
    +3.466060e-01 +1.384230e+00 +1.645080e+00 01 ba b8 a6
    +3.268300e-01 +1.407480e+00 +1.648720e+00 01 7c 76 76
    +4.386670e-01 +1.407980e+00 +1.683210e+00 01 59 50 57
    +3.320050e-01 +1.407900e+00 +1.649510e+00 01 81 7d 7e
    ...

produce the following content in the shifted _uv3_ file :

    +1.031424e+01 -8.722800e+00 +1.642900e+00 01 6d 69 6e
    +1.043373e+01 -8.596370e+00 +1.683750e+00 01 ce bc 95
    +1.035546e+01 -8.617960e+00 +1.646040e+00 01 67 71 78
    +1.035312e+01 -8.594760e+00 +1.649870e+00 01 82 77 76
    +1.034661e+01 -8.615770e+00 +1.645080e+00 01 ba b8 a6
    +1.032683e+01 -8.592520e+00 +1.648720e+00 01 7c 76 76
    +1.043867e+01 -8.592020e+00 +1.683210e+00 01 59 50 57
    +1.033200e+01 -8.592100e+00 +1.649510e+00 01 81 7d 7e
    ...

where the _x_ and _y_ vertex coordinates are translated while the _z_ one is left unchanged.