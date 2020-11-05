/*
 *  dalai-suite - shift
 *
 *      Nils Hamel - nils.hamel@alumni.epfl.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
 *      Copyright (c) 2020 Republic and Canton of Geneva
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

    /*! \file   dalai-shift.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - shift
     */

    /*! \mainpage dalai-suite
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2020 DHLAB, EPFL
     *  Copyright (c) 2020 Republic and Canton of Geneva
     * 
     *  This program is licensed under the terms of the GNU GPLv3. Documentation
     *  and illustrations are licensed under the terms of the CC BY 4.0.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_SHIFT__
    # define __DL_SHIFT__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

/*
    header - function prototypes
 */

    /*! \brief main methods
     *
     *  The main function reads the content of the provided input uv3 file and
     *  applies the provided translation on the vertex :
     *
     *      ./dalai-shift --input/-i [uv3 input file]
     *                    --output/-o [uv3 output file]
     *                    --x/-x [x-dimension shift]
     *                    --y/-y [y-dimension shift]
     *                    --z/-z [z-dimension shift]
     *
     *  The main function starts by opening both input and output files before
     *  to read the input stream chunk by chunk. The provided shift values are
     *  then added to the respective coordinates of each record of each chunk :
     *
     *      vertex_x = vertex_x + shift_x
     *      vertex_y = vertex_y + shift_y
     *      vertex_z = vertex_z + shift_z
     *
     *  The processed chunks are then exported one by one in the created output
     *  file.
     *
     *  \param  argc Standard parameter
     *  \param  argv Standard parameter
     *
     *  \return Returns standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

