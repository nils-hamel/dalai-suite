/*
 *  dalai-suite - ply-uf3
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2017 EPFL CDH DHLAB
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

    /*! \file   dalai-ply-uf3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite ply-uf3
     */

    /*! \mainpage dalai-suite
     *
     *  \section dalai-suite
     *
     *  The _dalai-suite_ is designed to be a gathering and standardising
     *  interface to the geodetic system developed at the DHLAB of EPFL. It
     *  consists in a set of softwares having to convert any kind of geodetic
     *  data, mainly point clouds, into the defined universal format considered
     *  in the geodetic system. In other words, it is well described by its
     *  tibetan name.
     *
     *  \section Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2017 EPFL CDH DHLAB
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_PLY_UF3__
    # define __DL_PLY_UF3__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <common-include.h>

/*
    header - preprocessor definitions
 */

    /* define ply format chunk size */
    # define DL_PLY_UF3_CHUNK ( 131072ll )

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

    /*! \brief main function
     *
     *  The main function reads the provided ply file content and converts it
     *  into a universal format file :
     *
     *      ./dalai-ply-uf3 --ply/-i [ply_file]
     *                      --uf3/-o [uf3_file]
     *
     *  The function starts by allocating i/o buffer memory and opens streams
     *  toward input and output file. It the reads the content of the input
     *  file by chunk. The chunks are converted into universal format before to
     *  be exported in the output file. The function ends by closing the streams
     *  and releasing the allocated memory.
     *
     *  Due to the complexity of the ply format, the main function only accept
     *  little endian binary format. In addition, the function, thats operates
     *  through the common library ply module, only considers vertex elements
     *  x, y, z, red, green and blue. All other element and properties are not
     *  handled by the function (i.e. the common library module).
     *
     *  \param argc Standard parameter
     *  \param argv Standard parameter
     *
     *  \return Returns standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

