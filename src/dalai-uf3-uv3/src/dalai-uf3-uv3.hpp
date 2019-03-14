/*
 *  dalai-suite - uf3-uv3
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2019 DHLAB, EPFL
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

    /*! \file   dalai-uf3-uv3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - uf3-uv3
     */

    /*! \mainpage dalai-suite
     *
     *  \section overview Overview
     *
     *  The _dalai-suite_ is part of the Eratosthene Project and is dedicated to
     *  3D models gathering, processing and format conversion. It allows to
     *  consider the most common formats and to convert them into the project
     *  common format. The _dalai-suite_ is mainly used for data gathering and
     *  processing, allowing to make them compatible with remote Eratosthene
     *  servers injection format.
     *
     *  In addition, the project common format allows to apply the _dalai-suite_
     *  solutions for 3D models processing. The suite offers solution for
     *  massive 3D models cleaning, hashing and assisted geographical
     *  registration. The suite also provides solution for 3D models display and
     *  analysis.
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2019 DHLAB, EPFL
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_UF3_UV3__
    # define __DL_UF3_UV3__

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

    /* define uf3 element */
    # define DL_UF3_POSE   ( sizeof( le_real_t ) * 3 )
    # define DL_UF3_DATA   ( sizeof( le_data_t ) * 3 )

    /* define uf3 record */
    # define DL_UF3_RECORD ( DL_UF3_POSE + DL_UF3_DATA )

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
     *  The main function reads the provided uf3 stream and converts it into an
     *  uv3 stream :
     *
     *      ./dalai-uf3-uv3 --uf3/-i [uf3 input file]
     *                      --uv3/-o [uv3 output file]
     *
     *  The main function starts by creating the two input and output streams
     *  before to starts the conversion. The input file is read by chunks and
     *  each chunk is converted from uf3 to uv3 format. The converted chunks are
     *  then exported in the output stream.
     *
     *  This software is design to ensure compatibility with the previous uf3
     *  format used by the dalai-suite.
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

