/*
 *  dalai-suite - uv3-ply
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2018 DHLAB, EPFL
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

    /*! \file   dalai-uv3-ply.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - uv3-ply
     */

    /*! \mainpage dalai-suite
     *
     *  \section _1 dalai-suite
     *
     *  The _dalai-suite_ is dedicated to the gathering and processing of
     *  geographical 3-dimensional information. It allows to considers the most
     *  common file formats and to convert them in a standardised and simple
     *  format.
     *
     *  This standardised format allows to use the suite tools for color
     *  edition, model cleaning and model hashing. In addition, the standard
     *  format is also expected by the _eratosthene-suite_ implementing the EPFL
     *  CDH DHLAB indexation server and its geographical 3-dimensional data
     *  injection tools.
     *
     *  \section _2 Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2018 DHLAB, EPFL
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_UV3_PLY__
    # define __DL_UV3_PLY__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <cstdint>
    # include <cinttypes>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define ply record */
    # define DL_PLY_VERTEX ( ( sizeof( float ) + sizeof( uint8_t ) ) * 3 )

    /* define ply record */
    # define DL_PLY_EDGE   ( sizeof( uint32_t ) * 2 )

    /* define ply record */
    # define DL_PLY_FACE   ( sizeof( uint8_t ) + sizeof( uint32_t ) * 3 )

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

    /* *** */

    le_void_t dl_uv3_ply_header( std::fstream & dl_stream, le_size_t const dl_vertex, le_size_t const dl_edge, le_size_t const dl_face );

    /* *** */

    le_void_t dl_uv3_ply_primitive( std::fstream & dl_stream, le_size_t * const dl_edge, le_size_t * const dl_face );

    /* *** */

    le_void_t dl_uv3_ply_vertex( std::fstream & dl_istream, std::fstream & dl_ostream );

    /* *** */

    le_void_t dl_uv3_ply_edge( std::fstream & dl_istream, std::fstream & dl_ostream );

    /* *** */

    le_void_t dl_uv3_ply_face( std::fstream & dl_istream, std::fstream & dl_ostream );

    /*! \brief main methods
     *
     *  The main function reads the provided universal format file and converts
     *  it into a ply file :
     *
     *      ./dalai-uv3-ply --uf3/-i [uf3 input file]
     *                      --ply/-o [ply output file]
     *
     *  The function starts by allocating the required i/o buffers memory and
     *  creating the i/o stream. It then creates and exports the output ply file
     *  header. It then reads the input file by chunk. Each chunk is converted
     *  and written in the output stream. As all the input file chunks have been
     *  read, the function closes the stream and release the allocated memory.
     *
     *  The ply format, being a very complicated format, is considered only for
     *  its little endian binary format. The function does not allow to choose
     *  another format to avoid too large code for a simple conversion tool.
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

