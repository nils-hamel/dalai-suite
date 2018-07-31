/*
 *  dalai-suite - ply-uv3
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

    /*! \file   dalai-ply-uv3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - ply-uv3
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

    # ifndef __DL_PLY_UV3__
    # define __DL_PLY_UV3__

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

    /* define header mode */
    # define DL_MODE_NONE     ( 0 )
    # define DL_MODE_PLY      ( 1 )
    # define DL_MODE_FORMAT   ( 2 )
    # define DL_MODE_ELEMENT  ( 3 )
    # define DL_MODE_PROPERTY ( 4 )

    /* define header primitive */
    # define DL_PRIM_NONE     ( 0 )
    # define DL_PRIM_VERTEX   ( 1 )
    # define DL_PRIM_FACE     ( 2 )

    /* define header type */
    # define DL_TYPE_NONE     (  0 )
    # define DL_TYPE_FLOAT    (  1 )
    # define DL_TYPE_DOUBLE   (  2 )
    # define DL_TYPE_CHAR     (  3 )
    # define DL_TYPE_UCHAR    (  4 )
    # define DL_TYPE_SHORT    (  5 )
    # define DL_TYPE_USHORT   (  6 )
    # define DL_TYPE_INT      (  8 )
    # define DL_TYPE_UINT     (  9 )
    # define DL_TYPE_LONG     ( 10 )

    /* define vertex property */
    # define DL_VERTEX_X      ( 0 )
    # define DL_VERTEX_Y      ( 1 )
    # define DL_VERTEX_Z      ( 2 )
    # define DL_VERTEX_R      ( 3 )
    # define DL_VERTEX_G      ( 4 )
    # define DL_VERTEX_B      ( 5 )

    /* define face property */
    # define DL_FACE_LIST     ( 0 )
    # define DL_FACE_VERTEX   ( 1 )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

    class dl_ply_t {

        private:

            std::fstream py_stream;

            le_size_t py_vcount;
            le_size_t py_fcount;
            le_size_t py_offset;
            le_size_t py_vsize;

            le_size_t py_vdata[6];
            le_size_t py_fdata[2];

            le_enum_t py_vtype[6];
            le_enum_t py_ftype[2];

        public:

        /* *** */

        dl_ply_t( le_char_t const * const dl_path );

        /* *** */

        ~dl_ply_t();

        private:

        /* *** */

        le_enum_t dl_ply_type_read( std::string & dl_word );

        /* *** */

        le_size_t dl_ply_type_length( le_enum_t const dl_type );

        /* *** */

        le_void_t dl_ply_header();

        /* *** */

        le_void_t dl_ply_header_vertex( le_enum_t const dl_type, std::string & dl_word );

        /* *** */

        le_void_t dl_ply_header_face( le_enum_t const dl_list, le_enum_t const dl_type );

        /* *** */

        long long int dl_ply_io_integer( le_enum_t const dl_type );

        /* *** */

        le_void_t dl_ply_io_vertex( le_size_t const dl_index, le_byte_t * const dl_buffer );

        public:

        /* *** */

        le_void_t dl_ply_convert( std::fstream & dl_stream );

        private:

        /* *** */

        le_void_t dl_ply_convert_point( std::fstream & dl_stream );

        /* *** */

        le_void_t dl_ply_convert_mesh( std::fstream & dl_stream );

        /* *** */

        double dl_ply_vertex_float( le_byte_t const * const dl_buffer, le_size_t const dl_offset, le_enum_t const dl_vertex );

        /* *** */

        long long int dl_ply_vertex_integer( le_byte_t const * const dl_buffer, le_size_t const dl_offset, le_enum_t const dl_vertex );

    };

/*
    header - function prototypes
 */

    /*! \brief main function (revoked)
     *
     *  The main function reads the provided ply file content and converts it
     *  into a universal format 3 (uf3) file :
     *
     *      ./dalai-ply-uv3 --ply/-i [ply input file]
     *                      --uf3/-o [uf3 output file]
     *
     *  The function starts by creating the class associated to the reading and
     *  analysis of the provided ply file. It then reads the content of the ply
     *  file by chunk and write the spatial and colorimetric elements in the
     *  output stream. The class associated to the ply file is then deleted and
     *  the output stream is closed.
     *
     *  Due to the complexity of the ply format, the main function only accept
     *  little endian binary format. In addition this function, that operates
     *  files through the common library ply module, only considers vertex
     *  elements x, y, z, red, green and blue. All other elements and properties
     *  are not handled.
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

