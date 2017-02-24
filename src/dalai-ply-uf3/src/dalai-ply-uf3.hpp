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
     *  dalai-suite - ply-uf3
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
     *  This standardised format allows to use the suite tools for colour
     *  edition, model cleaning and model hashing. In addition, the standard
     *  format is also expected by the _eratosthene-suite_ implementing the EPFL
     *  CDH DHLAB indexation server and its geographical 3-dimensional data
     *  injection tools.
     *
     *  \section _2 Copyright and License
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
    # include <cstring>
    # include <common-include.hpp>

/*
    header - preprocessor definitions
 */

    /* define header reading modes */
    # define LC_PLY_VALIDATE ( 0 )
    # define LC_PLY_DETECT   ( 1 )
    # define LC_PLY_STANDARD ( 2 )
    # define LC_PLY_FORMAT   ( 3 )
    # define LC_PLY_ELEMENT  ( 4 )
    # define LC_PLY_VERTEX   ( 5 )
    # define LC_PLY_PROPERTY ( 6 )
    # define LC_PLY_DATA     ( 7 )

    /* define data types */
    # define LC_PLY_NONE     ( 0  )
    # define LC_PLY_DOUBLE   ( 1  )
    # define LC_PLY_FLOAT    ( 2  )
    # define LC_PLY_UCHAR    ( 3  )
    # define LC_PLY_CHAR     ( 4  )
    # define LC_PLY_USHORT   ( 5  )
    # define LC_PLY_SHORT    ( 6  )
    # define LC_PLY_UINT     ( 7  )
    # define LC_PLY_INT      ( 8  )
    # define LC_PLY_ULONG    ( 9  )
    # define LC_PLY_LONG     ( 10 )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

    class dl_header_t {

    private:
        std::ifstream hd_stream;
        long long int hd_vertex;
        long long int hd_position;

        long long int hd_size;
        long long int hd_data[6];

        char *        hd_chunk;

    public:
        dl_header_t( char const * const dl_path );
        ~dl_header_t();

    private:
        int hd_det_type( char const * const dl_token );
        int hd_det_size( int dl_type );

    public:
        double hd_get_x( long long int lc_index );
        double hd_get_y( long long int lc_index );
        double hd_get_z( long long int lc_index );
        char hd_get_red( long long int lc_index );
        char hd_get_green( long long int lc_index );
        char hd_get_blue( long long int lc_index );

    private:
        void hd_set_data( char const * const dl_token, long long int const dl_offset );
    public:
        long long int hd_set_chunk( long long int const dl_size );

    };

/*
    header - function prototypes
 */

    /*! \brief main function
     *
     *  The main function reads the provided ply file content and converts it
     *  into a universal format file :
     *
     *      ./dalai-ply-uf3 --ply/-i [ply input file]
     *                      --uf3/-o [uf3 output file]
     *
     *  The function starts by allocating i/o buffer memory and opens streams
     *  toward input and output file. It the reads the content of the input
     *  file by chunk. The chunks are converted into universal format before to
     *  be exported in the output file. The function ends by closing the streams
     *  and releasing the allocated memory.
     *
     *  Due to the complexity of the ply format, the main function only accept
     *  little endian binary format. In addition this function, that operates
     *  files through the common library ply module, only considers vertex
     *  elements x, y, z, red, green and blue. All other element and properties
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

