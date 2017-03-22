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

    # ifndef __DL_ply_UF3__
    # define __DL_ply_UF3__

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

    /*! \class dl_header_t
     *  \brief ply format header
     *
     *  This class mainly holds the information on the pointed ply file that was
     *  read from the ply file header. In addition, it also maintain stream
     *  descriptor that allows its member functions to perform i/o operations on
     *  the handled ply file.
     *
     *  Currently, this structure is only designed to import vertex element with
     *  their space coordinates and their colorimetry. Its usage is then reduced
     *  to simple point clouds storage format conversion.
     *
     *  \var dl_header_t::hd_stream
     *  ply file stream descriptor
     *  \var dl_header_t::hd_vertex
     *  Number of vertex
     *  \var dl_header_t::hd_position
     *  ply file first data byte offset
     *  \var dl_header_t::hd_size
     *  Length, in bytes, of vertex records
     *  \var dl_header_t::hd_data
     *  Offsets, in bytes, of the vertex properties position in records
     *  \var dl_header_t::hd_chunk
     *  ply records chunk buffer
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

        /*! \brief constructor
         *
         *  The constructor opens the provided ply file after having initialised
         *  the class members. It then reads and analyses the content of the ply
         *  header to build the memory structures required for i/o operations on
         *  ply file data.
         *
         *  \param dl_path ply file path
         */

        dl_header_t( char const * const dl_path );

        /*! \brief destructor
         *
         *  The destructor release the memory structures built during class
         *  creation and close the stream on the pointed ply file.
         */

        ~dl_header_t();

    private:

        /*! \brief detection methods
         *
         *  This method converts the provided string token in its corresponding
         *  type constant.
         *
         *  \param  dl_token Token string
         *
         *  \return Type constant corresponding to the provided token string
         */

        int hd_det_type( char const * const dl_token );

        /*! \brief detection methods
         *
         *  This function returns, in bytes, the size of the type corresponding
         *  to the provided type constant.
         *
         *  \param  dl_type Type constant
         *
         *  \return Returns the size, in bytes, of the corresponding type
         */
        int hd_det_size( int dl_type );

    public:

        /*! \brief accessor methods
         *
         *  This function returns the x-coordinate of the point cloud element
         *  found at record index \b lc_index. The index is related to the
         *  i/o chunk buffer.
         *
         *  \param  lc_index Record chunk buffer index
         *
         *  \return Returns element x-coordinate
         */

        double hd_get_x( long long int lc_index );

        /*! \brief accessor methods
         *
         *  This function returns the y-coordinate of the point cloud element
         *  found at record index \b lc_index. The index is related to the
         *  i/o chunk buffer.
         *
         *  \param  lc_index Record chunk buffer index
         *
         *  \return Returns element y-coordinate
         */

        double hd_get_y( long long int lc_index );

        /*! \brief accessor methods
         *
         *  This function returns the z-coordinate of the point cloud element
         *  found at record index \b lc_index. The index is related to the
         *  i/o chunk buffer.
         *
         *  \param  lc_index Record chunk buffer index
         *
         *  \return Returns element z-coordinate
         */

        double hd_get_z( long long int lc_index );

        /*! \brief accessor methods
         *
         *  This function returns the red component of the point cloud element
         *  found at record index \b lc_index. The index is related to the
         *  i/o chunk buffer.
         *
         *  \param  lc_index Record chunk buffer index
         *
         *  \return Returns element red component
         */

        char hd_get_red( long long int lc_index );

        /*! \brief accessor methods
         *
         *  This function returns the green component of the point cloud element
         *  found at record index \b lc_index. The index is related to the
         *  i/o chunk buffer.
         *
         *  \param  lc_index Record chunk buffer index
         *
         *  \return Returns element green component
         */

        char hd_get_green( long long int lc_index );

        /*! \brief accessor methods
         *
         *  This function returns the blue component of the point cloud element
         *  found at record index \b lc_index. The index is related to the
         *  i/o chunk buffer.
         *
         *  \param  lc_index Record chunk buffer index
         *
         *  \return Returns element blue component
         */

        char hd_get_blue( long long int lc_index );

    private:

        /*! \brief mutator methods
         *
         *  This function is used to build the ply header structure hold in the
         *  class. It allows, according to the provided token, to store the
         *  record offset of the handled vertex property.
         *
         *  As explained in the class documentation, only the vertex coordinates
         *  and color components are actually handled by the class.
         *
         *  \param dl_token  Vertex property token
         *  \param dl_offset Record offset of the property
         */

        void hd_set_data( char const * const dl_token, long long int const dl_offset );

    public:

        /*! \brief mutator methods
         *
         *  This function is used to read the next chunk of the ply file opened
         *  by the class. It reads a chunk that contains \b dl_size records and
         *  stores it in the class i/o chunk buffer.
         *
         *  The information on vertex are only available through the class
         *  methods after at least one call to this function. Moreover, the
         *  amount of read record can be less than \b dl_size as the function
         *  tries to read the last ply file chunk.
         *
         *  Chunks are used to keep control on the memory used by the class in
         *  case very large ply files are considered.
         *
         *  \param  dl_size Number of record to read
         *
         *  \return Returns the actual number of read records
         */

        long long int hd_set_chunk( long long int const dl_size );

    };

/*
    header - function prototypes
 */

    /*! \brief main function
     *
     *  The main function reads the provided ply file content and converts it
     *  into a universal format 3 (uf3) file :
     *
     *      ./dalai-ply-uf3 --ply/-i [ply input file]
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

