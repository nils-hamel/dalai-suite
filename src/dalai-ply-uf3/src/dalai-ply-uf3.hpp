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

    /*! \struct lc_ply_header_struct
     *  \brief ply header structure
     *
     *  This structure holds the PLY format header information required to read
     *  data from a PLY file. It also holds the file descriptor and the chunk
     *  buffer used for i/o operation.
     *
     *  Currently, only to coordinates and colour components of the PLY vertex
     *  elements are supported for binary reading by this PLY module.
     *
     *  The structures holds, in addition to header properties and i/o related
     *  fields, two small arrays that indicates where to locate the desired
     *  vertex record properties (\b ph_vdata) and their type (\b ph_vtype). The
     *  arrays are created as the PLY header is read, usually as the structure
     *  is created.
     *
     *  \var lc_ply_header_struct::ph_handle
     *  PLY file descriptor
     *  \var lc_ply_header_struct::ph_format
     *  PLY file format (only supports binary_little_endian)
     *  \var lc_ply_header_struct::ph_vertex
     *  PLY vertex elements count
     *  \var lc_ply_header_struct::ph_vsize
     *  Length, in bytes, of vertex records
     *  \var lc_ply_header_struct::ph_vtype
     *  Type of PLY vertex record properties (x,y,z,r,g,b)
     *  \var lc_ply_header_struct::ph_vdata
     *  Offset, in bytes, of the PLY vertex record properties (x,y,z,r,g,b)
     *  \var lc_ply_header_struct::ph_position
     *  I/O position, in bytes, in the PLY file
     *  \var lc_ply_header_struct::ph_chunk
     *  PLY vertex records chunk buffer
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

    /*! \brief constructor/destructor methods
     *
     *  This function creates and returns the PLY header structure related to
     *  the provided input PLY file path.
     *
     *  It creates the PLY file descriptor and initialise the chunk memory
     *  buffer. It also reads the PLY header, using the \b lc_ply_io_i_header()
     *  function, and initialise the structure fields according to the header
     *  content.
     *
     *  This function returning the created structure, the status is stored in
     *  the structure itself using the reserved \b _status field (\b LC_TRUE
     *  on success, \b LC_FALSE otherwise).
     *
     *  \param  lc_path Path of the input PLY file
     *
     *  \return Returns created PLY structure
     */


    /*! \brief constructor/destructor methods
     *
     *  This function deletes the provided PLY structure and resets its fields.
     *  It closes the PLY file descriptor and releases the memory associated to
     *  the chunk buffer.
     *
     *  \param lc_ply PLY structure
     */

    /*! \brief accessor methods
     *
     *  This function returns the x coordinates of the vertex stored at index
     *  \b lc_index in the PLY structure chunk buffer.
     *
     *  This function does not check the state of the chunk buffer or the amount
     *  of vertex record it contains before to access the data.
     *
     *  \param  lc_ply   PLY structure
     *  \param  lc_index Index of the vertex record in the chunk buffer
     *
     *  \return Returns vertex x coordinates
     */

    /*! \brief accessor methods
     *
     *  This function returns the y coordinates of the vertex stored at index
     *  \b lc_index in the PLY structure chunk buffer.
     *
     *  This function does not check the state of the chunk buffer or the amount
     *  of vertex record it contains before to access the data.
     *
     *  \param  lc_ply   PLY structure
     *  \param  lc_index Index of the vertex record in the chunk buffer
     *
     *  \return Returns vertex y coordinates
     */

    /*! \brief accessor methods
     *
     *  This function returns the z coordinates of the vertex stored at index
     *  \b lc_index in the PLY structure chunk buffer.
     *
     *  This function does not check the state of the chunk buffer or the amount
     *  of vertex record it contains before to access the data.
     *
     *  \param  lc_ply   PLY structure
     *  \param  lc_index Index of the vertex record in the chunk buffer
     *
     *  \return Returns vertex z coordinates
     */

    /*! \brief accessor methods
     *
     *  This function returns the colour red component of the vertex stored at
     *  index \b lc_index in the PLY structure chunk buffer.
     *
     *  This function does not check the state of the chunk buffer or the amount
     *  of vertex record it contains before to access the data.
     *
     *  \param  lc_ply   PLY structure
     *  \param  lc_index Index of the vertex record in the chunk buffer
     *
     *  \return Returns vertex color red component
     */

    /*! \brief accessor methods
     *
     *  This function returns the colour green component of the vertex stored at
     *  index \b lc_index in the PLY structure chunk buffer.
     *
     *  This function does not check the state of the chunk buffer or the amount
     *  of vertex record it contains before to access the data.
     *
     *  \param  lc_ply   PLY structure
     *  \param  lc_index Index of the vertex record in the chunk buffer
     *
     *  \return Returns vertex color green component
     */

    /*! \brief accessor methods
     *
     *  This function returns the colour blue component of the vertex stored at
     *  index \b lc_index in the PLY structure chunk buffer.
     *
     *  This function does not check the state of the chunk buffer or the amount
     *  of vertex record it contains before to access the data.
     *
     *  \param  lc_ply   PLY structure
     *  \param  lc_index Index of the vertex record in the chunk buffer
     *
     *  \return Returns vertex color blue component
     */

    /*! \brief i/o methods
     *
     *  This function reads the PLY file header and initialise the PLY structure
     *  fields according to the header content.
     *
     *  This function expects a PLY structure containing a valid file descriptor
     *  with position in the beginning of the file.
     *
     *  In addition to header parameters analysis, this function builds the
     *  structure arrays used to access vertex record properties. It can handle
     *  any valid PLY types, according to the documentation, but only fills the
     *  arrays for the coordinates and colour components properties.
     *
     *  \param  lc_ply PLY structure
     *
     *  \return Returns \b LC_TRUE on success, \b LC_FALSE otherwise
     */

    /*! \brief i/o methods
     *
     *  This function reads \b lc_size vertex records from the PLY file pointed
     *  by the provided PLY structure and stores them in the structure chunk
     *  buffer. Data accessor method can then be used to access data stored in
     *  the filled chunk buffer.
     *
     *  This function expects an already created PLY structure containing a
     *  valid file descriptor with an i/o position just after the last byte of
     *  the PLY header, as left by the \b lc_ply_io_i_header() function, or a
     *  i/o position pointing the first byte of a vertex record.
     *
     *  \param lc_ply  PLY structure
     *  \param lc_size Number of vertex record to read
     *
     *  \return Returns the number of read vertex records
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

