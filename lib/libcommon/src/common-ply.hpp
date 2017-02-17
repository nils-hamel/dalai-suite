/*
 *  dalai-suite - common library
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

    /*! \file   common-ply.h
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - ply module
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_PLY__
    # define __LC_PLY__

/*
    header - internal includes
 */

    # include "common.hpp"

/*
    header - external includes
 */

    # include <cstdio>
    # include <cstdlib>
    # include <cstring>

/*
    header - preprocessor definitions
 */

    /* define pseudo-constructor */
    # define LC_PLY_C           { NULL, 0, 0, 0, { 0 }, { 0 }, 0, NULL, LC_TRUE }

    /* define ply format */
    # define LC_PLY_ASCII       ( 1 )
    # define LC_PLY_BINARY_LE   ( 2 )
    # define LC_PLY_BINARY_BE   ( 3 )

    /* define header reading modes */
    # define LC_PLY_RM_VALIDATE ( 0 )
    # define LC_PLY_RM_DETECT   ( 1 )
    # define LC_PLY_RM_STANDARD ( 2 )
    # define LC_PLY_RM_FORMAT   ( 3 )
    # define LC_PLY_RM_ELEMENT  ( 4 )
    # define LC_PLY_RM_VERTEX   ( 5 )
    # define LC_PLY_RM_PROPERTY ( 6 )
    # define LC_PLY_RM_DATA     ( 7 )

    /* define data types */
    # define LC_PLY_TP_NONE     ( 0  )
    # define LC_PLY_TP_DOUBLE   ( 1  )
    # define LC_PLY_TP_FLOAT    ( 2  )
    # define LC_PLY_TP_UCHAR    ( 3  )
    # define LC_PLY_TP_CHAR     ( 4  )
    # define LC_PLY_TP_USHORT   ( 5  )
    # define LC_PLY_TP_SHORT    ( 6  )
    # define LC_PLY_TP_UINT     ( 7  )
    # define LC_PLY_TP_INT      ( 8  )
    # define LC_PLY_TP_ULONG    ( 9  )
    # define LC_PLY_TP_LONG     ( 10 )

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

    typedef struct lc_ply_header_struct {

        FILE *        ph_handle;

        int           ph_format;
        long long int ph_vertex;

        int           ph_vsize;
        int           ph_vtype[6];
        int           ph_vdata[6];

        long long int ph_position;

        char *        ph_chunk;

    int _status; } lc_ply_header_t, lc_ply_t;

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

    lc_ply_t lc_ply_create( unsigned char const * const lc_path );

    /*! \brief constructor/destructor methods
     *
     *  This function deletes the provided PLY structure and resets its fields.
     *  It closes the PLY file descriptor and releases the memory associated to
     *  the chunk buffer.
     *
     *  \param lc_ply PLY structure
     */

    void lc_ply_delete( lc_ply_t * const lc_ply );

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

    double lc_ply_get_x( lc_ply_t const * const lc_ply, long long int lc_index );

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

    double lc_ply_get_y( lc_ply_t const * const lc_ply, long long int lc_index );

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

    double lc_ply_get_z( lc_ply_t const * const lc_ply, long long int lc_index );

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

    char lc_ply_get_red( lc_ply_t const * const lc_ply, long long int lc_index );

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

    char lc_ply_get_green( lc_ply_t const * const lc_ply, long long int lc_index );

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

    char lc_ply_get_blue( lc_ply_t const * const lc_ply, long long int lc_index );

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

    int lc_ply_io_i_header( lc_ply_t * const lc_ply );

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

    long long int lc_ply_io_i_chunk( lc_ply_t * const lc_ply, long long int const lc_size );

/*
    header - inclusion guard
 */

    # endif

