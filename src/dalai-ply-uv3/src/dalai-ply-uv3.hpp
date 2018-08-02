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
     *  \section overview Overview
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
     *  \section copyright Copyright and License
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

    /*! \class dl_ply_t
     *  \brief PLY reader class
     *
     *  This structures holds the required information and methods to reads the
     *  content of a ply file in order to converts it into a uv3 file.
     *
     *  The main members are the 'data' arrays and 'type' arrays for vertex and
     *  faces that indicates at which offset each of their property can be read
     *  and with which type they are encoded.
     *
     *  \var dl_ply_t::py_stream
     *  Stream descriptor to ply file
     *  \var dl_ply_t::py_vcount
     *  Amount of vertex
     *  \var dl_ply_t::py_fcount
     *  Amount of faces
     *  \var dl_ply_t::py_offset
     *  Offset, in bytes, of the first byte outside of the header
     *  \var dl_ply_t::py_vsize
     *  Size, in bytes, of a vertex record
     *  \var dl_ply_t::py_vdata
     *  Offset in record, in bytes, of each vertex property
     *  \var dl_ply_t::py_fdata
     *  Offset in record, in bytes, of each face property
     *  \var dl_ply_t::py_vtype
     *  Types of the vertex property
     *  \var dl_ply_t::py_ftype
     *  Types of the faces property
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

        /*! \brief constructor/destructor methods
         *
         *  The constructor starts by initialising the ply format descriptor
         *  arrays for both face and vertex elements. It then creates the stream
         *  descriptor towards the ply file.
         *
         *  The constructor ends by reading the header of the ply stream using
         *  the \b dl_ply_header() method.
         *
         *  \param dl_path Path to the ply stream
         */

        dl_ply_t( le_char_t const * const dl_path );

        /*! \brief constructor/destructor methods
         *
         *  The destructor simply deletes the stream descriptor toward the ply
         *  file.
         */

        ~dl_ply_t();

        private:

        /*! \brief type methods
         *
         *  This function converts the provided type string into a code that is
         *  returned.
         *
         *  \param dl_word Type string
         */

        le_enum_t dl_ply_type_read( std::string & dl_word );

        /*! \brief type methods
         *
         *  This function returns the length, in bytes, of the provided type.
         *
         *  \param dl_type Type constant
         *
         *  \return Returns the type length, in bytes
         */

        le_size_t dl_ply_type_length( le_enum_t const dl_type );

        /*! \brief header methods
         *
         *  This method is used to analyse the header of the ply file. It has
         *  to be called before any conversion process.
         *
         *  The function checks if the provided ply file is in a readable format
         *  and starts analysing the content in vertex and faces. For both
         *  vertex and faces, the function checks the specification format.
         *
         *  The objets members are updated according to the results of the ply
         *  header analysis.
         */

        le_void_t dl_ply_header();

        /*! \brief header methods
         *
         *  This header complementary function reads the vertex property and
         *  assign the corresponding representation to the object.
         *
         *  \param dl_type Type of the property
         *  \param dl_word Property name string
         */

        le_void_t dl_ply_header_vertex( le_enum_t const dl_type, std::string & dl_word );

        /*! \brief header methods
         *
         *  This header complementary function reads the face property and
         *  assign the corresponding representation to the object.
         *
         *  \param dl_list Type of the list size
         *  \param dl_type Type of list index
         */

        le_void_t dl_ply_header_face( le_enum_t const dl_list, le_enum_t const dl_type );

        /*! \brief i/o methods
         *
         *  This function is used to read, at the current ply stream offset, a
         *  specific integer type. The read value is returned by the function.
         *
         *  \param dl_type Type to read
         *
         *  \return Returns read type value
         */

        long long int dl_ply_io_integer( le_enum_t const dl_type );

        /*! \brief i/o methods
         *
         *  This function is used to read one record corresponding to a specific
         *  vertex in the ply stream. The vertex record is read, according to
         *  its length and is returned through the provided buffer.
         *
         *  \param dl_index  Index, zero-based, of the vertex
         *  \param dl_buffer Byte buffer receiving the read vertex record
         */

        le_void_t dl_ply_io_vertex( le_size_t const dl_index, le_byte_t * const dl_buffer );

        public:

        /*! \brief conversion methods
         *
         *  This function is the main conversion method. Its role is simply to
         *  invoke specialised conversion process according to the amount of
         *  faces found in the ply stream.
         *
         *  If only vertex are provided through the ply stream, the function
         *  performs a point-only conversion. If at least one face is found in
         *  the ply stream, a pure mesh conversion is then made.
         *
         *  \param dl_stream Conversion output stream descriptor
         */

        le_void_t dl_ply_convert( std::fstream & dl_stream );

        private:

        /*! \brief conversion methods
         *
         *  This method is specialised in point-based conversion from ply to uv3
         *  format. It reads ply vertex by chunks and converts them into uv3
         *  chunks before to exports them in the provided output stream.
         *
         *  \param dl_stream Conversion output stream descriptor
         */

        le_void_t dl_ply_convert_point( std::fstream & dl_stream );

        /*! \brief conversion methods
         *
         *  This method is specialised in mesh-based conversion from ply to uv3
         *  format. It reads each face index list and converts then into uv3
         *  polygonal record before to export them in the specified output
         *  stream.
         *
         *  \param dl_stream Conversion output stream descriptor
         */

        le_void_t dl_ply_convert_mesh( std::fstream & dl_stream );

        /*! \brief conversion methods
         *
         *  This function is used to read the vertex property as a floating
         *  value from its storage type.
         *
         *  \param dl_buffer Vertex record buffer
         *  \param dl_offset Vertex offset in the buffer
         *  \param dl_vertex Property to read
         *  
         *  \return Returns read property as double
         */

        double dl_ply_vertex_float( le_byte_t const * const dl_buffer, le_size_t const dl_offset, le_enum_t const dl_vertex );

        /*! \brief conversion methods
         *
         *  This function is used to read the vertex property as an integer
         *  value from its storage type.
         *
         *  \param dl_buffer Vertex record buffer
         *  \param dl_offset Vertex offset in the buffer
         *  \param dl_vertex Property to read
         *  
         *  \return Returns read property as integer
         */

        long long int dl_ply_vertex_integer( le_byte_t const * const dl_buffer, le_size_t const dl_offset, le_enum_t const dl_vertex );

    };

/*
    header - function prototypes
 */

    /*! \brief main function
     *
     *  The main function reads the provided ply stream content and converts it
     *  into uv3 stream : 
     *
     *      ./dalai-ply-uv3 --ply/-i [ply input file path]
     *                      --uv3/-o [uv3 output file path]
     *
     *  The function starts by creating the class associated to the reading and
     *  analysis of the provided ply file. It then reads the content of the ply
     *  file and converts it into uv3 format.
     *
     *  If the ply file contains only vertex, a specialised and fast function
     *  is used to converts the vertex into uv3 records. If the ply stream
     *  contains at least one face, only line and triangles are converted and
     *  exported in the output u3 stream.
     *
     *  The converted ply file is always considered in its binary representation
     *  and always considering little-endian byte order.
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

