/*
 *  dalai-suite - uv3-ply
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
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
     *  \section overview Overview
     *
     *  The _dalai-suite_ offers a toolbox for large scale 3D models
     *  manipulation, processing, conversion and geographical registration. It
     *  is mainly designed to offers solutions suitable for models that can
     *  weight hundreds of gigabytes and that are used in the domain of
     *  environment survey and land registering. It was initially developed to
     *  offer the required tools needed to allows massive data processing and
     *  preparation for the _Eratosthene Project_.
     *
     *  The _dalai-suite_ uses a common and very simple format for all the
     *  implemented tools that is also used for the _Eratosthene Project_. This
     *  format offers a simple way of storing massive amount of graphical
     *  primitives. This allows the _dalai-suite_ to manipulate all sort of 3D
     *  models, from large scale point-based models to more refined vector
     *  models.
     *
     *  The _dalai-suite_ comes also with tools dedicated to models
     *  visualization and large scale models processing such as automatic
     *  segmentation, cleaning, geographical transformation and assisted
     *  geographical registration.
     *
     *  The _dalai-suite_ also comes with a set of tools dedicated to file
     *  format conversion that allows data coming in their specific formats to
     *  be converted in the _suite_ format.
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2020 DHLAB, EPFL
     *
     *  This program is licensed under the terms of the GNU GPLv3. Documentation
     *  and illustrations are licensed under the terms of the CC BY 4.0.
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
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define ply record */
    # define DL_PLY_VERTEX ( ( sizeof( float ) + sizeof( uint8_t ) ) * 3 )

    /* define ply record */
    # define DL_PLY_LINE   ( sizeof( uint8_t ) + sizeof( int32_t ) * 2 )

    /* define ply record */
    # define DL_PLY_FACE   ( sizeof( uint8_t ) + sizeof( int32_t ) * 3 )

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

    /*! \brief header methods
     *
     *  This function exports the ply header in the provided output stream
     *  according to the amount of vertex and faces that have to be written
     *  in the ply stream.
     *
     *  \param dl_stream PLY stream descriptor
     *  \param dl_vertex Vertex count
     *  \param dl_face   Face count
     */

    le_void_t dl_uv3_ply_header( std::fstream & dl_stream, le_size_t const dl_vertex, le_size_t const dl_face );

    /*! \brief statistical methods
     *
     *  This function reads the provided input streams and determine the amount
     *  of points, lines and triangles. The function returns the total amount of
     *  lines and triangles all considered as faces.
     *
     *  \param dl_stream UV3 stream descriptor
     *
     *  \return Returns total amount of faces in the uv3 stream
     */

    le_size_t dl_uv3_ply_primitive( std::fstream & dl_stream );

    /*! \brief conversion methods
     *
     *  This function reads each record of the provided uv3 stream and export
     *  its coordinates as a vertex in the provided ply stream.
     *
     *  \param dl_istream UV3 stream descriptor
     *  \param dl_ostream PLY stream descriptor
     */

    le_void_t dl_uv3_ply_vertex( std::fstream & dl_istream, std::fstream & dl_ostream );

    /*! \brief conversion methods
     *
     *  This function detects the lines and triangles present in the provided
     *  uv3 stream and exports their definition in the provided output ply
     *  stream as faces.
     *
     *  \param dl_istream UV3 stream descriptor
     *  \param dl_ostream PLY stream descriptor
     */

    le_void_t dl_uv3_ply_face( std::fstream & dl_istream, std::fstream & dl_ostream );

    /*! \brief main methods
     *
     *  The main function reads the provided uv3 file and converts it into a
     *  ply file :
     *
     *      ./dalai-uv3-ply --input/-i [uv3 input file]
     *                      --output/-o [ply output file]
     *                      --vertex-only,-v [conversion switch]
     *
     *  The function starts by allocating the required i/o buffers memory and
     *  creating the i/o streams. It then creates and exports the output ply
     *  file header.
     *
     *  The function then starts by converting the vertex in the output ply
     *  file before to export the faces, if present. If the 'vertex-only' switch
     *  is specified, the main function only exports the vertex, i.e. the
     *  points, in the output ply file, discarding faces.
     *
     *  The created ply file is always considered in its binary representation
     *  and always considering little-endian byte order.
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

