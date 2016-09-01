/*
 *  dalai-suite - geodetic system
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016 EPFL CDH DHLAB
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

    /*! \file   dalai-ply-universal.hpp
     *  \author Nils Hamel <n.hamel@bluewin.ch>
     *
     *  dalai-suite - polygone file format (PLY, stanford) to universal format
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
     *  Copyright (c) 2016 EPFL CDH DHLAB
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_PLY_UNIVERSAL__
    # define __DL_PLY_UNIVERSAL__

/*
    header - includes
 */

    # include <iostream>
    # include <fstream>
    # include <common-args.h>
    # include <pcl/point_cloud.h>
    # include <pcl/point_types.h>
    # include <pcl/io/ply_io.h>

/*
    header - preprocessor definitions
 */

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
     *  The main function reads the provided input PLY file content and exports
     *  it in the output stream. Both coordinates and colors are exported in the
     *  output file.
     *
     *  \param argc Standard parameter
     *  \param argv Standard parameter
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

