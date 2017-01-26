/*
 *  dalai-suite - color
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

    /*! \file   dalai-color.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - color
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
     *  Copyright (c) 2016-2017 EPFL CDH DHLAB
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_COLOR__
    # define __DL_COLOR__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <cmath>
    # include <common-args.h>

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

    /*! \brief color mapping methods
     *
     *  This function assigns a colour to the provided color array based on the
     *  provided height value. It starts by clamping the height value using the
     *  boundary values provided as parameters. The heights is then normalised
     *  on this interval and the colour is computed following the implemented
     *  colormap.
     *
     *  \param dl_height Point height
     *  \param dl_data   Point color array pointer (unsigned 8-bits, RGB)
     *  \param dl_ledge  Height clamping range lower boundary
     *  \param dl_hedge  Height clamping range upper boundary
     */

    void dl_color( double dl_height, uint8_t * const dl_data, double const dl_ledge, double const dl_hedge );

    /*! \brief main methods
     *
     *  The main function reads the point cloud provided through the input file
     *  and overrides the colours using a colormap based on heights. The result
     *  is exported in the output file :
     *
     *      ./dalai-color --input/-i [input file]
     *                    --output/-o [output file]
     *                    --minimum/-m [colormap low height]
     *                    --maximum/-x [colormap high height]
     *                    --thread/-t [number of thread]
     *
     *  The point cloud third coordinates are used to access the colormap and
     *  assign new colours. If a third coordinates is outside of the provided
     *  height range, a cyclic condition is considered.
     *
     *  \param argc Standard parameter
     *  \param argv Standard parameter
     *
     *  \return Return standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

