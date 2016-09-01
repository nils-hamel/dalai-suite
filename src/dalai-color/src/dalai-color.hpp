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

    /*! \file   dalai-color.hpp
     *  \author Nils Hamel <n.hamel@bluewin.ch>
     *
     *  Dalai-suite - Polygon File Format to Universal Format converter
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

    # ifndef __DL_COLOR__
    # define __DL_COLOR__

/*
    header - includes
 */

    # include <iostream>
    # include <fstream>
    # include <cstdint>
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
     *  This function assign a color to the provided color array based on the
     *  provided height value. It starts by clamping the height value using the
     *  boundary value provided as parameters. The heights is the normalised on
     *  this intervalle and the color is computed following the implemented
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
     *  The main function reads the arguments and parameters and opens the input
     *  points cloud in memory. Parsing the cloud points, it assign to points a
     *  color provided by a color map based on points height (third component).
     *  The modified points cloud is the exported in the output stream.
     *
     *  \param argc Standard parameter
     *  \param argv Standard parameter
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

