/*
 *  dalai-suite - color
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

    /*! \file   dalai-color.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - color
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
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define chunk size */
    //# define DL_CHUNK ( 2396745 )

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

    /*! \brief color mapping methods (revoked)
     *
     *  This function assigns a color to the provided color vector based on the
     *  provided height value. It starts by clamping the height value using the
     *  boundary values provided as parameters. The heights is then normalised
     *  on this interval and the color is computed following the implemented
     *  colormap.
     *
     *  \param dl_height Point height
     *  \param dl_data   Point color array pointer (unsigned 8-bits, RGB)
     *  \param dl_ledge  Height clamping range lower boundary
     *  \param dl_hedge  Height clamping range upper boundary
     */

    le_void_t dl_color( le_real_t dl_height, le_data_t * const dl_data, le_real_t const dl_ledge, le_real_t const dl_hedge );

    /*! \brief main methods (revoked)
     *
     *  The main function reads the elements provided through the uv3 input file
     *  and overrides their color using a colormap based on element height. The
     *  corrected element are exported in the output uv3 file : 
     *
     *      ./dalai-color --input/-i [input file]
     *                    --output/-o [output file]
     *                    --minimum/-m [colormap low boundary]
     *                    --maximum/-x [colormap high boundary]
     *
     *  The element height is understood as the value of their third coordinate
     *  assumed to corresponds to natural model altitude. If the third component
     *  is outside of the provided range, a cyclic condition is considered.
     *
     *  \param  argc Standard parameter
     *  \param  argv Standard parameter
     *
     *  \return Return standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

