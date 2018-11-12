/*
 *  dalai-suite - cat
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

    /*! \file   dalai-cat.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - cat
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

    # ifndef __DL_CAT__
    # define __DL_CAT__

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

    /*! \brief address methods
     *
     *  This function converts the provided position vector into an eratosthene
     *  address structure before to display the spatial index of the computed
     *  address. The provided length gives the number of digit to consider for
     *  the spatial index.
     *
     *  \param dl_pose   Position vector
     *  \param dl_length Spatial index length - in digit count
     */

    le_void_t dl_cat_address( le_real_t * const dl_pose, le_byte_t const dl_length );

    /*! \brief main methods
     *
     *  The main function reads the primitives stored in the provided uv3 file
     *  and displays them on the standard output :
     *
     *      ./dalai-cat --uv3/-i [uv3 input file]
     *                  --index/-x [index size]
     *
     *  The main function reads the provided file and imports the element
     *  records one by one before to display them on the standard output. The
     *  element vertex coordinates, type and color are considered for output.
     *
     *  If the provided '--index' is non-zero, the vertex coordinates of each
     *  element is replaced by the eratosthene address spatial index using
     *  the value as spatial index length. In such a case, the provided model
     *  is expected to be aligned in the WGS84 coordinate system, with heights
     *  over the frame ellipsoid.
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

