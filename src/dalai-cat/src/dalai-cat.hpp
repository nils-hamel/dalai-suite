/*
 *  dalai-suite - cat
 *
 *      Nils Hamel - nils.hamel@alumni.epfl.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
 *      Copyright (c) 2020 Republic and Canton of Geneva
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
     *  address structure before to display its spatial index. The provided
     *  length gives the number of digits to consider for the spatial index.
     *
     *  \param dl_pose   Position vector
     *  \param dl_length Spatial index length (digit count)
     */

    le_void_t dl_cat_address( le_real_t * const dl_pose, le_byte_t const dl_length );

    /*! \brief main methods
     *
     *  The main function reads the primitives stored in the provided uv3 file
     *  and displays them on the standard output :
     *
     *      ./dalai-cat --input/-i [uv3 input file]
     *                  --index/-x [index size]
     *
     *  The main function reads the provided file and imports the primitives
     *  records one by one before to display them on the standard output. The
     *  primitives vertex coordinates, type and color are displayed.
     *
     *  If the provided '--index' is non-zero, the vertex coordinates of each
     *  primitive is replaced by the eratosthene address spatial index using
     *  the value as spatial index length. In such a case, the provided model
     *  is expected to be aligned in the WGS84 coordinate system, with heights
     *  above the frame ellipsoid.
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

