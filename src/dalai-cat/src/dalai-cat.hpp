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

    /*! \brief address methods (revoked)
     *
     *  This function is used to display an address structure spatial index as
     *  a sequence of digits in base eight on the standard output. The provided
     *  address structure is expected to contain a valid index consistent with
     *  its size.
     *
     *  \param dl_addr Address structure
     */

    le_void_t dl_cat_address( le_real_t * const dl_pose, le_byte_t const dl_length );

    /*! \brief main methods (revoked)
     *
     *  The main function reads the primitives stored in the provided uv3 file 
     *  and displays its content on the standard output :
     *
     *      ./dalai-cat --uv3/-i [uv3 input file] --index/-x [size]
     *
     *  The main function opens the provided file and reads its element one by
     *  one. For each element, the main function displays a line on the standard
     *  output containing the element coordinates, type and color.
     *
     *  If the '--index' parameter is provided to a non-zero value, the function
     *  displays the elements coordinates in terms of eratosthene index instead
     *  of geographical coordinates. The elements provided in the uv3 file are
     *  then expected to be provided in the WGS84 ellipsoidal frame.
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

