/*
 *  dalai-suite - hash
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

    /*! \file   dalai-hash.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - hash
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

    # ifndef __DL_HASH__
    # define __DL_HASH__

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

    /*! \brief main function
     *
     *  The main function hashes the provided uv3 stream into a set of smaller
     *  sub-stream stored in the output directory :
     *
     *      ./dalai-hash --uv3/-i [input uv3 file path]
     *                   --output/-o [output path directory]
     *                   --count/-c [sampled element count]
     *                   --param/-p [hashing parameter]
     *
     *  The functions starts by gathering the parameters and opens the provided
     *  input stream. It computes the stream minimum distance mean value and
     *  hashes it using the \b lc_hash() function. See the documentation of
     *  \b libcommon for more information. The hashed sub-stream are written in
     *  the provided output directory.
     *
     *  The count value gives the amount of stream record to consider to compute
     *  the minimum distance mean value. The hashing parameter is used with the
     *  mean value to determine the size of the sub-stream.
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

