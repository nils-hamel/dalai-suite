/*
 *  dalai-suite - common library
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

    /*! \file   common.h
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library
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
     *  This standardised format allows to use the suite tools for colour
     *  edition, model cleaning and model hashing. In addition, the standard
     *  format is also expected by the _eratosthene-suite_ implementing the EPFL
     *  CDH DHLAB indexation server and its geographical 3-dimensional data
     *  injection tools.
     *
     *  \section _2 Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2017 EPFL CDH DHLAB
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __LC__
    # define __LC__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <cstdint>

/*
    header - preprocessor definitions
 */

    /* define errors code */
    # define LC_ERROR_NONE      ( 0 )
    # define LC_ERROR_MEMORY    ( 1 )
    # define LC_ERROR_IO_ACCESS ( 2 )
    # define LC_ERROR_IO_READ   ( 3 )
    # define LE_ERROR_IO_WRITE  ( 4 )

    /* define boolean values */
    # define LC_FALSE ( 0 )
    # define LC_TRUE  ( 1 )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

    /* universal format floating point */
    typedef double lc_real_t;

    /* universal format colorimetry */
    typedef uint8_t lc_data_t;

/*
    header - structures
 */

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

