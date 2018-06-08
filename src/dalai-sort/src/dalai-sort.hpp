/*
 *  dalai-suite - sort
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

    /*! \file   dalai-sort.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - sort
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

    # ifndef __DL_SORT__
    # define __DL_SORT__

/*
    header - internal includes
 */

    # include "dalai-sort-algorithm.hpp"

/*
    header - external includes
 */

    # include <common-include.hpp>
    # include <eratosthene-include.h>
    # include <cstdio>
    # include <fstream>
    # include <sys/stat.h>

/*
    header - preprocessor definitions
 */

/*
    header - preprocessor macros
 */

    /* define path composition pattern */
    # define DL_SORT_ORIGIN "%s/c-%" _LE_SIZE_P ".tmp"
    # define DL_SORT_TARGET "%s/d-%" _LE_SIZE_P ".tmp"

/*
    header - type definition
 */

/*
    header - structures
 */

/*
    header - function prototypes
 */

    /* *** */

    le_size_t dl_sort_filesize( le_char_t const * const dl_path );

    /* *** */

    le_void_t dl_sort_copy( le_char_t const * const dl_ipath, le_char_t const * const dl_opath );

    /* *** */

    le_size_t dl_sort_dispatch( le_char_t const * const dl_path, le_size_t const dl_size, le_byte_t const dl_depth, le_char_t const * const dl_temp );

    /* *** */

    void dl_sort_memory( le_char_t const * const dl_ipath, le_char_t const * const dl_opath, le_size_t const dl_size, le_byte_t const dl_depth );

    /* *** */

    void dl_sort_disk( le_char_t const * const dl_ipath, le_char_t const * const dl_opath, le_size_t const dl_size, le_byte_t const dl_depth, le_char_t const * const dl_temp );

    /*! \brief main methods
     *
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

