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

    /*! \file   dalai-sort-algorithm.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - sort algorithm
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_SORT_ALGORITHM__
    # define __DL_SORT_ALGORITHM__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <common-include.hpp>
    # include <eratosthene-include.h>
    # include <fstream>
    # include <cstring>

/*
    header - preprocessor definitions
 */

    /* define chunk size */
    # define DL_SORT_CHUNK  ( LE_ARRAY_UF3 * 9942053 )

    /* define buffer size */
    # define DL_SORT_BUFFER ( LE_ARRAY_UF3 * 4971026 )

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

    /* *** */

    bool dl_sort_algorithm( le_real_t const * const dl_fpose, le_real_t const * const dl_spose, le_byte_t const dl_depth );

    /* *** */

    le_byte_t * dl_sort_algorithm_memory( le_byte_t * const dl_buffer, le_size_t const dl_size, le_byte_t const dl_depth );

    /* *** */

    void dl_sort_algorithm_disk( le_char_t const * const dl_fpath, le_size_t const dl_flength, le_char_t const * const dl_spath, le_size_t const dl_slength, le_char_t const * const dl_opath, le_byte_t const dl_depth );

/*
    header - inclusion guard
 */

    # endif

