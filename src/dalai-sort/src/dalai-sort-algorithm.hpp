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

    # include <fstream>
    # include <cstring>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define chunk size */
    //# define DL_SORT_CHUNK  ( LE_ARRAY_UF3 * 9942053 )
    # define DL_SORT_CHUNK  ( LE_UV3_RECORD * 9942053 )

    /* define buffer size */
    //# define DL_SORT_BUFFER ( LE_ARRAY_UF3 * 4971026 )
    # define DL_SORT_BUFFER ( LE_UV3_RECORD * 4971026 )

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

    /*! \brief comparison methods
     *
     *  This function implements a index-based comparison algorithm of the
     *  provided two earth-attached 3D points. The successive digits of the
     *  two points are computed in parallel and are used to determine the
     *  order of the two element.
     *
     *  The function returns \b true if the first element is greater than the
     *  second one, false otherwise.
     *
     *  This function is the main comparison function used to implement earth-
     *  attached points and index-based sorting algorithm.
     *
     *  \param dl_fpose Pointer to a 3-vector containing the first point
     *  \param dl_spose Pointer to a 3-vector containing the second point
     *  \param dl_depth Sort comparison index maximum depth
     *
     *  \return Returns true if the first element is greater, false otherwise.
     */

    bool dl_sort_algorithm( le_real_t const * const dl_fpose, le_real_t const * const dl_spose, le_byte_t const dl_depth );

    /*! \brief sorting methods
     *
     *  This function implements a memory based merge sort algorithm for uf3
     *  records list.
     *
     *  The provided buffer has to contain the uf3 records to sort. The function
     *  returns that point at the sorted buffer. From a memory point of view, as
     *  the sorting process requires a secondary buffer, the returned buffer can
     *  differ from the provided one. In such a case, the provided buffer is
     *  released by the function.
     *
     *  \param dl_buffer Buffer containing the uf3 records
     *  \param dl_size   Size, in bytes, of the buffer
     *  \param dl_depth Sort comparison index maximum depth
     *
     *  \return Pointer to the sorted buffer
     */

    le_byte_t * dl_sort_algorithm_memory( le_byte_t * const dl_buffer, le_size_t const dl_size, le_byte_t const dl_depth );

    /*! \brief sorting methods
     *
     *  This function performs one step of the merge-sort algorithm by merging
     *  the two provided input file into one single and sorted output file.
     *
     *  \param dl_fpath   Path of the first file
     *  \param dl_flength Size, in bytes, of the first file
     *  \param dl_spath   Path of the second file
     *  \param dl_slength Size, in bytes, of the second file
     *  \param dl_opath   Path of the output file
     *  \param dl_depth Sort comparison index maximum depth
     */

    void dl_sort_algorithm_disk( le_char_t const * const dl_fpath, le_size_t const dl_flength, le_char_t const * const dl_spath, le_size_t const dl_slength, le_char_t const * const dl_opath, le_byte_t const dl_depth );

/*
    header - inclusion guard
 */

    # endif

