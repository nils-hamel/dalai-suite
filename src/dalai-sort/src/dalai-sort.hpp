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

    /*! \brief common methods
     *
     *  This function allows to obtain the size, in bytes, of the file pointed
     *  by the provided path. If the function fails or if the file does not
     *  exist, the function returns zero.
     *
     *  \param dl_path Path of the file
     *
     *  \return Return the file size in bytes on success, zero otherwise
     */

    le_size_t dl_sort_filesize( le_char_t const * const dl_path );

    /*! \brief common methods
     *
     *  This function performs file copy. The first provided path has to point
     *  to the source file while the second path has to point to the destination
     *  file. If the destination file exists, the function rewrites it.
     *
     *  \param dl_ipath Source path
     *  \param dl_opath Destination path
     */

    le_void_t dl_sort_copy( le_char_t const * const dl_ipath, le_char_t const * const dl_opath );

    /*! \brief dispatch methods
     *
     *  This function reads the provided file and segments it in smaller parts.
     *  Each parts of the file is sorted before to be exported. The function
     *  returns the amount of exported chunk.
     *
     *  The exported chunk have an identical size, \b DL_SORT_CHUNK, except for
     *  the last chunk that can be smaller according to the file original size.
     *
     *  \param dl_path  File path
     *  \param dl_size  File length, in bytes
     *  \param dl_depth Sort comparison index maximum depth
     *  \param dl_temp  Directory path of chunk exportation
     *
     *  \return Returns the amount of exported chunk
     */

    le_size_t dl_sort_dispatch( le_char_t const * const dl_path, le_size_t const dl_size, le_byte_t const dl_depth, le_char_t const * const dl_temp );

    /*! \brief sorting methods
     *
     *  This function performs merge sort of the provided file using a single
     *  memory buffer. The sorted content is exported in the provided output
     *  file.
     *
     *  \param dl_ipath Input file path
     *  \param dl_opath Output file path
     *  \param dl_size  Input file size, in bytes
     *  \param dl_depth Sort comparison index maximum depth
     */

    void dl_sort_memory( le_char_t const * const dl_ipath, le_char_t const * const dl_opath, le_size_t const dl_size, le_byte_t const dl_depth );

    /*! \brief sorting methods
     *
     *  This function performs a merge sort of the provided file using disk as
     *  intermediate storage. This function is used as the size of the file to
     *  sort is larger than the available memory. The sorted file is exported
     *  using the provided output path.
     *
     *  The function uses the provided temporary directory to store the merge
     *  sort steps intermediates elements.
     *
     *  \param dl_ipath Input file path
     *  \param dl_opath Output file path
     *  \param dl_size  Input file size, in bytes
     *  \param dl_depth Sort comparison index maximum depth
     *  \param dl_temp  Temporary directory path
     */

    void dl_sort_disk( le_char_t const * const dl_ipath, le_char_t const * const dl_opath, le_size_t const dl_size, le_byte_t const dl_depth, le_char_t const * const dl_temp );

    /*! \brief main methods
     *
     *  This software allows to perform a sorting of the provided uf3 file in
     *  order to perform eratosthene server pre-injection optimisation :
     *
     *      ./dalai-sort --input/-i [input file path]
     *                   --output/-o [output file path]
     *                   --depth/-d [comparison index depth]
     *                   --temporary/-t [temporary directory path]
     *
     *  The function starts by checking the consistency of the parameter before
     *  to decide, based on the input file size, if a pure memory process can
     *  be performed.
     *
     *  In such case, the function calls sub-routine that loads the file in
     *  memory before to merge-sort and export it.
     *
     *  In the case the input file is larger than the imposed limit, the main
     *  function calls the disk-assisted merge-sort process. In such a case,
     *  the merge sort is performed on chunks of the input file before to be
     *  merged and exported to the output file. In this case, the temporary
     *  directory has to be provided.
     *
     *  The comparison index depth indicates to the comparison function the
     *  maximum amount of digits to consider while comparing to element index.
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

