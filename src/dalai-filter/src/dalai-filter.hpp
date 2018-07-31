/*
 *  dalai-suite - filter
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

    /*! \file   dalai-filter.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - filter
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

    # ifndef __DL_FILTER__
    # define __DL_FILTER__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <cstdio>
    # include <dirent.h>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define hashing parameter */
    # define DL_FILTER_HASH ( 75.0 )

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

    /*! \brief filtering methods
     *
     *  This function applies the filtering process on each file contained in
     *  the provided \b dl_ipath directory. It simply enumerates the files of
     *  the directory and applies the filtering process on each one. The
     *  resulting filtered point clouds are all exported in the same output
     *  stream.
     *
     *  Usually, this function is applied on the directory in which the input
     *  point cloud has been hashed. In other words, this filtering function
     *  assumes that all the point cloud files found in the provided input
     *  directory are parts of the same input point cloud.
     *
     *  Two variations of the same filtering methods are available through this
     *  function, the homogeneous filtering and the adaptive one. See the
     *  documentation of \b libcommon for more information about these filtering
     *  methods.
     *
     *  \param dl_ostream   Output stream
     *  \param dl_ipath     Input directory path
     *  \param dl_mean      Minimum distance mean value
     *  \param dl_factor    Minimum distance mean value factor
     *  \param dl_threshold Neighbour count threshold
     *  \param dl_adaptive  adaptive filtering switch
     */

    le_void_t dl_filter( std::ofstream & dl_ostream, le_char_t const * const dl_ipath, le_real_t dl_mean, le_real_t const dl_factor, le_size_t const dl_threshold, bool const dl_adaptive );

    /*! \brief main function
     *
     *  The main function reads the point cloud provided through the input file
     *  and exports its filtered version in the provided output stream :
     *
     *      ./dalai-filter --input/-i [input uv3 file path]
     *                     --output/-o [output uv3 file path]
     *                     --adaptive/-a [filtering mode switch]
     *                     --factor/-f [minimum distance mean value factor]
     *                     --count/-c [sampled elements for mean computation]
     *                     --threshold/-t [neighbour count threshold]
     *
     *  The main function assumes that the provided input uv3 stream contains
     *  only point primitive. The filtering process does not checks the record
     *  primitive type and applies the filter on each record.
     *
     *  The main function starts by reading the parameters and opens the input
     *  file. It computes the elements minimum distance mean value using the
     *  count parameter.
     *
     *  In order to performs the filtering process, the input stream is cut in
     *  smaller pieces using the hashing function. The hashing function uses the
     *  minimum distance mean value to scale the hashing process.
     *
     *  The filtering process is then applied on each piece of the hashed input
     *  stream and the results of the filtering of all the pieces is exported in
     *  the output stream.
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

