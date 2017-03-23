/*
 *  dalai-suite - filter
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
     *  Copyright (c) 2016-2017 EPFL CDH DHLAB
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
    # include <cstring>
    # include <cstdlib>
    # include <cstdint>
    # include <cinttypes>
    # include <unistd.h>
    # include <dirent.h>
    # include <common-include.hpp>

/*
    header - preprocessor definitions
 */

    /* define storage modes */
    # define DL_FILTER_CREATE ( 0 )
    # define DL_FILTER_DELETE ( 1 )

    /* define hashing parameter */
    # define DL_FILTER_HASH   ( 100.0 )

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

    /*! \brief storage methods
     *
     *  As \b DL_FILTER_CREATE is provided as mode, the function asks the system
     *  for an unused temporary directory in the system temporary volume. On
     *  success, the function creates the temporary directory and updates the
     *  content of the provided path with the created directory path.
     *
     *  If the provided mode is \b DL_FILTER_DELETE, the function simply removes
     *  the temporary directory provided by the \b dl_path parameter. It uses
     *  the \b rmdir function assuming the temporary directory is already empty.
     *  An error is reported otherwise.
     *
     *  \param dl_path Temporary storage directory path
     *  \param dl_mode Temporary storage mode
     */

    void dl_filter_temporary( char * const dl_path, int dl_mode );

    /*! \brief filtering methods
     *
     *  This function applies the filtering process on each file contained in
     *  the provided \b dl_opath directory. It simply enumerates the files of
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
     *  documentation of \b libcommon for more information.
     *
     *  \param dl_ostream    Output stream descriptor
     *  \param dl_ipath      Input directory path
     *  \param dl_mean       Minimum distance mean value
     *  \param dl_factor     Minimum distance mean multiplier
     *  \param dl_threshold  Neighbour count threshold
     *  \param dl_adaptative Adaptative filtering switch
     */

    void dl_filter( std::ofstream & dl_ostream, char const * const dl_ipath, double const dl_mean, double const dl_factor, int64_t const dl_threshold, bool const dl_adaptative );

    /*! \brief main function
     *
     *  The main function reads the point cloud provided through the input file
     *  and exports its filtered version in the provided output stream :
     *
     *      ./dalai-filter --input/-i [input file path]
     *                     --output/-o [output file path]
     *                     --adaptative/-a [filtering mode switch]
     *                     --factor/-f [minimum distance mean multiplier]
     *                     --count/-c [sampled elements for mean computation]
     *                     --threshold/-t [neighbour elements count threshold]
     *
     *  The main function starts by reading the parameters and opens the input
     *  file. It computes the point cloud elements minimum distances mean value
     *  using the count parameter.
     *
     *  In order to performs the filtering process, the input point cloud is
     *  cut in smallest pieces using the hashing function. The hashing function
     *  uses the minimums mean to adapt the hash to the input file point cloud.
     *
     *  The filtering process is then applied on each piece of the hashed point
     *  cloud and the results of the filtering of all the pieces is exported in
     *  the output file.
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

