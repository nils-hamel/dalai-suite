/*
 *  dalai-suite - filter
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2019 DHLAB, EPFL
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
     *  \section overview Overview
     *
     *  The _dalai-suite_ is part of the Eratosthene Project and is dedicated to
     *  3D models gathering, processing and format conversion. It allows to
     *  consider the most common formats and to convert them into the project
     *  common format. The _dalai-suite_ is mainly used for data gathering and
     *  processing, allowing to make them compatible with remote Eratosthene
     *  servers injection format.
     *
     *  In addition, the project common format allows to apply the _dalai-suite_
     *  solutions for 3D models processing. The suite offers solution for
     *  massive 3D models cleaning, hashing and assisted geographical
     *  registration. The suite also provides solution for 3D models display and
     *  analysis.
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2019 DHLAB, EPFL
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
     *  resulting filtered models are all exported in the same output stream.
     *
     *  Usually, this function is applied on the directory in which the input
     *  model has been hashed. In other words, this filtering function assumes
     *  that all the model files found in the provided input directory are part
     *  of the same input model.
     *
     *  Two variations of the same filtering methods are available through this
     *  function, the homogeneous filtering and the adaptive one. See the
     *  documentation of \b libcommon for more information about these filtering
     *  methods. If \b true is provided as \b dl_adaptive parameter, the
     *  adaptive filtering process is considered.
     *
     *  Note : The implementation of the filtering process does not takes into
     *  account the type of the read records. It is then not suitable for non
     *  point-based models.
     *
     *  \param dl_ostream   Output stream
     *  \param dl_ipath     Input directory path
     *  \param dl_mean      Minimum distances mean value
     *  \param dl_factor    Minimum distances mean value factor
     *  \param dl_threshold Neighbour count threshold
     *  \param dl_adaptive  Adaptive filtering switch
     */

    le_void_t dl_filter( std::ofstream & dl_ostream, le_char_t const * const dl_ipath, le_real_t dl_mean, le_real_t const dl_factor, le_size_t const dl_threshold, bool const dl_adaptive );

    /*! \brief main function
     *
     *  The main function reads the model provided through the input uv3 file
     *  and exports its filtered version in the provided output uv3 file :
     *
     *      ./dalai-filter --input/-i [input uv3 file path]
     *                     --output/-o [output uv3 file path]
     *                     --adaptive/-a [filtering mode switch]
     *                     --factor/-f [minimum distances mean value factor]
     *                     --count/-c [sampled elements for mean computation]
     *                     --threshold/-t [neighbour count threshold]
     *
     *  The main function assumes that the provided input uv3 stream contains
     *  only point primitives. The filtering process does not checks the record
     *  primitive type and applies the filter on each record.
     *
     *  The main function starts by reading the parameters and opens the input
     *  file. It computes the elements minimum distances mean value using the
     *  count parameter.
     *
     *  In order to performs the filtering process, the input stream is cut in
     *  smaller pieces using the \b libcommon hashing function. The hashing
     *  function uses the minimum distances mean value to scale the hashing
     *  process.
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

