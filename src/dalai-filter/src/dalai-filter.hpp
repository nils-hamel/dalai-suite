/*
 *  dalai-suite - filter
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
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
     *  The _dalai-suite_ offers a toolbox for large scale 3D models
     *  manipulation, processing, conversion and geographical registration. It
     *  is mainly designed to offers solutions suitable for models that can
     *  weight hundreds of gigabytes and that are used in the domain of
     *  environment survey and land registering. It was initially developed to
     *  offer the required tools needed to allows massive data processing and
     *  preparation for the _Eratosthene Project_.
     *
     *  The _dalai-suite_ uses a common and very simple format for all the
     *  implemented tools that is also used for the _Eratosthene Project_. This
     *  format offers a simple way of storing massive amount of graphical
     *  primitives. This allows the _dalai-suite_ to manipulate all sort of 3D
     *  models, from large scale point-based models to more refined vector
     *  models.
     *
     *  The _dalai-suite_ comes also with tools dedicated to models
     *  visualization and large scale models processing such as automatic
     *  segmentation, cleaning, geographical transformation and assisted
     *  geographical registration.
     *
     *  The _dalai-suite_ also comes with a set of tools dedicated to file
     *  format conversion that allows data coming in their specific formats to
     *  be converted in the _suite_ format.
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2020 DHLAB, EPFL
     *
     *  This program is licensed under the terms of the GNU GPLv3. Documentation
     *  and illustrations are licensed under the terms of the CC BY 4.0.
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
     *  the directory and applies the filtering process on each of them. The
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
     *  adaptive filtering process is considered, the homogeneous one otherwise.
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
     *                     --temporary/-y [Temporary directory path]
     *                     --adaptive/-a [filtering mode switch]
     *                     --factor/-f [minimum distances mean value factor]
     *                     --count/-c [sampled elements for mean computation]
     *                     --threshold/-t [neighbour count threshold]
     *
     *  The main function assumes that the provided input uv3 stream contains
     *  only point primitives. The filtering process does not checks the record
     *  primitive type and applies the filter on each record which can breaks
     *  them in case line and triangles appears in the uv3 stream.
     *
     *  The main function starts by reading the parameters and opens the input
     *  file. It computes the elements minimum distances mean value using the
     *  count parameter provided by the user.
     *
     *  In order to performs the filtering process, the input stream is cut in
     *  smaller pieces using the \b libcommon hashing function. The hashing
     *  function uses the minimum distances mean value and the default factor
     *  \b DL_FILTER_HASH to scale the hashing.
     *
     *  The filtering process is then applied on each piece of the hashed input
     *  stream and the results of the filtering of all the pieces is exported in
     *  the output stream.
     *
     *  The temporary directory is mandatory and used to store the pieces of the
     *  model created during the hashing process. It has to offer enough of free
     *  space according to the size of the model to filter.
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

