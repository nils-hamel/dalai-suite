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
     *  \section dalai-suite
     *
     *  The _dalai-suite_ is designed to be a gathering and standardising
     *  interface to the geodetic system developed at the DHLAB of EPFL. It
     *  consists in a set of softwares having to convert any kind of geodetic
     *  data, mainly point clouds, into the defined universal format considered
     *  in the geodetic system. In other words, it is well described by its
     *  tibetan name.
     *
     *  \section Copyright and License
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
    # include <sstream>
    # include <cmath>
    # include <limits>
    # include <cstring>
    # include <cstdlib>
    # include <cstdint>
    # include <unistd.h>
    # include <dirent.h>
    # include <common-include.h>

/*
    header - preprocessor definitions
 */

    /* define chunk size */
    # define DL_FILTER_CHUNK      ( 2097152ll )

    /* define filtering mode */
    # define DL_FILTER_UNIFORM    ( 0 )
    # define DL_FILTER_ADAPTATIVE ( 1 )

    /* define temporary storage mode */
    # define DL_FILTER_CREATE     ( 0 )
    # define DL_FILTER_DELETE     ( 1 )

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
     *
     */

    bool dl_filter_temporary( char * const dl_path, int dl_mode );

    /*! \brief hashing methods
     *
     *
     */

    bool dl_filter_hash( std::ifstream & dl_istream, char const * const dl_opath, double const dl_mean );

    /*! \brief filtering methods
     *
     *
     */

    double dl_filter_stat( std::ifstream & dl_istream, int64_t const dl_size, int64_t const dl_count );

    /*! \brief filtering methods
     *
     *
     */

    bool dl_filter( std::ofstream & dl_ostream, char const * const dl_opath, double const dl_mean, double const dl_factor, int const dl_mode );

    /*! \brief filtering methods
     *
     *
     */

    bool dl_filter_threshold( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_mean, double const dl_factor, int const dl_mode );

    /*! \brief main function
     *
     *
     *  \param argc Standard parameter
     *  \param argv Standard parameter
     *
     *  \return Returns standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

