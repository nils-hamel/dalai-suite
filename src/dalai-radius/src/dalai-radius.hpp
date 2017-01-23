/*
 *  dalai-suite - radius
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

    /*! \file   dalai-radius.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - radius
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

    # ifndef __DL_RADIUS__
    # define __DL_RADIUS__

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

    # define DL_RADIUS_CHUNK ( 2097152ll )

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

    bool dl_radius_temp_create( char * const dl_tpath );

    /*! \brief storage methods
     *
     *
     */

    void dl_radius_temp_delete( char const * const dl_tpath );

    /*! \brief hashing methods
     *
     *
     */

    bool dl_radius_hash( std::ifstream & dl_istream, char const * const dl_tpath, int64_t const dl_size, double const dl_radius );

    /*! \brief filtering methods
     *
     *
     */

    double dl_radius_mean( std::ifstream & dl_istream, int64_t const dl_size, int64_t const dl_count );

    /*! \brief filtering methods
     *
     *
     */

    bool dl_radius_filter( std::ofstream & dl_ostream, char const * const dl_tpath, double const dl_radius, double const dl_factor );

    /*! \brief filtering methods
     *
     *
     */

    bool dl_radius_filter_uniform( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_radius, double const dl_factor );

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

