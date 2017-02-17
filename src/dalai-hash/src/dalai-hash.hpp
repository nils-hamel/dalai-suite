/*
 *  dalai-suite - hash
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

    /*! \file   dalai-hash.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - hash
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
     *  This standardised format allows to use the suite tools for colour
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

    # ifndef __DL_HASH__
    # define __DL_HASH__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <cmath>
    # include <limits>
    # include <cstring>
    # include <cstdlib>
    # include <cstdint>
    # include <common-include.hpp>

/*
    header - preprocessor definitions
 */

    /* define chunk size */
    # define DL_HASH_CHUNK ( 2097152ll )

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

    /*! \brief statistical methods
     *
     *  This function computes and returns the mean value of the point cloud
     *  elements distance to their closest element. To achieve this computation
     *  in a reasonable amount of time, the following strategy is considered.
     *
     *  The function starts by sampling \b dl_count elements in the point cloud
     *  provided through the stream descriptor. For each sampled element, it
     *  searches the distance to its closest element. It finally computes the
     *  mean value of the found minimal distances on the sampled set.
     *
     *  The approximation of the minimums mean value gets better as \b dl_count
     *  increases. Nevertheless, a value of 32 already allows to compute a very
     *  good approximation of the minimums mean value.
     *
     *  \param  dl_istream Input stream descriptor
     *  \param  dl_size    Size, in bytes, of the input stream
     *  \param  dl_count   Number of sampled elements
     *
     *  \return Returns mean minimum distance on success, 0.0 otherwise
     */

    double dl_hash_stat( std::ifstream & dl_istream, int64_t const dl_size, int64_t const dl_count );

    /*! \brief hashing methods
     *
     *  This function imports the point cloud provided by the input stream and
     *  hashes it in the output directory. The hashing consists in cutting the
     *  provided point cloud into many smaller point clouds.
     *
     *  The function reads the input stream elements one by one and uses the
     *  provided hashing parameter and the minimums mean value to determine in
     *  which sub point cloud each element has to be written. The hash function
     *  is driven by the following values computation :
     *
     *      h_i = ( int ) floor( p_i / ( dl_param * dl_mean ) )
     *
     *  with i = x,y,z. The three computed h_i values are then used to composed
     *  the sub point clouds file name. All elements sharing the same h_i values
     *  are then written in the same sub-point-cloud.
     *
     *  \param  dl_istream Input stream descriptor
     *  \param  dl_opath   Output path
     *  \param  dl_param   Hashing parameter
     *  \param  dl_mean    Minimums mean value
     *
     *  \return Returns \b true on success, \b false otherwise
     */

    bool dl_hash( std::ifstream & dl_istream, char const * const dl_opath, double const dl_param, double const dl_mean );

    /*! \brief main function
     *
     *  The main function hashes the provided point cloud into a set of smaller
     *  sub point clouds stored in the output directory :
     *
     *      ./dalai-hash --uf3/-i [input uf3 file]
     *                   --output/-o [output path directory]
     *                   --count/-c [sampled element count]
     *                   --param/-p [hashing parameter]
     *
     *  The functions starts by gathering the parameters and opens the provided
     *  input file. It computes the point cloud minimum distances mean value
     *  and hashes the point cloud using the \b dl_hash() function. The hashed
     *  sub point clouds are written in the provided output directory.
     *
     *  The count value gives the amount of points to consider to compute the
     *  minimums mean value. The hashing parameter is used with the mean value
     *  to determine the size of the hashed point clouds.
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

