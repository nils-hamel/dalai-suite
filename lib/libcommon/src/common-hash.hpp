/*
 *  dalai-suite - common library
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

    /*! \file   common-hash.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - hashing module
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_HASH__
    # define __LC_HASH__

/*
    header - internal includes
 */

    # include "common-uf3.hpp"
    # include "common-error.hpp"

/*
    header - external includes
 */

    # include <fstream>
    # include <cmath>
    # include <cstdint>
    # include <inttypes.h>

/*
    header - preprocessor definitions
 */

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

    /*! \brief hashing methods
     *
     *  This function imports the point cloud provided by the input stream and
     *  hashes it in the output directory. The hashing consists in cutting the
     *  provided point cloud into many smaller point clouds.
     *
     *  The function reads the input stream elements one by one and uses the
     *  provided hashing parameter and the minimum distance mean value to
     *  determine in which sub point cloud each element has to be written. The
     *  hash function is driven by the following h_i values :
     *
     *      h_i = ( int ) floor( p_i / ( dl_param * dl_mean ) )
     *
     *  with i = x,y,z. The three computed h_i values are then used to composed
     *  the sub point clouds file name. All elements sharing the same h_i values
     *  are then written in the same sub point cloud.
     *
     *  In addition, a chunk size parameter has to be provided. It indicates the
     *  size of the segments to considers to read the provided input stream. It
     *  allows to maintain the amount of used memory to a specific value.
     *
     *  \param dl_istream Input stream descriptor
     *  \param dl_opath   Output directory path
     *  \param dl_param   Hashing parameter
     *  \param dl_mean    Minimum distance mean value
     *  \param dl_chunk   Chunk size, in elements count
     */

    void lc_hash( std::ifstream & lc_istream, char const * const lc_opath, double const lc_param, double const lc_mean, int64_t const lc_chunk );

/*
    header - inclusion guard
 */

    # endif

