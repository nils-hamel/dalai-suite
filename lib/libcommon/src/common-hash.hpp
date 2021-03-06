/*
 *  dalai-suite - common library
 *
 *      Nils Hamel - nils.hamel@alumni.epfl.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
 *      Copyright (c) 2020 Republic and Canton of Geneva
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
     *  \author Nils Hamel <nils.hamel@alumni.epfl.ch>
     *
     *  dalai-suite - common library - hashing
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_HASH__
    # define __LC_HASH__

/*
    header - internal includes
 */

    # include "common-error.hpp"

/*
    header - external includes
 */

    # include <fstream>
    # include <cmath>
    # include <cstdint>
    # include <inttypes.h>
    # include <eratosthene-include.h>

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
     *  This function imports the primitives provided by the input stream and
     *  hashes them in the output directory. The hashing consists in cutting the
     *  provided model into many smaller sub-models based on cell-based space
     *  segmentation.
     *
     *  The function reads the input stream elements one by one and uses the
     *  provided hashing parameter and the minimum distances mean value to
     *  determine in which sub-model each element has to be written. The hash
     *  function is driven by the following h_i values :
     *
     *      h_i = ( int ) floor( p_i / ( lc_param * lc_mean ) )
     *
     *  with i = x,y,z. The three computed h_i values are then used to composed
     *  the sub-model file name. All elements sharing the same h_i values are
     *  then written in the same sub-model.
     *
     *  \param lc_istream Input stream descriptor
     *  \param lc_opath   Output directory path
     *  \param lc_param   Hashing parameter
     *  \param lc_mean    Minimum distance mean value
     */

    le_void_t lc_hash( std::ifstream & lc_istream, le_char_t const * const lc_opath, le_real_t const lc_param, le_real_t const lc_mean );

/*
    header - inclusion guard
 */

    # endif

