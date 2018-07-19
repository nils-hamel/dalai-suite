/*
 *  dalai-suite - common library
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

    /*! \file   common-statistic.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - statistic module
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_STATISTIC__
    # define __LC_STATISTIC__

/*
    header - internal includes
 */

    //# include "common-uf3.hpp"
    # include "common-error.hpp"

/*
    header - external includes
 */

    # include <fstream>
    # include <limits>
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

    /*! \brief statistical methods (revoked)
     *
     *  This function computes and returns the mean value of the elements vertex
     *  distance to their closest element. To achieve this computation in a
     *  reasonable amount of time, the following strategy is considered.
     *
     *  The function starts by sampling \b lc_count elements in the input stream
     *  provided through the stream descriptor. For each sampled element, it
     *  searches the distance to its closest element. It finally computes the
     *  mean value of the found minimal distances on the sampled set.
     *
     *  The approximation of the minimum distance mean value gets better as
     *  \b lc_count increases. Nevertheless, a value of 32 already allows to
     *  compute a very good approximation of the minimum distance mean value.
     *
     *  \param lc_istream Input stream descriptor
     *  \param lc_count   Number of sampled elements
     *
     *  \return Returns minimum distance mean value
     */

    double lc_statistic_mdmv( std::ifstream & lc_istream, int64_t const lc_count );

/*
    header - inclusion guard
 */

    # endif

