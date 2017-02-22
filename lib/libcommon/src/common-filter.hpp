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

    /*! \file   common-args.h
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - arguments and parameters module
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_FILTER__
    # define __LC_FILTER__

/*
    header - internal includes
 */

    # include "common-uf3.hpp"
    # include "common-error.hpp"

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <limits>
    # include <cmath>
    # include <cstring>
    # include <cstdint>

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

    /*! \brief filtering methods
     *
     *  This filtering function reads the point cloud provided through the input
     *  stream and exports its filtered version in the output stream.
     *
     *  This function implements a variation of filtering methods implemented
     *  in \b dl_filter_unity() using the \b DL_FILTER_UNITY_UNIF mode value. In
     *  addition to the distance criterion, this function only keeps elements
     *  for which the criterion is true for at least \b dl_threshold of their
     *  neighbour elements.
     *
     *  \param  dl_istream   Input stream descriptor
     *  \param  dl_ostream   Output stream descriptor
     *  \param  dl_size      Size, in bytes, of the input stream
     *  \param  dl_mean      Minimums mean value
     *  \param  dl_factor    Minimums mean multiplier
     *  \param  dl_threshold Neighbour count threshold
     *
     *  \return Returns true on success, false otherwise
     */

    void lc_filter_homogeneous( std::ifstream & lc_istream, std::ofstream & lc_ostream, double const lc_mean, double const lc_factor, int64_t const lc_threshold );

    /*! \brief filtering methods
     *
     *  This filtering function reads the point cloud provided through the input
     *  stream and exports its filtered version in the output stream.
     *
     *  This function implements a variation of filtering methods implemented
     *  in \b dl_filter_unity() using the \b DL_FILTER_UNITY_ADAP mode value. In
     *  addition to the distance criterion, this function only keeps elements
     *  for which the criterion is true for at least \b dl_threshold of their
     *  neighbour elements.
     *
     *  \param  dl_istream   Input stream descriptor
     *  \param  dl_ostream   Output stream descriptor
     *  \param  dl_size      Size, in bytes, of the input stream
     *  \param  dl_mean      Minimums mean value
     *  \param  dl_factor    Minimums mean multiplier
     *  \param  dl_threshold Neighbour count threshold
     *
     *  \return Returns true on success, false otherwise
     */

    void lc_filter_adaptative( std::ifstream & lc_istream, std::ofstream & lc_ostream, double const lc_mean, double const lc_factor, int64_t const lc_threshold );

/*
    header - inclusion guard
 */

    # endif

