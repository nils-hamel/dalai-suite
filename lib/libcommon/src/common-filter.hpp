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

    /*! \file   common-filter.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - filtering module
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_FILTER__
    # define __LC_FILTER__

/*
    header - internal includes
 */

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

    /*! \brief filtering methods (revoked)
     *
     *  This filtering function reads the point cloud provided through the input
     *  stream and exports its filtered version in the output stream.
     *
     *  The implemented filtering method is based on the provided minimum
     *  distance mean value. This value is expected to give the mean value of
     *  the nearest neighbour of each element of the model. Usually, this value
     *  is computed considering a sample of the model (\b lc_statistic_mdmv()).
     *
     *  The function defines a threshold distance given by the multiplication
     *  of the minimum distance mean value with the provided \b lc_factor The
     *  function checks for each element of the model the amount of neighbour
     *  that are closer to the defined threshold. The elements that have at
     *  least \b lc_threshold elements below the condition are kept, the other
     *  being discarded.
     *
     *  This filtering method is called homogeneous because the model provided
     *  through the input stream is assumed to be only a portion of a larger
     *  model. It follows that the provided minimum distance mean value is
     *  computed on the overall model instead of the provided portion. As a
     *  result, the applied filtering condition is homogeneous from the entire
     *  model point of view.
     *
     *  \param lc_istream   Input stream descriptor
     *  \param lc_ostream   Output stream descriptor
     *  \param lc_mean      Minimums distance mean value
     *  \param lc_factor    Mean value multiplier
     *  \param lc_threshold Neighbour count threshold
     */

    le_void_t lc_filter_homogeneous( std::ifstream & lc_istream, std::ofstream & lc_ostream, le_real_t const lc_mean, le_real_t const lc_factor, le_size_t const lc_threshold );

    /*! \brief filtering methods (revoked)
     *
     *  This filtering function reads the point cloud provided through the input
     *  stream and exports its filtered version in the output stream.
     *
     *  This function implements a variation of the filtering method implemented
     *  in the \b lc_filter_homogeneous() function.
     *
     *  The variation of the filtering method consists in computing the minimum
     *  distance mean value of the input stream as if it was a entire model
     *  rather that a portion of it. The filtering condition is identical as in
     *  the previous function expect that the condition is computed using the
     *  computed local mean value.
     *
     *  This function can then be used to filter a complete model stored in a
     *  single stream without to provide a mean value. It can also be used on
     *  streams that contain only a portion of a larger model in order to
     *  implement a filtering algorithm able to take into account the local
     *  specificity of the model when applying the filtering condition.
     *
     *  \param lc_istream   Input stream descriptor
     *  \param lc_ostream   Output stream descriptor
     *  \param lc_factor    Local mean value multiplier
     *  \param lc_threshold Neighbour count threshold
     */

    le_void_t lc_filter_adaptative( std::ifstream & lc_istream, std::ofstream & lc_ostream, le_real_t const lc_factor, le_size_t const lc_threshold );

/*
    header - inclusion guard
 */

    # endif

