/*
 *  dalai-suite - common library
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

    /*! \file   common-geometry.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - geometry
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_GEOMETRY__
    # define __LC_GEOMETRY__

/*
    header - internal includes
 */

    # include <eratosthene-include.h>

/*
    header - external includes
 */

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

    /*! \brief geometry methods
     *
     *  This function computes and returns the squared value of the euclidean
     *  distance between the two provided three-dimensional vectors.
     *
     *  \param dl_vec_a First vector array
     *  \param dl_vec_b Second vector array
     *
     *  \return Returns the squared value of the vector distance
     */

    le_real_t lc_geometry_squaredist( le_real_t * dl_vec_a, le_real_t * dl_vec_b );

/*
    header - inclusion guard
 */

    # endif

