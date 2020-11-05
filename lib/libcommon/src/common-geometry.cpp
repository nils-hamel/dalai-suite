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

    # include "common-geometry.hpp"

/*
    source - geometry methods
 */

    le_real_t lc_geometry_squaredist( le_real_t * dl_vec_a, le_real_t * dl_vec_b ) {

        /* component variable */
        le_real_t lc_comp_x( dl_vec_a[0] - dl_vec_b[0] );
        le_real_t lc_comp_y( dl_vec_a[1] - dl_vec_b[1] );
        le_real_t lc_comp_z( dl_vec_a[2] - dl_vec_b[2] );

        /* compute and return euclidean distance */
        return( lc_comp_x * lc_comp_x + lc_comp_y * lc_comp_y + lc_comp_z * lc_comp_z );

    }
