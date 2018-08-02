/*
 *  dalai-suite - vision
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

    # include "dalai-vision-arcball.hpp"

/*
    source - constructor/destructor methods
 */

    dl_arcball_t::dl_arcball_t( le_size_t const dl_width, le_size_t dl_height ) 

        : ab_width ( dl_width  >> 1 )
        , ab_height( dl_height >> 1 )

    {

        /* initialise matrix */
        ab_set_identity();

        /* detect largest dimension */
        ab_size = ( ab_width > ab_height ) ? ab_width : ab_height;

    }

/*
    source - accessor methods
 */

    le_void_t dl_arcball_t::ab_get_rotate( le_void_t ) {

        /* apply arcball on matrix */
        glMultMatrixd( ab_view );

    }

/*
    source - mutator methods
 */

    le_void_t dl_arcball_t::ab_set_identity( le_void_t ) {

        /* parsing matrix */
        for ( le_size_t dl_parse( 0 ); dl_parse < 16; dl_parse ++ ) {

            /* set matrix to identity */
            ab_view[dl_parse] = ( ( dl_parse % 5 ) == 0 ) ? 1.0 : 0.0;

        }

    }

    le_void_t dl_arcball_t::ab_set_multiply( le_real_t * dl_matrix ) {

        /* copy variable */
        le_real_t dl_copy[16];

        /* create matrix copy */
        std::memcpy( dl_copy, ab_view, sizeof( le_real_t ) * 16 );

        /* compute matrix product */
        ab_view[ 0] = dl_matrix[ 0] * dl_copy[ 0] + dl_matrix[ 4] * dl_copy[ 1] + dl_matrix[ 8] * dl_copy[ 2] + dl_matrix[12] * dl_copy[ 3];
        ab_view[ 4] = dl_matrix[ 0] * dl_copy[ 4] + dl_matrix[ 4] * dl_copy[ 5] + dl_matrix[ 8] * dl_copy[ 6] + dl_matrix[12] * dl_copy[ 7];
        ab_view[ 8] = dl_matrix[ 0] * dl_copy[ 8] + dl_matrix[ 4] * dl_copy[ 9] + dl_matrix[ 8] * dl_copy[10] + dl_matrix[12] * dl_copy[11];
        ab_view[12] = dl_matrix[ 0] * dl_copy[12] + dl_matrix[ 4] * dl_copy[13] + dl_matrix[ 8] * dl_copy[14] + dl_matrix[12] * dl_copy[15];

        /* compute matrix product */
        ab_view[ 1] = dl_matrix[ 1] * dl_copy[ 0] + dl_matrix[ 5] * dl_copy[ 1] + dl_matrix[ 9] * dl_copy[ 2] + dl_matrix[13] * dl_copy[ 3];
        ab_view[ 5] = dl_matrix[ 1] * dl_copy[ 4] + dl_matrix[ 5] * dl_copy[ 5] + dl_matrix[ 9] * dl_copy[ 6] + dl_matrix[13] * dl_copy[ 7];
        ab_view[ 9] = dl_matrix[ 1] * dl_copy[ 8] + dl_matrix[ 5] * dl_copy[ 9] + dl_matrix[ 9] * dl_copy[10] + dl_matrix[13] * dl_copy[11];
        ab_view[13] = dl_matrix[ 1] * dl_copy[12] + dl_matrix[ 5] * dl_copy[13] + dl_matrix[ 9] * dl_copy[14] + dl_matrix[13] * dl_copy[15];

        /* compute matrix product */
        ab_view[ 2] = dl_matrix[ 2] * dl_copy[ 0] + dl_matrix[ 6] * dl_copy[ 1] + dl_matrix[10] * dl_copy[ 2] + dl_matrix[14] * dl_copy[ 3];
        ab_view[ 6] = dl_matrix[ 2] * dl_copy[ 4] + dl_matrix[ 6] * dl_copy[ 5] + dl_matrix[10] * dl_copy[ 6] + dl_matrix[14] * dl_copy[ 7];
        ab_view[10] = dl_matrix[ 2] * dl_copy[ 8] + dl_matrix[ 6] * dl_copy[ 9] + dl_matrix[10] * dl_copy[10] + dl_matrix[14] * dl_copy[11];
        ab_view[14] = dl_matrix[ 2] * dl_copy[12] + dl_matrix[ 6] * dl_copy[13] + dl_matrix[10] * dl_copy[14] + dl_matrix[14] * dl_copy[15];

        /* compute matrix product */
        ab_view[ 3] = dl_matrix[ 3] * dl_copy[ 0] + dl_matrix[ 7] * dl_copy[ 1] + dl_matrix[11] * dl_copy[ 2] + dl_matrix[15] * dl_copy[ 3];
        ab_view[ 7] = dl_matrix[ 3] * dl_copy[ 4] + dl_matrix[ 7] * dl_copy[ 5] + dl_matrix[11] * dl_copy[ 6] + dl_matrix[15] * dl_copy[ 7];
        ab_view[11] = dl_matrix[ 3] * dl_copy[ 8] + dl_matrix[ 7] * dl_copy[ 9] + dl_matrix[11] * dl_copy[10] + dl_matrix[15] * dl_copy[11];
        ab_view[15] = dl_matrix[ 3] * dl_copy[12] + dl_matrix[ 7] * dl_copy[13] + dl_matrix[11] * dl_copy[14] + dl_matrix[15] * dl_copy[15];

    }

    le_void_t dl_arcball_t::ab_set_update( le_size_t const dl_init_x, le_size_t const dl_init_y, le_size_t const dl_step_x, le_size_t const dl_step_y ) {

        /* position variable */
        le_real_t dl_init[4] = { 0.0 };

        /* position variable */
        le_real_t dl_step[4] = { 0.0 };

        /* vector variable */
        le_real_t dl_axis[4] = { 0.0 };

        /* matrix variable */
        le_real_t dl_rotate[16] = { 0.0 };

        /* angle variable */
        le_real_t dl_cos( 0.0 );
        le_real_t dl_sin( 0.0 );

        /* optimisation variable */
        le_real_t dl_optim( 0.0 );

        /* compute position vector */
        dl_init[0] = ( dl_init_x - ab_width  ) / ab_size;
        dl_init[1] = ( ab_height - dl_init_y ) / ab_size;

        /* compute position vector */
        dl_step[0] = ( dl_step_x - ab_width  ) / ab_size;
        dl_step[1] = ( ab_height - dl_step_y ) / ab_size;
        
        /* compute radial component */
        dl_init[3] = dl_init[0] * dl_init[0] + dl_init[1] * dl_init[1];
        dl_step[3] = dl_step[0] * dl_step[0] + dl_step[1] * dl_step[1];

        /* compute z-component */
        if ( dl_init[3] <= 1.0 ) dl_init[2] = std::sqrt( 1.0 - dl_init[3] );
        if ( dl_step[3] <= 1.0 ) dl_step[2] = std::sqrt( 1.0 - dl_step[3] );

        /* compute norms */
        dl_init[3] = std::sqrt( dl_init[3] + dl_init[2] * dl_init[2] );
        dl_step[3] = std::sqrt( dl_step[3] + dl_step[2] * dl_step[2] );

        /* normalise vector */
        dl_init[0] /= dl_init[3];
        dl_init[1] /= dl_init[3];
        dl_init[2] /= dl_init[3];

        /* normalise vector */
        dl_step[0] /= dl_step[3];
        dl_step[1] /= dl_step[3];
        dl_step[2] /= dl_step[3];

        /* compute angle cosine */
        dl_cos = dl_init[0] * dl_step[0] + dl_init[1] * dl_step[1] + dl_init[2] * dl_step[2];

        /* compute angle sine */
        dl_sin = std::sqrt( 1.0 - dl_cos * dl_cos );

        /* compute rotation axis */
        dl_axis[0] = ( dl_init[1] * dl_step[2] - dl_init[2] * dl_step[1] ) / dl_sin;
        dl_axis[1] = ( dl_init[2] * dl_step[0] - dl_init[0] * dl_step[2] ) / dl_sin;
        dl_axis[2] = ( dl_init[0] * dl_step[1] - dl_init[1] * dl_step[0] ) / dl_sin;

        /* optimisation variable */
        dl_optim = 1.0 - dl_cos;

        /* compute rotation matrix */
        dl_rotate[ 0] = dl_axis[0] * dl_axis[0] * dl_optim + dl_cos;
        dl_rotate[ 4] = dl_axis[0] * dl_axis[1] * dl_optim - dl_sin * dl_axis[2];
        dl_rotate[ 8] = dl_axis[0] * dl_axis[2] * dl_optim + dl_sin * dl_axis[1];
        dl_rotate[ 1] = dl_axis[0] * dl_axis[1] * dl_optim + dl_sin * dl_axis[2];
        dl_rotate[ 5] = dl_axis[1] * dl_axis[1] * dl_optim + dl_cos;
        dl_rotate[ 9] = dl_axis[1] * dl_axis[2] * dl_optim - dl_sin * dl_axis[0];
        dl_rotate[ 2] = dl_axis[0] * dl_axis[2] * dl_optim - dl_sin * dl_axis[1];
        dl_rotate[ 6] = dl_axis[1] * dl_axis[2] * dl_optim + dl_sin * dl_axis[0];
        dl_rotate[10] = dl_axis[2] * dl_axis[2] * dl_optim + dl_cos;
        dl_rotate[15] = 1.0;

        /* apply matrix on arcball */
        ab_set_multiply( dl_rotate );

    }

