/*
 *  dalai-suite - vision
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

    # include "dalai-vision-surface.hpp"

/*
    source - constructor/destructor methods
 */

    dl_surface_t:: dl_surface_t( void )

        : sf_cx( 0.0 )
        , sf_cy( 0.0 )
        , sf_cz( 0.0 )

        , sf_px( 0.0 )
        , sf_py( 0.0 )
        , sf_pz( 0.0 )
        , sf_pc( 0.0 )

        , sf_ux( 0.0 )
        , sf_uy( 0.0 )
        , sf_uz( 0.0 )
        , sf_vx( 0.0 )
        , sf_vy( 0.0 )
        , sf_vz( 0.0 )

        , sf_cr( 0.0 )
        , sf_cg( 0.0 )
        , sf_cb( 0.0 )

        , sf_size( 0 )
        , sf_virt( 0 )
        , sf_data( nullptr )

    {

        /* set memory buffer */
        sf_set_memory();

    }

    dl_surface_t::~dl_surface_t( void ) {

        /* release memory buffer */
        sf_set_release();

    }

/*
    source - accessor methods
 */

    void dl_surface_t::sf_get_equation( double dl_equation[4] ) {

        /* assign surface equation parameters */
        dl_equation[0] = sf_px;
        dl_equation[1] = sf_py;
        dl_equation[2] = sf_pz;
        dl_equation[3] = sf_pc;

    }

/*
    source - mutator methods
 */

    void dl_surface_t::sf_set_point_push( double const dl_x, double const dl_y, double const dl_z, double const dl_limit ) {

        /* point removal procedure - abort on remove */
        if ( sf_set_point_remove( dl_x, dl_y, dl_z, dl_limit ) == true ) return;

        /* check buffer memory state */
        if ( ( sf_size + 3 ) >= sf_virt ) sf_set_memory();

        /* push point on buffer */
        sf_data[sf_size ++] = dl_x;
        sf_data[sf_size ++] = dl_y;
        sf_data[sf_size ++] = dl_z;

        /* recompute surface equation */
        sf_set_equation();

    }

    void dl_surface_t::sf_set_point_auto( char const * const dl_data, long long const dl_size, double const dl_limit ) {

        /* planimetric range variables */
        double dl_range( 0.0 );

        /* distance variables */
        double dl_distance( 0.0 );

        /* optimisation variables */
        double dl_norm( sqrt( sf_px * sf_px + sf_py * sf_py + sf_pz * sf_pz ) );

        /* array mapping variables */
        double * dl_pose( nullptr );

        /* parsing surface elements */
        for ( long long dl_parse( 0 ); dl_parse < sf_size; dl_parse += 3 ) {

            /* compute distance to surface centroid */
            dl_distance  = ( sf_data[dl_parse + 0] - sf_cx ) * ( sf_data[dl_parse + 0] - sf_cx );
            dl_distance += ( sf_data[dl_parse + 1] - sf_cy ) * ( sf_data[dl_parse + 1] - sf_cy );
            dl_distance += ( sf_data[dl_parse + 2] - sf_cz ) * ( sf_data[dl_parse + 2] - sf_cz );

            /* search for largest distance */
            if ( dl_distance > dl_range ) dl_range = dl_distance;

        }

        /* reset surface buffer size */
        sf_size = 0;

        /* parsing model elements */
        for( long long dl_parse( 0 ); dl_parse < dl_size; dl_parse += 27 ) {

            /* compute and assign array mapping */
            dl_pose = ( double * ) ( dl_data + dl_parse );

            /* compute distance to surface centroid */
            dl_distance  = ( dl_pose[0] - sf_cx ) * ( dl_pose[0] - sf_cx );
            dl_distance += ( dl_pose[1] - sf_cy ) * ( dl_pose[1] - sf_cy );
            dl_distance += ( dl_pose[2] - sf_cz ) * ( dl_pose[2] - sf_cz );

            /* check distance to surface centroid */
            if ( dl_distance > dl_range ) continue;

            /* compute orthogonal distance to surface */
            dl_distance = fabs( sf_px * dl_pose[0] + sf_py * dl_pose[1] + sf_pz * dl_pose[2] + sf_pc ) / dl_norm;

            /* check orthogonal distance to surface */
            if ( dl_distance > dl_limit ) continue;

            /* check buffer memory state */
            if ( ( sf_size + 3 ) >= sf_virt ) sf_set_memory();

            /* push element on buffer */
            sf_data[sf_size ++] = dl_pose[0];
            sf_data[sf_size ++] = dl_pose[1];
            sf_data[sf_size ++] = dl_pose[2];

        }

        /* recompute surface equation */
        sf_set_equation();

    }

    void dl_surface_t::sf_set_point_clear( void ) {

        /* reset surface buffer size */
        sf_size = 0;

    }

    bool dl_surface_t::sf_set_point_remove( double const dl_x, double const dl_y, double const dl_z, double const dl_limit ) {

        /* distance variables */
        double dl_distance( 0.0 );

        /* optimisation variables */
        double dl_condition( dl_limit * dl_limit );

        /* parsing surface elements */
        for ( long long dl_parse( 0 ); dl_parse < sf_size; dl_parse += 3 ) {

            /* compute distance to pushed element */
            dl_distance  = ( sf_data[dl_parse + 0] - dl_x ) * ( sf_data[dl_parse + 0] - dl_x );
            dl_distance += ( sf_data[dl_parse + 1] - dl_y ) * ( sf_data[dl_parse + 1] - dl_y );
            dl_distance += ( sf_data[dl_parse + 2] - dl_z ) * ( sf_data[dl_parse + 2] - dl_z );

            /* check distance to pushed element */
            if ( dl_distance > dl_condition ) continue;

            /* shift surface elements */
            for ( long long dl_index( dl_parse + 3 ); dl_index < sf_size; dl_index += 3 ) {

                /* shift element */
                sf_data[dl_index - 3] = sf_data[dl_index + 0];
                sf_data[dl_index - 2] = sf_data[dl_index + 1];
                sf_data[dl_index - 1] = sf_data[dl_index + 2];

            }

            /* update surface buffer size */
            sf_size -= 3;

            /* return status */
            return( true );

        }

        /* return status */
        return( false );

    }

    void dl_surface_t::sf_set_color( float const dl_cr, float const dl_cg, float const dl_cb ) {

        /* assign surface color */
        sf_cr = dl_cr;
        sf_cg = dl_cg;
        sf_cb = dl_cb;

    }

    void dl_surface_t::sf_set_equation( void ) {

        /* matrix variables */
        Eigen::MatrixXd dl_matrix( 3, sf_size / 3 );

        /* check consistency - abort computation */
        if ( sf_size < 12 ) return;

        /* reset surface centroid */
        sf_cx = 0.0;
        sf_cy = 0.0;
        sf_cz = 0.0;

        /* compute surface centroid */
        for ( long long dl_parse( 0 ); dl_parse < sf_size; dl_parse += 3 ) {

            /* accumulate elements coordinates */
            sf_cx += sf_data[dl_parse + 0];
            sf_cy += sf_data[dl_parse + 1];
            sf_cz += sf_data[dl_parse + 2];

        }

        /* compute surface centroid */
        sf_cx /= double( sf_size / 3 );
        sf_cy /= double( sf_size / 3 );
        sf_cz /= double( sf_size / 3 );

        /* initialise matrix elements */
        for ( long long dl_parse( 0 ); dl_parse < sf_size; dl_parse += 3 ) {

            /* initialise elements */
            dl_matrix(0,dl_parse / 3) = sf_data[dl_parse + 0] - sf_cx;
            dl_matrix(1,dl_parse / 3) = sf_data[dl_parse + 1] - sf_cy;
            dl_matrix(2,dl_parse / 3) = sf_data[dl_parse + 2] - sf_cz;

        }

        /* compute matrix svd decomposition */
        Eigen::JacobiSVD <Eigen::MatrixXd> dl_svd( dl_matrix, Eigen::ComputeFullU );

        /* assign computed surface normal */
        sf_px = dl_svd.matrixU()(0,2);
        sf_py = dl_svd.matrixU()(1,2);
        sf_pz = dl_svd.matrixU()(2,2);

        /* compute surface last component */
        sf_pc = - sf_px * sf_cx - sf_py * sf_cy - sf_pz * sf_cz;

        /* compute surface vector */
        sf_ux = + sf_px * sf_px + sf_py * sf_py;
        sf_uy = - sf_px / sf_ux;
        sf_ux = + sf_py / sf_ux;
        sf_uz = + 0.0;

        /* compute surface vector */
        sf_vx = sf_py * sf_uz - sf_pz * sf_uy;
        sf_vy = sf_pz * sf_ux - sf_px * sf_uz;
        sf_vz = sf_px * sf_uy - sf_py * sf_ux;

    }

    void dl_surface_t::sf_set_memory( void ) {

        /* memory swap variables */
        double * dl_swap( nullptr );

        /* re-allocate and check buffer memory */
        if ( ( dl_swap = ( double * ) realloc( sf_data, ( sf_virt + DL_SURFACE_STEP ) * sizeof( double ) ) ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* update virtual size */
        sf_virt += DL_SURFACE_STEP;

        /* assign memory pointer */
        sf_data = dl_swap;

    }

    void dl_surface_t::sf_set_release( void ) {

        /* reset surface buffer size and virtual size */
        sf_size = 0;
        sf_virt = 0;

        /* release buffer memory */
        free( sf_data );

        /* invalidate buffer pointer */
        sf_data = nullptr;

    }

/*
    source - render methods
 */

    void dl_surface_t::sf_ren_surface( double const dl_w ) {

        /* check consistency - abort rendering */
        if ( sf_size < 12 ) return;

        /* assign surface color */
        glColor4f( sf_cr, sf_cg, sf_cb, 0.25 );

        /* opengl primitives */
        glBegin( GL_QUADS );

            glVertex3f( sf_cx + ( + sf_ux + sf_vx ) * dl_w, sf_cy + ( + sf_uy + sf_vy ) * dl_w, sf_cz + ( + sf_uz + sf_vz ) * dl_w );
            glVertex3f( sf_cx + ( - sf_ux + sf_vx ) * dl_w, sf_cy + ( - sf_uy + sf_vy ) * dl_w, sf_cz + ( - sf_uz + sf_vz ) * dl_w );
            glVertex3f( sf_cx + ( - sf_ux - sf_vx ) * dl_w, sf_cy + ( - sf_uy - sf_vy ) * dl_w, sf_cz + ( - sf_uz - sf_vz ) * dl_w );
            glVertex3f( sf_cx + ( + sf_ux - sf_vx ) * dl_w, sf_cy + ( + sf_uy - sf_vy ) * dl_w, sf_cz + ( + sf_uz - sf_vz ) * dl_w );

        /* opengl primitives */
        glEnd();

    }

    void dl_surface_t::sf_ren_point( void ) {

        /* assign surface color */
        glColor3f( sf_cr, sf_cg, sf_cb );

        /* update opengl array state */
        glEnableClientState ( GL_VERTEX_ARRAY );
        glDisableClientState( GL_COLOR_ARRAY  );

        /* update opengl array pointer */
        glVertexPointer( 3, GL_DOUBLE, 0, sf_data );

        /* display opengl primitive arrays */
        glDrawArrays( GL_POINTS, 0, sf_size / 3 );

    }

