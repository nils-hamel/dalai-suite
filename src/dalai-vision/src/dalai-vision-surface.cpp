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

    # include "dalai-vision-surface.hpp"

/*
    source - constructor/destructor methods
 */

    dl_surface_t::dl_surface_t( void )

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

        , sf_radius( 0.0 )

        , sf_r( 0.0 )
        , sf_g( 0.0 )
        , sf_b( 0.0 )

        , sf_size( 0 )
        , sf_virt( 0 )
        , sf_data( nullptr )

    { }

    dl_surface_t::~dl_surface_t( void ) {

        /* release memory */
        sf_set_release();

    }

/*
    source - accessor methods
 */

    le_void_t dl_surface_t::sf_get_intersection( dl_surface_t & dl_s2, dl_surface_t & dl_s3 ) {

        /* check consistency */
        if ( ( this->sf_size >= DL_SURFACE_MIN ) && ( dl_s2.sf_size >= DL_SURFACE_MIN ) && ( dl_s3.sf_size >= DL_SURFACE_MIN ) ) {

            /* intersection variable */
            le_real_t dl_xint( 0.0 );
            le_real_t dl_yint( 0.0 );
            le_real_t dl_zint( 0.0 );

            /* determinant variable */
            le_real_t dl_det( 0.0 );

            /* compute surfaces determinant */
            dl_det  += this->sf_px * ( dl_s2.sf_py * dl_s3.sf_pz - dl_s2.sf_pz * dl_s3.sf_py );
            dl_det  -= this->sf_py * ( dl_s2.sf_px * dl_s3.sf_pz - dl_s2.sf_pz * dl_s3.sf_px );
            dl_det  += this->sf_pz * ( dl_s2.sf_px * dl_s3.sf_py - dl_s2.sf_py * dl_s3.sf_px );

            /* compute intersection */
            dl_xint -= this->sf_pc * ( dl_s2.sf_py * dl_s3.sf_pz - dl_s2.sf_pz * dl_s3.sf_py );
            dl_xint += this->sf_py * ( dl_s2.sf_pc * dl_s3.sf_pz - dl_s2.sf_pz * dl_s3.sf_pc );
            dl_xint -= this->sf_pz * ( dl_s2.sf_pc * dl_s3.sf_py - dl_s2.sf_py * dl_s3.sf_pc );

            /* compute intersection */
            dl_yint -= this->sf_px * ( dl_s2.sf_pc * dl_s3.sf_pz - dl_s2.sf_pz * dl_s3.sf_pc );
            dl_yint += this->sf_pc * ( dl_s2.sf_px * dl_s3.sf_pz - dl_s2.sf_pz * dl_s3.sf_px );
            dl_yint -= this->sf_pz * ( dl_s2.sf_px * dl_s3.sf_pc - dl_s2.sf_pc * dl_s3.sf_px );

            /* compute intersection */
            dl_zint -= this->sf_px * ( dl_s2.sf_py * dl_s3.sf_pc - dl_s2.sf_pc * dl_s3.sf_py );
            dl_zint += this->sf_py * ( dl_s2.sf_px * dl_s3.sf_pc - dl_s2.sf_pc * dl_s3.sf_px );
            dl_zint -= this->sf_pc * ( dl_s2.sf_px * dl_s3.sf_py - dl_s2.sf_py * dl_s3.sf_px );

            /* display intersection */
            std::cerr << dl_xint / dl_det << " " << dl_yint / dl_det << " " << dl_zint / dl_det << std::endl;

        }

    }

/*
    source - mutator methods
 */

    le_void_t dl_surface_t::sf_set_point_push( le_real_t const dl_x, le_real_t const dl_y, le_real_t const dl_z, le_real_t const dl_tolerance ) {

        /* check removal procedure */
        if ( sf_set_point_remove( dl_x, dl_y, dl_z, dl_tolerance ) == false ) {

            /* memory management */
            sf_set_memory( 3 );

            /* push point */
            sf_data[sf_size - 3] = dl_x;
            sf_data[sf_size - 2] = dl_y;
            sf_data[sf_size - 1] = dl_z;

            /* update equation */
            sf_set_equation();

        }

    }

    le_void_t dl_surface_t::sf_set_point_auto( le_byte_t const * const dl_data, le_size_t const dl_size, le_real_t const dl_tolerance, le_real_t const dl_grow ) {

        /* buffer pointer variable */
        le_real_t * dl_uv3p( nullptr );

        /* distance variable */
        le_real_t dl_distance( 0.0 );

        /* check estimation constraint */
        if ( sf_size < DL_SURFACE_MIN ) {

            /* abort computation */
            return;

        }

        /* empty stack */
        sf_size = 0;

        /* parsing model */
        for ( le_size_t dl_parse( 0 ); dl_parse < dl_size; dl_parse += LE_UV3_RECORD ) {

            /* compute buffer pointer */
            dl_uv3p = ( le_real_t * ) ( dl_data + dl_parse );

            /* apply proximity condition */
            if ( std::fabs( sf_px * dl_uv3p[0] + sf_py * dl_uv3p[1] + sf_pz * dl_uv3p[2] + sf_pc ) <= dl_tolerance ) {

                /* compute distance to centroid */
                dl_distance  = ( dl_uv3p[0] - sf_cx ) * ( dl_uv3p[0] - sf_cx );
                dl_distance += ( dl_uv3p[1] - sf_cy ) * ( dl_uv3p[1] - sf_cy );
                dl_distance += ( dl_uv3p[2] - sf_cz ) * ( dl_uv3p[2] - sf_cz );

                /* apply proximity condition */
                if ( std::sqrt( dl_distance ) <= ( sf_radius + dl_grow ) ) {

                    /* memory management */
                    sf_set_memory( 3 );

                    /* push element */
                    sf_data[sf_size - 3] = dl_uv3p[0];
                    sf_data[sf_size - 2] = dl_uv3p[1];
                    sf_data[sf_size - 1] = dl_uv3p[2];

                }

            }

        }

        /* update equation */
        sf_set_equation();

    }

    bool dl_surface_t::sf_set_point_remove( le_real_t const dl_x, le_real_t const dl_y, le_real_t const dl_z, le_real_t dl_tolerance ) {

        /* distance variable */
        le_real_t dl_distance( 0.0 );

        /* compute squared tolerence */
        dl_tolerance *= dl_tolerance;

        /* parsing surface elements */
        for ( le_size_t dl_parse( 0 ); dl_parse < sf_size; dl_parse += 3 ) {

            /* compute point-point distance */
            dl_distance  = ( sf_data[dl_parse + 0] - dl_x ) * ( sf_data[dl_parse + 0] - dl_x );
            dl_distance += ( sf_data[dl_parse + 1] - dl_y ) * ( sf_data[dl_parse + 1] - dl_y );
            dl_distance += ( sf_data[dl_parse + 2] - dl_z ) * ( sf_data[dl_parse + 2] - dl_z );

            /* apply condition */
            if ( dl_distance <= dl_tolerance ) {

                /* stack element shift */
                for ( le_size_t dl_index( dl_parse + 3 ); dl_index < sf_size; dl_index += 3 ) {

                    /* shift element */
                    sf_data[dl_index - 3] = sf_data[dl_index + 0];
                    sf_data[dl_index - 2] = sf_data[dl_index + 1];
                    sf_data[dl_index - 1] = sf_data[dl_index + 2];

                }

                /* update stack size */
                sf_size -= 3;

                /* return answer */
                return( true );

            }

        }

        /* return status */
        return( false );

    }

    le_void_t dl_surface_t::sf_set_point_clear( le_void_t ) {

        /* empty stack */
        sf_size = 0;

    }

    le_void_t dl_surface_t::sf_set_color( le_real_t const dl_r, le_real_t const dl_g, le_real_t const dl_b ) {

        /* assign surface color */
        sf_r = dl_r;
        sf_g = dl_g;
        sf_b = dl_b;

    }

    le_void_t dl_surface_t::sf_set_equation( le_void_t ) {

        /* matrix variables */
        Eigen::MatrixXd dl_matrix( 3, sf_size / 3 );

        /* parsing variable */
        le_size_t dl_index( 0 );

        /* distance variable */
        le_real_t dl_radius( 0.0 );

        /* check estimation constraint */
        if ( sf_size < DL_SURFACE_MIN ) {

            /* abort computation */
            return;

        }

        /* reset surface centroid */
        sf_cx = 0.0;
        sf_cy = 0.0;
        sf_cz = 0.0;

        /* surface point accumulation */
        for ( le_size_t dl_parse( 0 ); dl_parse < sf_size; dl_parse += 3 ) {

            /* accumulate coordinates */
            sf_cx += sf_data[dl_parse + 0];
            sf_cy += sf_data[dl_parse + 1];
            sf_cz += sf_data[dl_parse + 2];

        }

        /* compute surface centroid */
        sf_cx /= double( sf_size / 3 );
        sf_cy /= double( sf_size / 3 );
        sf_cz /= double( sf_size / 3 );

        /* reset surface radius */
        sf_radius = 0.0;

        /* initialise matrix elements */
        for ( le_size_t dl_parse( 0 ); dl_parse < sf_size; dl_parse += 3 ) {

            /* compute matrix index */
            dl_index = dl_parse / 3;

            /* initialise elements */
            dl_matrix(0,dl_index) = sf_data[dl_parse + 0] - sf_cx;
            dl_matrix(1,dl_index) = sf_data[dl_parse + 1] - sf_cy;
            dl_matrix(2,dl_index) = sf_data[dl_parse + 2] - sf_cz;

            /* compute distance to centroid */
            dl_radius = std::sqrt( 

                dl_matrix(0,dl_index) * dl_matrix(0,dl_index) + 
                dl_matrix(1,dl_index) * dl_matrix(1,dl_index) + 
                dl_matrix(2,dl_index) * dl_matrix(2,dl_index) 

            );

            /* maximum radius detection */
            sf_radius = ( dl_radius > sf_radius ) ? dl_radius : sf_radius;

        }

        /* compute svd decomposition */
        Eigen::JacobiSVD <Eigen::MatrixXd> dl_svd( dl_matrix, Eigen::ComputeFullU );

        /* assign surface normal */
        sf_px = dl_svd.matrixU()(0,2);
        sf_py = dl_svd.matrixU()(1,2);
        sf_pz = dl_svd.matrixU()(2,2);

        /* compute surface constant */
        sf_pc = - sf_px * sf_cx - sf_py * sf_cy - sf_pz * sf_cz;

        /* perpare norm */
        sf_ux = std::sqrt( + sf_px * sf_px + sf_py * sf_py );

        /* compute surface vector */
        sf_uy = - sf_px / sf_ux;
        sf_ux = + sf_py / sf_ux;
        sf_uz = + 0.0;

        /* compute surface vector */
        sf_vx = sf_py * sf_uz - sf_pz * sf_uy;
        sf_vy = sf_pz * sf_ux - sf_px * sf_uz;
        sf_vz = sf_px * sf_uy - sf_py * sf_ux;

    }

    le_void_t dl_surface_t::sf_set_memory( le_size_t const dl_add ) {

        /* swap variable */
        le_real_t * dl_swap( nullptr );

        /* check requirement */
        if ( ( sf_size += dl_add ) > sf_virt ) {

            /* update virtual size */
            sf_virt += DL_SURFACE_STEP;

            /* buffer memory re-allocation */
            if ( ( dl_swap = ( ( le_real_t * ) realloc( sf_data, sf_virt * sizeof( le_real_t ) ) ) ) == nullptr ) {

                /* send message */
                throw( LC_ERROR_MEMORY );

            }

            /* update pointer */
            sf_data = dl_swap;

        }

    }

    le_void_t dl_surface_t::sf_set_release( le_void_t ) {

        /* reset stack size */
        sf_size = ( sf_virt = 0 );

        /* release buffer memory */
        free( sf_data );

        /* pointer invalidation */
        sf_data = nullptr;

    }

    le_void_t dl_surface_t::sf_set_pointsize( le_real_t const dl_factor ) {

        /* parameter variable */
        GLint dl_pointsize( 0 );

        /* retrieve point size */
        glGetIntegerv( GL_POINT_SIZE, & dl_pointsize );

        /* update point size */
        glPointSize( dl_pointsize * dl_factor );

    }

/*
    source - rendering methods
 */

    le_void_t dl_surface_t::sf_ren_surface( le_void_t ) {

        /* angle variable */
        le_real_t dl_cos( 0.0 );
        le_real_t dl_sin( 0.0 );

        /* assign color */
        glColor4f( sf_r, sf_g, sf_b, 0.6 );

        /* update array state */
        glEnableClientState ( GL_VERTEX_ARRAY );
        glDisableClientState( GL_COLOR_ARRAY  );

        /* update array pointer */
        glVertexPointer( 3, GL_DOUBLE, 0, sf_data );

        /* update point size */
        sf_set_pointsize( 2.0 );

        /* display surface points */
        glDrawArrays( GL_POINTS, 0, sf_size / 3 );

        /* update point size */
        sf_set_pointsize( 0.5 );

        /* check surface estimation */
        if ( sf_size >= DL_SURFACE_MIN ) {

            /* primitive bloc */
            glBegin( GL_LINE_LOOP );

            /* parsing angle */
            for ( le_real_t dl_parse( 0.0 ); dl_parse < LE_2P; dl_parse += ( LE_2P / 90.0 ) ) {

                /* compute angle */
                dl_cos = std::cos( dl_parse );
                dl_sin = std::sin( dl_parse );

                /* primitive vertex */
                glVertex3f( 

                    sf_cx + ( sf_ux * dl_cos + sf_vx * dl_sin ) * sf_radius * 5.0,
                    sf_cy + ( sf_uy * dl_cos + sf_vy * dl_sin ) * sf_radius * 5.0,
                    sf_cz + ( sf_uz * dl_cos + sf_vz * dl_sin ) * sf_radius * 5.0

                );

            }

            /* primitive bloc */
            glEnd();

        }

    }

    le_void_t dl_surface_t::sf_ren_blend( le_void_t ) {

        /* angle variable */
        le_real_t dl_cos( 0.0 );
        le_real_t dl_sin( 0.0 );

        /* assign color */
        glColor4f( sf_r, sf_g, sf_b, 0.6 );

        /* check surface estimation */
        if ( sf_size >= DL_SURFACE_MIN ) {

            /* update blending state */
            glEnable( GL_BLEND );

            /* primitive bloc */
            glBegin( GL_TRIANGLE_FAN );

            /* parsing angle */
            for ( le_real_t dl_parse( 0.0 ); dl_parse < LE_2P; dl_parse += ( LE_2P / 90.0 ) ) {

                /* compute angle */
                dl_cos = std::cos( dl_parse );
                dl_sin = std::sin( dl_parse );

                /* primitive vertex */
                glVertex3f( 

                    sf_cx + ( sf_ux * dl_cos + sf_vx * dl_sin ) * sf_radius * 5.0,
                    sf_cy + ( sf_uy * dl_cos + sf_vy * dl_sin ) * sf_radius * 5.0,
                    sf_cz + ( sf_uz * dl_cos + sf_vz * dl_sin ) * sf_radius * 5.0

                );

            }

            /* primitive bloc */
            glEnd();

        }

    }

