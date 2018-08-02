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

    # include "dalai-vision-model.hpp"

/*
    source - constructor/destructor methods
 */

    dl_model_t::dl_model_t( le_char_t const * const dl_path )

        : ml_size( 0 )
        , ml_real( 0 )
        , ml_data( nullptr )
        , ml_x( 0.0 )
        , ml_y( 0.0 )
        , ml_z( 0.0 )
        , ml_hide( 1 )
        , ml_mdmv( 0.0 )
        , ml_span( 0.0 )
        , ml_active( 0 )

    {

        /* stream variable */
        std::ifstream dl_stream( ( char * ) dl_path, std::ios::binary | std::ios::ate );

        /* check stream */
        if ( dl_stream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* push stream size */
        ml_real = ( ml_size = dl_stream.tellg() ) / LE_UV3_RECORD;

        /* check random capability */
        if ( RAND_MAX < ml_size ) {

            /* send message */
            throw( LC_ERROR_RANDOM );

        }

        /* initialise random */
        srand( time( nullptr ) );

        /* allocate buffer memory */
        if ( ( ml_data = new ( std::nothrow ) le_byte_t[ml_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* clear stream */
        dl_stream.clear();

        /* stream position */
        dl_stream.seekg( 0 );

        /* import stream bytes */
        dl_stream.read( ( char * ) ml_data, ml_size );

        /* check read bytes */
        if ( dl_stream.gcount() != ml_size ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* delete stream */
        dl_stream.close();

        /* create surface colors */
        ml_surface[0].sf_set_color( 1.0, 0.2, 0.2 );
        ml_surface[1].sf_set_color( 0.2, 1.0, 0.2 );
        ml_surface[2].sf_set_color( 0.2, 0.2, 1.0 );

        /* initialise primitive count */
        ml_count[0] = 0;
        ml_count[1] = 0;

        /* initialise primitive index */
        ml_index[0] = nullptr;
        ml_index[1] = nullptr;

        /* model analysis */
        ml_set_analysis();

    }

    dl_model_t::~dl_model_t() {

        /* check buffer */
        if ( ml_data != nullptr ) {

            /* release buffer memory */
            delete [] ml_data;

            /* pointer invalidation */
            ml_data = nullptr;

        }

        /* check array */
        if ( ml_index[0] != nullptr ) {

            /* release array memory */
            delete [] ml_index[0];

            /* pointer invalidation */
            ml_index[0] = nullptr;

        }

        /* check array */
        if ( ml_index[1] != nullptr ) {

            /* release array memory */
            delete [] ml_index[1];

            /* pointer invalidation */
            ml_index[1] = nullptr;

        }

    }

/*
    source - accessor methods
 */

    le_real_t dl_model_t::ml_get_span( le_void_t ) {

        /* return model span */
        return( ml_span );

    }

    le_void_t dl_model_t::ml_get_intersection( le_void_t ) {

        /* compute surface intersection */
        ml_surface[0].sf_get_intersection( ml_surface[1], ml_surface[2] );

    }

    le_void_t dl_model_t::ml_get_translation( le_void_t ) {

        /* apply model center translation */
        glTranslated( -ml_x, -ml_y, -ml_z );

    }

/*
    source - mutator methods
 */

    le_void_t dl_model_t::ml_set_center( le_real_t const dl_x, le_real_t const dl_y, le_real_t const dl_z ) {

        /* model center */
        ml_x = dl_x;
        ml_y = dl_y;
        ml_z = dl_z;

    }

    le_void_t dl_model_t::ml_set_surface( le_size_t const dl_surface ) {

        /* check consistency */
        if ( ( dl_surface < 0 ) || ( dl_surface > 2 ) ) {

            /* send message */
            throw( LC_ERROR_DOMAIN );

        }

        /* update active surface */
        ml_active = dl_surface;

    }

    le_void_t dl_model_t::ml_set_switch( le_void_t ) {

        /* update display mode */
        ml_hide = 1 - ml_hide;

    }

    le_void_t dl_model_t::ml_set_push( le_void_t ) {

        /* push center to surface */
        ml_surface[ml_active].sf_set_point_push( ml_x, ml_y, ml_z, ml_mdmv * 2.0 );

    }

    le_void_t dl_model_t::ml_set_auto( le_size_t const dl_mode ) {

        /* check mode */
        if ( dl_mode < 0 ) {

            /* automatic point selection */
            ml_surface[ml_active].sf_set_point_auto( ml_data, ml_size, ml_mdmv * 2.0, - ml_mdmv * 2.0 );

        } else if ( dl_mode > 0 ) {

            /* automatic point selection */
            ml_surface[ml_active].sf_set_point_auto( ml_data, ml_size, ml_mdmv * 2.0, + ml_mdmv * 2.0 );

        } else {

            /* automatic point selection */
            ml_surface[ml_active].sf_set_point_auto( ml_data, ml_size, ml_mdmv * 2.0, 0 );

        }

    }

    le_void_t dl_model_t::ml_set_clear( le_void_t ) {

        /* empty surface stack */
        ml_surface[ml_active].sf_set_point_clear();

    }

    le_void_t dl_model_t::ml_set_analysis( le_void_t ) {

        /* sample array variable */
        le_size_t dl_sample[DL_MODEL_SAMPLE];

        /* mdmv array variable */
        le_real_t dl_mdmv[DL_MODEL_SAMPLE];

        /* distance variable */
        le_real_t dl_distance( 0.0 );

        /* primitive type variable */
        le_size_t dl_type( 0 );

        /* buffer pointer variable */
        le_real_t * dl_uv3p( nullptr );

        /* buffer pointer variable */
        le_real_t * dl_uv3s( nullptr );

        /* initialise arrays */
        for ( le_size_t dl_parse( 0 ); dl_parse < DL_MODEL_SAMPLE; dl_parse ++ ) {

            /* initialise mdmv array */
            dl_mdmv[dl_parse] = std::numeric_limits<le_real_t>::max();

            /* initialise sampling array */
            dl_sample[dl_parse] = ( rand() % ml_real ) * LE_UV3_RECORD;

        }

        /* parsing model */
        for ( le_size_t dl_parse( 0 ); dl_parse < ml_size; dl_parse += LE_UV3_RECORD ) {

            /* compute buffer pointer */
            dl_uv3p = ( le_real_t * ) ( ml_data + dl_parse );

            /* retrieve and check primitive type */
            if ( ( dl_type = * ( ( le_byte_t * ) ( dl_uv3p + 3 ) ) - 2 ) >= 0 ) {

                /* update primitive count */
                ml_count[dl_type] ++;

            }

            /* model center computation */
            ml_x += dl_uv3p[0];
            ml_y += dl_uv3p[1];
            ml_z += dl_uv3p[2];

            /* parsing sample */
            for ( le_size_t dl_index( 0 ); dl_index < DL_MODEL_SAMPLE; dl_index ++ ) {

                /* avoid identity */
                if ( dl_parse != dl_sample[dl_index] ) {

                    /* compute buffer pointer */
                    dl_uv3s = ( le_real_t * ) ( ml_data + dl_sample[dl_index] );

                    /* compute and compare distance */
                    if ( ( dl_distance = lc_geometry_squaredist( dl_uv3p, dl_uv3s ) ) < dl_mdmv[dl_index] ) {

                        /* update mdmv array */
                        dl_mdmv[dl_index] = dl_distance;

                    }

                }

            }

        }

        /* compute model center */
        ml_x /= le_real_t( ml_real );
        ml_y /= le_real_t( ml_real );
        ml_z /= le_real_t( ml_real );

        /* compute mdmv value */
        for ( le_size_t dl_parse( 0 ); dl_parse < DL_MODEL_SAMPLE; dl_parse ++ ) {

            /* value accumulation */
            ml_mdmv += std::sqrt( dl_mdmv[dl_parse] );

        }

        /* compute model mdmv */
        ml_mdmv /= le_real_t( DL_MODEL_SAMPLE );

        /* allocate array memory */
        ml_index[0] = new ( std::nothrow ) GLuint[ml_count[0]];
        ml_index[1] = new ( std::nothrow ) GLuint[ml_count[1]];

        /* reset array size */
        ml_count[0] = 0;
        ml_count[1] = 0;

        /* parsing model */
        for ( le_size_t dl_parse( 0 ); dl_parse < ml_size; dl_parse += LE_UV3_RECORD ) {

            /* compute buffer pointer */
            dl_uv3p = ( le_real_t * ) ( ml_data + dl_parse );

            /* retrieve and check primitive type */
            if ( ( dl_type = * ( ( le_byte_t * ) ( dl_uv3p + 3 ) ) - 2 ) >= 0 ) {

                /* assign primitive index */
                ml_index[dl_type][ml_count[dl_type]++] = dl_parse / LE_UV3_RECORD;

            }

            /* compute distance to center */
            dl_distance = ( dl_uv3p[0] - ml_x ) * ( dl_uv3p[0] - ml_x ) + ( dl_uv3p[1] - ml_y ) * ( dl_uv3p[1] - ml_y ) + ( dl_uv3p[2] - ml_z ) * ( dl_uv3p[2] - ml_z );

            /* search extremal distance */
            if ( dl_distance > ml_span ) {

                /* update model span */
                ml_span = dl_distance;

            }
            
        }

        /* compute model span */
        ml_span = std::sqrt( ml_span ) * 2.0;

    }

/*
    source - rendering methods
 */

    le_void_t dl_model_t::ml_ren_frame( le_void_t ) {

        /* frame limit variable */
        le_real_t dl_frame1( ml_mdmv );
        le_real_t dl_frame2( ml_mdmv * 2.0 );
        le_real_t dl_frame3( ml_mdmv * 100.0 );
        le_real_t dl_frame4( ml_mdmv * 200.0 );
        le_real_t dl_frame5( ml_span * 0.5 );

        /* primitive bloc */
        glBegin( GL_LINES );

        /* frame color */
        glColor3f( 1.0, 0.0, 0.0 );

        /* frame element */
        glVertex3f( - dl_frame5, 0.0, 0.0 );
        glVertex3f( - dl_frame4, 0.0, 0.0 );
        glVertex3f( - dl_frame3, 0.0, 0.0 );
        glVertex3f( - dl_frame2, 0.0, 0.0 );
        glVertex3f( - dl_frame1, 0.0, 0.0 );
        glVertex3f( + dl_frame1, 0.0, 0.0 );
        glVertex3f( + dl_frame2, 0.0, 0.0 );
        glVertex3f( + dl_frame3, 0.0, 0.0 );
        glVertex3f( + dl_frame4, 0.0, 0.0 );
        glVertex3f( + dl_frame5, 0.0, 0.0 );

        /* frame color */
        glColor3f( 0.0, 1.0, 0.0 );

        /* frame element */
        glVertex3f( 0.0, - dl_frame5, 0.0 );
        glVertex3f( 0.0, - dl_frame4, 0.0 );
        glVertex3f( 0.0, - dl_frame3, 0.0 );
        glVertex3f( 0.0, - dl_frame2, 0.0 );
        glVertex3f( 0.0, - dl_frame1, 0.0 );
        glVertex3f( 0.0, + dl_frame1, 0.0 );
        glVertex3f( 0.0, + dl_frame2, 0.0 );
        glVertex3f( 0.0, + dl_frame3, 0.0 );
        glVertex3f( 0.0, + dl_frame4, 0.0 );
        glVertex3f( 0.0, + dl_frame5, 0.0 );

        /* frame color */
        glColor3f( 0.0, 0.0, 1.0 );

        /* frame element */
        glVertex3f( 0.0, 0.0, - dl_frame5 );
        glVertex3f( 0.0, 0.0, - dl_frame4 );
        glVertex3f( 0.0, 0.0, - dl_frame3 );
        glVertex3f( 0.0, 0.0, - dl_frame2 );
        glVertex3f( 0.0, 0.0, - dl_frame1 );
        glVertex3f( 0.0, 0.0, + dl_frame1 );
        glVertex3f( 0.0, 0.0, + dl_frame2 );
        glVertex3f( 0.0, 0.0, + dl_frame3 );
        glVertex3f( 0.0, 0.0, + dl_frame4 );
        glVertex3f( 0.0, 0.0, + dl_frame5 );

        /* primitive bloc */
        glEnd();

    }

    le_void_t dl_model_t::ml_ren_model( le_void_t ) {

        /* update array state */
        glEnableClientState( GL_VERTEX_ARRAY );
        glEnableClientState( GL_COLOR_ARRAY  );

        /* update array pointer */
        glVertexPointer( 3, GL_DOUBLE, LE_UV3_RECORD, ml_data );

        /* update array pointer */
        glColorPointer ( 3, GL_UNSIGNED_BYTE, LE_UV3_RECORD, ml_data + LE_UV3_POSE + LE_UV3_TYPE );

        /* display model primitive */
        glDrawArrays( GL_POINTS, 0, ml_real );

        /* display model primitive */
        glDrawElements( GL_LINES, ml_count[0], GL_UNSIGNED_INT, ml_index[0] );

        /* display model primitive */
        glDrawElements( GL_TRIANGLES, ml_count[1], GL_UNSIGNED_INT, ml_index[1] );

        /* render model surface */
        ml_surface[2].sf_ren_surface();
        ml_surface[1].sf_ren_surface();
        ml_surface[0].sf_ren_surface();

        /* check display mode */
        if ( ml_hide != 0 ) {

            /* render model surface */
            ml_surface[0].sf_ren_blend();
            ml_surface[1].sf_ren_blend();
            ml_surface[2].sf_ren_blend();

        }

    }

