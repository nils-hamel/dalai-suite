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

    # include "dalai-vision-model.hpp"

/*
    source - constructor/destructor methods
 */

    dl_model_c::dl_model_c()

        : ml_size( 0 )
        , ml_data( nullptr )
        , ml_minx( 0.0 )
        , ml_maxx( 0.0 )
        , ml_miny( 0.0 )
        , ml_maxy( 0.0 )
        , ml_minz( 0.0 )
        , ml_maxz( 0.0 )
        , ml_cenx( 0.0 )
        , ml_ceny( 0.0 )
        , ml_cenz( 0.0 )

    {}

    dl_model_c::dl_model_c( char * ml_model )

        : ml_size( 0 )
        , ml_data( nullptr )
        , ml_minx( 0.0 )
        , ml_maxx( 0.0 )
        , ml_miny( 0.0 )
        , ml_maxy( 0.0 )
        , ml_minz( 0.0 )
        , ml_maxz( 0.0 )
        , ml_cenx( 0.0 )
        , ml_ceny( 0.0 )
        , ml_cenz( 0.0 )

    {

        /* status variables */
        bool ml_status( true );

        /* input stream variables */
        std::ifstream ml_istream( ml_model, std::ios::binary | std::ios::ate );

        /* check input stream */
        if ( ml_istream.is_open() == false ) {

            /* push nessage */
            ml_status = false;

        } else {

            /* retrieve model size */
            ml_size = ml_istream.tellg();

            /* allocate and check buffer memory */
            if ( ( ml_data = new char[ml_size] ) == nullptr ) {

                /* push message */
                ml_status = false;

            } else {

                /* clear input stream */
                ml_istream.clear();

                /* reset input stream */
                ml_istream.seekg( 0 );

                /* read model bytes */
                ml_istream.read( ml_data, ml_size );

                /* check model bytes reading */
                if ( ml_istream.gcount() != ml_size ) {

                    /* push message */
                    ml_status = false;

                } else {

                    /* model analysis */
                    ml_set_edges();

                }

            }

            /* delete input stream */
            ml_istream.close();

        }

        /* check status */
        if ( ml_status == false ) throw( 1 );

    }

    dl_model_c::~dl_model_c() {

        /* check model state */
        if ( ml_size > 0 ) {

            /* release model memory */
            delete [] ml_data;

        }

    }

/*
    source - accessor methods
 */

    double dl_model_c::ml_get_diag( void ) {

        /* compute characteristic size of the model */
        return( sqrt( ( ml_maxx - ml_minx ) * ( ml_maxx - ml_minx ) + ( ml_maxy - ml_miny ) * ( ml_maxy - ml_miny ) + ( ml_maxz - ml_minz ) * ( ml_maxz - ml_minz ) ) );

    }

/*
    source - mutator methods
 */

    void dl_model_c::ml_set_center( double ml_x, double ml_y, double ml_z ) {

        /* assign model pseudo-center */
        ml_cenx = ml_x;
        ml_ceny = ml_y;
        ml_cenz = ml_z;

    }

    void dl_model_c::ml_set_edges( void ) {

        /* buffer mapping variables */
        double * dl_pose( nullptr );

        /* initialise edges */
        ml_minx = std::numeric_limits<double>::max();
        ml_maxx = std::numeric_limits<double>::min();
        ml_miny = std::numeric_limits<double>::max();
        ml_maxy = std::numeric_limits<double>::min();
        ml_minz = std::numeric_limits<double>::max();
        ml_maxz = std::numeric_limits<double>::min();

        /* parsing model elements */
        for ( long long int dl_parse( 0 ); dl_parse < ml_size; dl_parse += 27 ) {

            /* compute array mapping */
            dl_pose = ( double * ) ( ml_data + dl_parse );

            /* search extremums */
            if ( ml_minx > dl_pose[0] ) ml_minx = dl_pose[0];
            if ( ml_maxx < dl_pose[0] ) ml_maxx = dl_pose[0];
            if ( ml_miny > dl_pose[1] ) ml_miny = dl_pose[1];
            if ( ml_maxy < dl_pose[1] ) ml_maxy = dl_pose[1];
            if ( ml_minz > dl_pose[2] ) ml_minz = dl_pose[2];
            if ( ml_maxz < dl_pose[2] ) ml_maxz = dl_pose[2];

        }

        /* compute model pseudo-center */
        ml_cenx = 0.5 * ( ml_maxx + ml_minx );
        ml_ceny = 0.5 * ( ml_maxy + ml_miny );
        ml_cenz = 0.5 * ( ml_maxz + ml_minz );

    }

    void dl_model_c::ml_set_actp( double ml_x, double ml_y, double ml_z ) {

        /* assign active point */
        ml_actp[0] = ml_x;
        ml_actp[1] = ml_y;
        ml_actp[2] = ml_z;

    }

/*
    source - rendering methods
 */

    void dl_model_c::ml_display_model( void ) {

        /* model centering translations */
        glTranslatef( -ml_cenx, -ml_ceny, -ml_cenz );

        /* configure opengl arrays */
        glVertexPointer( 3, DL_GLARRAY_VERTEX, DL_GLARRAY_STRIPE, ml_data + DL_GLARRAY_BASE_V );
        glColorPointer ( 3, DL_GLARRAY_COLORS, DL_GLARRAY_STRIPE, ml_data + DL_GLARRAY_BASE_C );

        /* display opengl arrays content - points */
        glDrawArrays( GL_POINTS, 0, ml_size / 27 );

        /* primitive - active point */
        glBegin( GL_POINTS );

            /* active color */
            glColor3f( 1.0, 0.0, 0.0 );

            /* display active point */
            glVertex3f( ml_actp[0], ml_actp[1], ml_actp[2] );

        glEnd();

    }

    void dl_model_c::ml_display_frame( void ) {

        /* primitive - frame */
        glBegin( GL_LINES );

            /* frame color */
            glColor3f( 1.0, 0.0, 0.0 );

            /* frame element */
            glVertex3f( -1.0, 0.0, 0.0 );
            glVertex3f( +1.0, 0.0, 0.0 );

            /* frame color */
            glColor3f( 0.0, 1.0, 0.0 );

            /* frame element */
            glVertex3f( 0.0, -1.0, 0.0 );
            glVertex3f( 0.0, +1.0, 0.0 );

            /* frame color */
            glColor3f( 0.0, 0.0, 1.0 );

            /* frame element */
            glVertex3f( 0.0, 0.0, -1.0 );
            glVertex3f( 0.0, 0.0, +1.0 );

        glEnd();

    }


