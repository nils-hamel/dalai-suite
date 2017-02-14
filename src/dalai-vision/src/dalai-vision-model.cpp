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

    dl_model_t::dl_model_t( char * dl_model )

        : ml_size( 0 )
        , ml_data( nullptr )
        , ml_wide( 1 )
        , ml_dsfc( 1 )
        , ml_mean( 0.0 )
        , ml_minx( 0.0 )
        , ml_maxx( 0.0 )
        , ml_miny( 0.0 )
        , ml_maxy( 0.0 )
        , ml_minz( 0.0 )
        , ml_maxz( 0.0 )
        , ml_cenx( 0.0 )
        , ml_ceny( 0.0 )
        , ml_cenz( 0.0 )
        , ml_push( 0 )

    {

        /* input stream variables */
        std::ifstream dl_istream( dl_model, std::ios::binary | std::ios::ate );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( DL_ERROR_IO_ACCESS );

        }

        /* allocate and check buffer memory */
        if ( ( ml_data = new char[ml_size = dl_istream.tellg()] ) == nullptr ) {

            /* send message */
            throw( DL_ERROR_MEMORY );

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* read model bytes */
        dl_istream.read( ml_data, ml_size );

        /* check bytes reading */
        if ( dl_istream.gcount() != ml_size ) {

            /* send message */
            throw( DL_ERROR_IO_READ );

        }

        /* model analysis */
        ml_set_analysis();

        /* assign surface color */
        ml_s[0].sf_set_color( 1.0, 0.0, 0.0 );
        ml_s[1].sf_set_color( 0.0, 1.0, 0.0 );
        ml_s[2].sf_set_color( 0.0, 0.0, 1.0 );

        /* delete input stream */
        dl_istream.close();

    }

    dl_model_t::~dl_model_t() {

        /* check buffer memory */
        if ( ml_data != nullptr ) {

            /* release buffer memory */
            delete [] ml_data;

            /* invalidate pointer */
            ml_data = nullptr;

        }

    }

/*
    source - accessor methods
 */

    double dl_model_t::ml_get_mean( void ) {

        /* return mimimal distance mean */
        return( ml_mean );

    }

    double dl_model_t::ml_get_wideness( void ) {

        /* intervalle variables */
        double dl_dx( ml_maxx - ml_minx );
        double dl_dy( ml_maxy - ml_miny );
        double dl_dz( ml_maxz - ml_minz );

        /* compute and return wideness */
        return( sqrt( dl_dx * dl_dx + dl_dy * dl_dy + dl_dz * dl_dz ) );

    }

    void dl_model_t::ml_get_intersect( void ) {

        /* equation variables */
        double dl_eqa[4];
        double dl_eqb[4];
        double dl_eqc[4];

        /* intersection values */
        double dl_x( 0.0 );
        double dl_y( 0.0 );
        double dl_z( 0.0 );

        /* computation variables */
        double dl_dd( 0.0 );
        double dl_dx( 0.0 );
        double dl_dy( 0.0 );
        double dl_dz( 0.0 );

        /* retrieve surface equation */
        ml_s[0].sf_get_equation( dl_eqa );
        ml_s[1].sf_get_equation( dl_eqb );
        ml_s[2].sf_get_equation( dl_eqc );

        /* compute determinants */
        dl_dd = + dl_eqa[0] * ( dl_eqb[1] * dl_eqc[2] - dl_eqb[2] * dl_eqc[1] )
                - dl_eqa[1] * ( dl_eqb[0] * dl_eqc[2] - dl_eqb[2] * dl_eqc[0] )
                + dl_eqa[2] * ( dl_eqb[0] * dl_eqc[1] - dl_eqb[1] * dl_eqc[0] );
        dl_dx = - dl_eqa[3] * ( dl_eqb[1] * dl_eqc[2] - dl_eqb[2] * dl_eqc[1] )
                + dl_eqa[1] * ( dl_eqb[3] * dl_eqc[2] - dl_eqb[2] * dl_eqc[3] )
                - dl_eqa[2] * ( dl_eqb[3] * dl_eqc[1] - dl_eqb[1] * dl_eqc[3] );
        dl_dy = - dl_eqa[0] * ( dl_eqb[3] * dl_eqc[2] - dl_eqb[2] * dl_eqc[3] )
                + dl_eqa[3] * ( dl_eqb[0] * dl_eqc[2] - dl_eqb[2] * dl_eqc[0] )
                - dl_eqa[2] * ( dl_eqb[0] * dl_eqc[3] - dl_eqb[3] * dl_eqc[0] );
        dl_dz = - dl_eqa[0] * ( dl_eqb[1] * dl_eqc[3] - dl_eqb[3] * dl_eqc[1] )
                + dl_eqa[1] * ( dl_eqb[0] * dl_eqc[3] - dl_eqb[3] * dl_eqc[0] )
                - dl_eqa[3] * ( dl_eqb[0] * dl_eqc[1] - dl_eqb[1] * dl_eqc[0] );

        /* compute intersection */
        dl_x = dl_dx / dl_dd;
        dl_y = dl_dy / dl_dd;
        dl_z = dl_dz / dl_dd;

        /* display intersection */
        std::cerr << dl_x << " " << dl_y << " " << dl_z << std::endl;

    }

/*
    source - mutator methods
 */

    void dl_model_t::ml_set_center( double const dl_x, double const dl_y, double const dl_z ) {

        /* assign model pseudo-center */
        ml_cenx = dl_x;
        ml_ceny = dl_y;
        ml_cenz = dl_z;

    }

    void dl_model_t::ml_set_active( long long int const dl_active ) {

        /* check consistency */
        if ( ( dl_active < 0 ) || ( dl_active > 2 ) ) {

            /* send message */
            throw( DL_ERROR_PARAMS );

        }

        /* assign active surface */
        ml_push = dl_active;

    }

    void dl_model_t::ml_set_switch( void ) {

        /* assign display flag */
        ml_dsfc = 1 - ml_dsfc;

    }

    void dl_model_t::ml_set_push( void ) {

        /* push current center */
        ml_s[ml_push].sf_set_point_push( ml_cenx, ml_ceny, ml_cenz, ml_mean );

    }

    void dl_model_t::ml_set_clear( void ) {

        /* clear surface */
        ml_s[ml_push].sf_set_point_clear();

    }

    void dl_model_t::ml_set_wide( long long int const dl_wide ) {

        /* assign model point size */
        ml_wide = dl_wide;

    }

    void dl_model_t::ml_set_autopoint( void ) {

        /* automatic resampling */
        ml_s[ml_push].sf_set_point_auto( ml_data, ml_size, 2 * ml_mean );

    }

    void dl_model_t::ml_set_analysis( void ) {

        /* distance variables */
        double dl_distance( 0.0 );

        /* distances array variables */
        double * dl_array( nullptr );

        /* buffer mapping variables */
        double * dl_posea( nullptr );
        double * dl_poseb( nullptr );

        /* allocate and check array memory */
        if ( ( dl_array = new double[DL_MEAN_COUNT] ) == nullptr ) {

            /* send message */
            throw( DL_ERROR_MEMORY );

        }

        /* initialise model edges */
        ml_minx = + std::numeric_limits<double>::max();
        ml_maxx = - std::numeric_limits<double>::max();
        ml_miny = + std::numeric_limits<double>::max();
        ml_maxy = - std::numeric_limits<double>::max();
        ml_minz = + std::numeric_limits<double>::max();
        ml_maxz = - std::numeric_limits<double>::max();

        /* initilaise distances array */
        for ( long long int dl_parse( 0 ); dl_parse < DL_MEAN_COUNT; dl_parse ++ ) {

            /* initialise distance */
            dl_array[dl_parse] = std::numeric_limits<double>::max();

        }

        /* parsing model elements */
        for ( long long int dl_parse( 0 ); dl_parse < ml_size; dl_parse += 27 ) {

            /* compute array mapping */
            dl_posea = ( double * ) ( ml_data + dl_parse );

            /* search extremums */
            if ( ml_minx > dl_posea[0] ) ml_minx = dl_posea[0];
            if ( ml_maxx < dl_posea[0] ) ml_maxx = dl_posea[0];
            if ( ml_miny > dl_posea[1] ) ml_miny = dl_posea[1];
            if ( ml_maxy < dl_posea[1] ) ml_maxy = dl_posea[1];
            if ( ml_minz > dl_posea[2] ) ml_minz = dl_posea[2];
            if ( ml_maxz < dl_posea[2] ) ml_maxz = dl_posea[2];

            /* parsing model elements */
            for ( long long int dl_index( 0 ); dl_index < DL_MEAN_COUNT; dl_index ++ ) {

                /* compute array mapping */
                dl_poseb = ( double * ) ( ml_data + dl_index * ( ( ml_size / 27 ) / DL_MEAN_COUNT ) * 27 );

                /* compute distance */
                dl_distance = ( dl_posea[0] - dl_poseb[0] ) * ( dl_posea[0] - dl_poseb[0] ) +
                              ( dl_posea[1] - dl_poseb[1] ) * ( dl_posea[1] - dl_poseb[1] ) +
                              ( dl_posea[2] - dl_poseb[2] ) * ( dl_posea[2] - dl_poseb[2] );

                /* search minimum distance */
                if ( ( dl_distance > 0.0 ) && ( dl_distance < dl_array[dl_index] ) ) dl_array[dl_index] = dl_distance;

            }

        }

        /* reset model minimum mean */
        ml_mean = 0.0;

        /* compute minimum mean */
        for ( long long int dl_parse( 0 ); dl_parse < DL_MEAN_COUNT; dl_parse ++ ) {

            /* accumulate minimum distances */
            ml_mean += sqrt( dl_array[dl_parse] );

        }

        /* terminate and assign minimum mean computation */
        ml_mean /= double( DL_MEAN_COUNT );

        /* assign mean as surface tolerence */
        //ml_s[0].sf_set_tolerence( ml_mean );
        //ml_s[1].sf_set_tolerence( ml_mean );
        //ml_s[2].sf_set_tolerence( ml_mean );

        /* compute model pseudo-center */
        ml_cenx = 0.5 * ( ml_maxx + ml_minx );
        ml_ceny = 0.5 * ( ml_maxy + ml_miny );
        ml_cenz = 0.5 * ( ml_maxz + ml_minz );

        /* release array memory */
        delete [] dl_array;

    }

/*
    source - rendering methods
 */

    void dl_model_t::ml_ren_model( void ) {

        /* model center */
        glTranslatef( -ml_cenx, -ml_ceny, -ml_cenz );

        /* disable blending */
        glDisable( GL_BLEND );

        /* set point size */
        glPointSize( ml_wide );

        /* update opengl array state */
        glEnableClientState( GL_VERTEX_ARRAY );
        glEnableClientState( GL_COLOR_ARRAY  );

        /* configure opengl arrays */
        glVertexPointer( 3, DL_GLARRAY_VERTEX, DL_GLARRAY_STRIPE, ml_data + DL_GLARRAY_BASE_V );
        glColorPointer ( 3, DL_GLARRAY_COLORS, DL_GLARRAY_STRIPE, ml_data + DL_GLARRAY_BASE_C );

        /* display opengl arrays content - points */
        glDrawArrays( GL_POINTS, 0, ml_size / 27 );

    }

    void dl_model_t::ml_ren_surface( void ) {

        /* model center */
        glTranslatef( -ml_cenx, -ml_ceny, -ml_cenz );

        /* disable blending */
        glDisable( GL_BLEND );

        /* display surface points */
        ml_s[0].sf_ren_point( ml_wide << 1 );
        ml_s[1].sf_ren_point( ml_wide << 1 );
        ml_s[2].sf_ren_point( ml_wide << 1 );

        /* check display flag */
        if ( ml_dsfc == 1 ) {

            /* enable blending */
            glEnable( GL_BLEND );

            /* display surfaces */
            ml_s[0].sf_ren_surface( ml_get_wideness() / 2.0 );
            ml_s[1].sf_ren_surface( ml_get_wideness() / 2.0 );
            ml_s[2].sf_ren_surface( ml_get_wideness() / 2.0 );

        }

    }

    void dl_model_t::ml_ren_frame( void ) {

        /* scale variables */
        double dl_lscale( ml_get_mean() );
        double dl_uscale( ml_get_wideness() );

        /* frame position variables */
        double dl_fpos1( dl_lscale );
        double dl_fpos2( dl_lscale * 2 );
        double dl_fpos3( dl_lscale * 100 );
        double dl_fpos4( dl_lscale * 110 );
        double dl_fpos5( dl_uscale / 2.0 );

        /* disable blending */
        glDisable( GL_BLEND );

        /* set line width */
        glLineWidth( ml_wide );

        /* primitive - frame */
        glBegin( GL_LINES );

            /* frame color */
            glColor3f( 1.0, 0.0, 0.0 );

            /* frame element */
            glVertex3f( -dl_fpos5, 0.0, 0.0 );
            glVertex3f( -dl_fpos4, 0.0, 0.0 );
            glVertex3f( -dl_fpos3, 0.0, 0.0 );
            glVertex3f( -dl_fpos2, 0.0, 0.0 );
            glVertex3f( -dl_fpos1, 0.0, 0.0 );
            glVertex3f( +dl_fpos1, 0.0, 0.0 );
            glVertex3f( +dl_fpos2, 0.0, 0.0 );
            glVertex3f( +dl_fpos3, 0.0, 0.0 );
            glVertex3f( +dl_fpos4, 0.0, 0.0 );
            glVertex3f( +dl_fpos5, 0.0, 0.0 );

            /* frame color */
            glColor3f( 0.0, 1.0, 0.0 );

            /* frame element */
            glVertex3f( 0.0, -dl_fpos5, 0.0 );
            glVertex3f( 0.0, -dl_fpos4, 0.0 );
            glVertex3f( 0.0, -dl_fpos3, 0.0 );
            glVertex3f( 0.0, -dl_fpos2, 0.0 );
            glVertex3f( 0.0, -dl_fpos1, 0.0 );
            glVertex3f( 0.0, +dl_fpos1, 0.0 );
            glVertex3f( 0.0, +dl_fpos2, 0.0 );
            glVertex3f( 0.0, +dl_fpos3, 0.0 );
            glVertex3f( 0.0, +dl_fpos4, 0.0 );
            glVertex3f( 0.0, +dl_fpos5, 0.0 );

            /* frame color */
            glColor3f( 0.0, 0.0, 1.0 );

            /* frame element */
            glVertex3f( 0.0, 0.0, -dl_fpos5 );
            glVertex3f( 0.0, 0.0, -dl_fpos4 );
            glVertex3f( 0.0, 0.0, -dl_fpos3 );
            glVertex3f( 0.0, 0.0, -dl_fpos2 );
            glVertex3f( 0.0, 0.0, -dl_fpos1 );
            glVertex3f( 0.0, 0.0, +dl_fpos1 );
            glVertex3f( 0.0, 0.0, +dl_fpos2 );
            glVertex3f( 0.0, 0.0, +dl_fpos3 );
            glVertex3f( 0.0, 0.0, +dl_fpos4 );
            glVertex3f( 0.0, 0.0, +dl_fpos5 );

        glEnd();

    }

