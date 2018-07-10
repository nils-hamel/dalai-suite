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

    dl_model_t::dl_model_t( char * dl_model )

        : ml_size( 0 )
        , ml_data( nullptr )

        , ml_psize( 1 )
        , ml_sflag( 1 )
        , ml_pflag( 1 )

        , ml_xmin( 0.0 )
        , ml_xmax( 0.0 )
        , ml_ymin( 0.0 )
        , ml_ymax( 0.0 )
        , ml_zmin( 0.0 )
        , ml_zmax( 0.0 )

        , ml_mdmv( 0.0 )

        , ml_xcen( 0.0 )
        , ml_ycen( 0.0 )
        , ml_zcen( 0.0 )

        , ml_sact( 0 )

    {

        /* input stream variables */
        std::ifstream dl_istream( dl_model, std::ios::binary | std::ios::ate );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* allocate and check buffer memory */
        if ( ( ml_data = new char[ml_size = dl_istream.tellg()] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

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
            throw( LC_ERROR_IO_READ );

        }

        /* model analysis */
        ml_set_analysis();

        /* delete input stream */
        dl_istream.close();

        /* create surface colors */
        ml_s[0].sf_set_color( 1.0, 0.2, 0.2 );
        ml_s[1].sf_set_color( 0.2, 1.0, 0.2 );
        ml_s[2].sf_set_color( 0.2, 0.2, 1.0 );

    }

    dl_model_t::~dl_model_t() {

        /* check buffer memory */
        if ( ml_data != nullptr ) {

            /* release buffer memory */
            delete [] ml_data;

            /* invalidate buffer pointer */
            ml_data = nullptr;

        }

    }

/*
    source - accessor methods
 */

    double dl_model_t::ml_get_mdmv( void ) {

        /* minimal distance mean value */
        return( ml_mdmv );

    }

    double dl_model_t::ml_get_span( void ) {

        /* model range variables */
        double dl_xspan( ml_xmax - ml_xmin );
        double dl_yspan( ml_ymax - ml_ymin );
        double dl_zspan( ml_zmax - ml_zmin );

        /* compute and return model span */
        return( sqrt( dl_xspan * dl_xspan + dl_yspan * dl_yspan + dl_zspan * dl_zspan ) );

    }

    void dl_model_t::ml_get_intersect( void ) {

        /* surface equation variables */
        double dl_seqa[4] = { 0.0 };
        double dl_seqb[4] = { 0.0 };
        double dl_seqc[4] = { 0.0 };

        /* intersection coordinates variables */
        double dl_xint( 0.0 );
        double dl_yint( 0.0 );
        double dl_zint( 0.0 );

        /* surfaces determinant variables */
        double dl_sdet( 0.0 );

        /* retreive surface equation */
        ml_s[0].sf_get_equation( dl_seqa );
        ml_s[1].sf_get_equation( dl_seqb );
        ml_s[2].sf_get_equation( dl_seqc );

        /* compute surfaces determinant */
        dl_sdet += dl_seqa[0] * ( dl_seqb[1] * dl_seqc[2] - dl_seqb[2] * dl_seqc[1] );
        dl_sdet -= dl_seqa[1] * ( dl_seqb[0] * dl_seqc[2] - dl_seqb[2] * dl_seqc[0] );
        dl_sdet += dl_seqa[2] * ( dl_seqb[0] * dl_seqc[1] - dl_seqb[1] * dl_seqc[0] );

        /* compute intersection coordinates */
        dl_xint -= dl_seqa[3] * ( dl_seqb[1] * dl_seqc[2] - dl_seqb[2] * dl_seqc[1] );
        dl_xint += dl_seqa[1] * ( dl_seqb[3] * dl_seqc[2] - dl_seqb[2] * dl_seqc[3] );
        dl_xint -= dl_seqa[2] * ( dl_seqb[3] * dl_seqc[1] - dl_seqb[1] * dl_seqc[3] );

        /* compute intersection coordinates */
        dl_yint -= dl_seqa[0] * ( dl_seqb[3] * dl_seqc[2] - dl_seqb[2] * dl_seqc[3] );
        dl_yint += dl_seqa[3] * ( dl_seqb[0] * dl_seqc[2] - dl_seqb[2] * dl_seqc[0] );
        dl_yint -= dl_seqa[2] * ( dl_seqb[0] * dl_seqc[3] - dl_seqb[3] * dl_seqc[0] );

        /* compute intersection coordinates */
        dl_zint -= dl_seqa[0] * ( dl_seqb[1] * dl_seqc[3] - dl_seqb[3] * dl_seqc[1] );
        dl_zint += dl_seqa[1] * ( dl_seqb[0] * dl_seqc[3] - dl_seqb[3] * dl_seqc[0] );
        dl_zint -= dl_seqa[3] * ( dl_seqb[0] * dl_seqc[1] - dl_seqb[1] * dl_seqc[0] );

        /* export intersection coordinates */
        std::cerr << dl_xint / dl_sdet << " " << dl_yint / dl_sdet << " " << dl_zint / dl_sdet << std::endl;

    }

/*
    source - mutator methods
 */

    void dl_model_t::ml_set_center( double const dl_x, double const dl_y, double const dl_z ) {

        /* model center */
        ml_xcen = dl_x;
        ml_ycen = dl_y;
        ml_zcen = dl_z;

    }

    void dl_model_t::ml_set_surface( long long const dl_surface ) {

        /* check consistency */
        if ( ( dl_surface < 0 ) || ( dl_surface > 2 ) ) {

            /* send message */
            throw( LC_ERROR_DOMAIN );

        }

        /* assign active surface */
        ml_sact = dl_surface;

    }

    void dl_model_t::ml_set_surface_switch( void ) {

        /* update display flag */
        ml_sflag = 1 - ml_sflag;

    }

    void dl_model_t::ml_set_pointsize( long long const dl_pointsize ) {

        /* update display point size */
        ml_psize = dl_pointsize;

    }

    void dl_model_t::ml_set_point_push( void ) {

        /* push current center */
        ml_s[ml_sact].sf_set_point_push( ml_xcen, ml_ycen, ml_zcen, ml_mdmv );

    }

    void dl_model_t::ml_set_point_auto( void ) {

        /* automatic resampling */
        ml_s[ml_sact].sf_set_point_auto( ml_data, ml_size, 2 * ml_mdmv );

    }

    void dl_model_t::ml_set_point_clear( void ) {

        /* clear surface */
        ml_s[ml_sact].sf_set_point_clear();

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
        if ( ( dl_array = new double[DL_MODEL_MDMVC] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* initialise model edges */
        ml_xmin = + std::numeric_limits<double>::max();
        ml_xmax = - std::numeric_limits<double>::max();
        ml_ymin = + std::numeric_limits<double>::max();
        ml_ymax = - std::numeric_limits<double>::max();
        ml_zmin = + std::numeric_limits<double>::max();
        ml_zmax = - std::numeric_limits<double>::max();

        /* initilaise distances array */
        for ( long long int dl_parse( 0 ); dl_parse < DL_MODEL_MDMVC; dl_parse ++ ) {

            /* initialise distance */
            dl_array[dl_parse] = std::numeric_limits<double>::max();

        }

        ml_gsize[0] = 0;
        ml_gsize[1] = 0;
        ml_gsize[2] = 0;

        /* parsing model elements */
        //for ( long long int dl_parse( 0 ); dl_parse < ml_size; dl_parse += 27 ) {
        for ( long long int dl_parse( 0 ); dl_parse < ml_size; dl_parse += LE_UV3_RECORD ) {

            /* compute array mapping */
            dl_posea = ( double * ) ( ml_data + dl_parse );

            /* search extremums */
            if ( ml_xmin > dl_posea[0] ) ml_xmin = dl_posea[0];
            if ( ml_xmax < dl_posea[0] ) ml_xmax = dl_posea[0];
            if ( ml_ymin > dl_posea[1] ) ml_ymin = dl_posea[1];
            if ( ml_ymax < dl_posea[1] ) ml_ymax = dl_posea[1];
            if ( ml_zmin > dl_posea[2] ) ml_zmin = dl_posea[2];
            if ( ml_zmax < dl_posea[2] ) ml_zmax = dl_posea[2];

            /* parsing model elements */
            for ( long long int dl_index( 0 ); dl_index < DL_MODEL_MDMVC; dl_index ++ ) {

                /* compute array mapping */
                //dl_poseb = ( double * ) ( ml_data + dl_index * ( ( ml_size / 27 ) / DL_MODEL_MDMVC ) * 27 );
                dl_poseb = ( double * ) ( ml_data + dl_index * ( ( ml_size / LE_UV3_RECORD ) / DL_MODEL_MDMVC ) * LE_UV3_RECORD );

                /* compute distance */
                dl_distance = ( dl_posea[0] - dl_poseb[0] ) * ( dl_posea[0] - dl_poseb[0] ) +
                              ( dl_posea[1] - dl_poseb[1] ) * ( dl_posea[1] - dl_poseb[1] ) +
                              ( dl_posea[2] - dl_poseb[2] ) * ( dl_posea[2] - dl_poseb[2] );

                /* search minimum distance */
                if ( ( dl_distance > 0.0 ) && ( dl_distance < dl_array[dl_index] ) ) dl_array[dl_index] = dl_distance;

            }

            ml_gsize[( * ( ( le_byte_t * ) ( dl_posea + 3 ) ) ) - 1] ++;

        }

        ml_gdata[0] = new GLuint[ml_gsize[0]];
        ml_gdata[1] = new GLuint[ml_gsize[1]];
        ml_gdata[2] = new GLuint[ml_gsize[2]];

        ml_gsize[0] = 0;
        ml_gsize[1] = 0;
        ml_gsize[2] = 0;

        for ( long long int dl_parse( 0 ); dl_parse < ml_size; dl_parse += LE_UV3_RECORD ) {

            int dl_prime = ( * ( ( le_byte_t * ) ( dl_posea + 3 ) ) ) - 1;

            ml_gdata[dl_prime][ml_gsize[dl_prime]++] = dl_parse / LE_UV3_RECORD;

        }

        /* reset model minimum mean */
        ml_mdmv = 0.0;

        /* compute minimum mean */
        for ( long long int dl_parse( 0 ); dl_parse < DL_MODEL_MDMVC; dl_parse ++ ) {

            /* accumulate minimum distances */
            ml_mdmv += sqrt( dl_array[dl_parse] );

        }

        /* terminate and assign minimum mean computation */
        ml_mdmv /= double( DL_MODEL_MDMVC );

        /* compute model pseudo-center */
        ml_xcen = 0.5 * ( ml_xmax + ml_xmin );
        ml_ycen = 0.5 * ( ml_ymax + ml_ymin );
        ml_zcen = 0.5 * ( ml_zmax + ml_zmin );

        /* release array memory */
        delete [] dl_array;

    }

/*
    source - rendering methods
 */

    void dl_model_t::ml_ren_model( void ) {

        /* model translation */
        glTranslated( -ml_xcen, -ml_ycen, -ml_zcen );

        /* disable blending */
        glDisable( GL_BLEND );

        /* set point size */
        glPointSize( ml_psize );

        /* update opengl array state */
        glEnableClientState( GL_VERTEX_ARRAY );
        glEnableClientState( GL_COLOR_ARRAY  );

        /* configure opengl arrays */
        //glVertexPointer( 3, DL_MODEL_V_TYPE, DL_MODEL_STRIPE, ml_data + DL_MODEL_V_BASE );
        //glColorPointer ( 3, DL_MODEL_C_TYPE, DL_MODEL_STRIPE, ml_data + DL_MODEL_C_BASE );

        /* reference to vertex array */
        glVertexPointer( 3, GL_DOUBLE, LE_UV3_RECORD, ml_data );

        /* reference to color array */
        glColorPointer ( 3, GL_UNSIGNED_BYTE, LE_UV3_RECORD, ml_data + LE_UV3_POSE + LE_UV3_TYPE );

        /* display opengl arrays content - points */
        //glDrawArrays( GL_POINTS, 0, ml_size / 27 );
        //glDrawArrays( GL_POINTS, 0, ml_size / LE_UV3_RECORD );

        glDrawElements( GL_POINTS, ml_gsize[0], GL_UNSIGNED_INT, ml_gdata[0] );

        glDrawElements( GL_LINES, ml_gsize[1], GL_UNSIGNED_INT, ml_gdata[1] );

        glDrawElements( GL_TRIANGLES, ml_gsize[2], GL_UNSIGNED_INT, ml_gdata[2] );        

        //glDrawElements( GL_TRIANGLES, ml_tsize, GL_UNSIGNED_INT, ml_tprim );

    }

    void dl_model_t::ml_ren_surface( void ) {

        /* model translation */
        glTranslated( -ml_xcen, -ml_ycen, -ml_zcen );

        /* check display flag */
        if ( ml_pflag != 0 ) {

            /* disable blending */
            glDisable( GL_BLEND );

            /* opengl primitive properties */
            glPointSize( ml_psize * 2 );

            /* display surface points */
            ml_s[0].sf_ren_point();
            ml_s[1].sf_ren_point();
            ml_s[2].sf_ren_point();

        }

        /* check display flag */
        if ( ml_sflag != 0 ) {

            /* enable blending */
            glEnable( GL_BLEND );

            /* display surfaces */
            ml_s[0].sf_ren_surface( ml_get_span() / 2.0 );
            ml_s[1].sf_ren_surface( ml_get_span() / 2.0 );
            ml_s[2].sf_ren_surface( ml_get_span() / 2.0 );

        }

    }

    void dl_model_t::ml_ren_frame( void ) {

        /* scale variables */
        double dl_mdmv( ml_get_mdmv() );
        double dl_span( ml_get_span() );

        /* frame position variables */
        double dl_frame1( dl_mdmv );
        double dl_frame2( dl_mdmv * 2 );
        double dl_frame3( dl_mdmv * 100 );
        double dl_frame4( dl_mdmv * 110 );
        double dl_frame5( dl_span / 2.0 );

        /* disable blending */
        glDisable( GL_BLEND );

        /* opengl primitive properties */
        glLineWidth( ml_psize );

        /* opengl primitives */
        glBegin( GL_LINES );

            /* frame color */
            glColor3f( 1.0, 0.0, 0.0 );

            /* frame elements */
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

            /* frame elements */
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

            /* frame elements */
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

        /* opengl primitives */
        glEnd();

    }

