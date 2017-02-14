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

    /*! \file   dalai-vision-model.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - vision - model module
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_VISION_MODEL__
    # define __DL_VISION_MODEL__

/*
    header - internal includes
 */

    # include "dalai-vision-common.hpp"
    # include "dalai-vision-surface.hpp"

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <limits>
    # include <cmath>
    # include <GL/gl.h>
    # include <GL/glu.h>
    # include <Eigen/Dense>
    # include <common-include.h>

/*
    header - preprocessor definitions
 */

    /* define array mean estimation count */
    # define DL_MEAN_COUNT     ( 32 )

    /* define opengl arrays base pointer */
    # define DL_GLARRAY_BASE_V ( 0 )
    # define DL_GLARRAY_BASE_C ( sizeof( double ) * 3 )

    /* define opengl arrays stripe */
    # define DL_GLARRAY_STRIPE ( ( sizeof( double ) + sizeof( char ) ) * 3 )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

    class dl_model_t {

    private:

        long long int ml_size;
        char *        ml_data;
        long long int ml_wide;
        long long int ml_dsfc;

        double        ml_mean;

        double        ml_minx;
        double        ml_maxx;
        double        ml_miny;
        double        ml_maxy;
        double        ml_minz;
        double        ml_maxz;

        double        ml_cenx;
        double        ml_ceny;
        double        ml_cenz;

        long long int ml_push;
        dl_surface_t  ml_s[3];

    public:
        dl_model_t( char * ml_model );
        ~dl_model_t();

    public:
        double ml_get_mean( void );
        double ml_get_wideness( void );
        void ml_get_intersect( void );

    public:
        void ml_set_center( double const dl_x, double const dl_y, double const dl_z );
        void ml_set_active( long long int const dl_active );
        void ml_set_switch( void );
        void ml_set_push( void );
        void ml_set_clear( void );
        void ml_set_wide( long long int const dl_wide );
        void ml_set_autopoint( void );

    private:

        void ml_set_analysis( void );

    public:

        void ml_ren_model( void );
        void ml_ren_surface( void );
        void ml_ren_frame( void );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

