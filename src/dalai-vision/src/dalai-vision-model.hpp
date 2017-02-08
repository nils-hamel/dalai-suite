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

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <limits>
    # include <cmath>
    # include <GL/gl.h>
    # include <GL/glu.h>
    # include <common-include.h>

/*
    header - preprocessor definitions
 */

    /* define opengl arrays types */
    # define DL_GLARRAY_VERTEX ( GL_DOUBLE )
    # define DL_GLARRAY_COLORS ( GL_UNSIGNED_BYTE )

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

    class dl_model_c {

    private:

        long long int ml_size;
        char *        ml_data;

        double        ml_minx;
        double        ml_maxx;
        double        ml_miny;
        double        ml_maxy;
        double        ml_minz;
        double        ml_maxz;

        double        ml_cenx;
        double        ml_ceny;
        double        ml_cenz;

        double        ml_actp[3];

    public:

        dl_model_c();
        dl_model_c( char * ml_model );
        ~dl_model_c();

        double ml_get_diag( void );

        void ml_set_center( double dl_x, double dl_y, double dl_z );

    private:

        void ml_set_edges( void );

    public:

        void ml_set_actp( double ml_x, double ml_y, double ml_z );

        void ml_display_model( void );
        void ml_display_frame( void );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

