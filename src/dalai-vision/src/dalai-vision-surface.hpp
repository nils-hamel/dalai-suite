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
     *  dalai-suite - vision - surface module
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_VISION_SURFACE__
    # define __DL_VISION_SURFACE__

/*
    header - internal includes
 */

    # include "dalai-vision-common.hpp"

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <cstdlib>
    # include <limits>
    # include <cmath>
    # include <GL/gl.h>
    # include <GL/glu.h>
    # include <Eigen/Dense>
    # include <common-include.h>

/*
    header - preprocessor definitions
 */

    /* define memory allocation steps */
    # define DL_SURFACE_STEP ( 8192 )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

    class dl_surface_t {

    private:
        double    sf_cx;
        double    sf_cy;
        double    sf_cz;

        double    sf_px;
        double    sf_py;
        double    sf_pz;
        double    sf_pc;

        double    sf_ux;
        double    sf_uy;
        double    sf_uz;
        double    sf_vx;
        double    sf_vy;
        double    sf_vz;

        float     sf_cr;
        float     sf_cg;
        float     sf_cb;

        long long sf_size;
        long long sf_virt;
        double *  sf_data;

    public:
        dl_surface_t( void );
        ~dl_surface_t( void );

    public:
        void sf_get_equation( double dl_equation[4] );

    public:
        void sf_set_point_push( double const dl_x, double const dl_y, double const dl_z, double const dl_limit );
        void sf_set_point_auto( char const * const dl_data, long long const dl_size, double const dl_limit );
        void sf_set_point_clear( void );

    private:
        bool sf_set_point_remove( double const dl_x, double const dl_y, double const dl_z, double const dl_limit );

    public:
        void sf_set_color( float const dl_cr, float const dl_cg, float const dl_cb );
        void sf_set_tolerence( double const dl_tolerence );

    private:
        void sf_set_equation( void );
        void sf_set_memory( void );
        void sf_set_release( void );

    public:
        void sf_ren_surface( double const dl_w );
        void sf_ren_point( void );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

