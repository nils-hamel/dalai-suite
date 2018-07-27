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

    /*! \file   dalai-vision-arcball.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - vision - arcball module
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_VISION_ARCBALL__
    # define __DL_VISION_ARCBALL__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <cstring>
    # include <GL/gl.h>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

    class dl_arcball_t {

    private:

        le_size_t ab_width;
        le_size_t ab_height;
        le_real_t ab_size;
        le_real_t ab_view[16];

    public:

        /* *** */

        dl_arcball_t( le_size_t const dl_width, le_size_t dl_height ) ;

        /* *** */

        le_void_t ab_get_rotate( le_void_t );

    private:

        /* *** */

        le_void_t ab_set_identity( le_void_t );

        /* *** */

        le_void_t ab_set_multiply( le_real_t * dl_matrix );

    public:

        /* *** */

        le_void_t ab_set_update( le_size_t const dl_init_x, le_size_t const dl_init_y, le_size_t const dl_step_x, le_size_t const dl_step_y );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

