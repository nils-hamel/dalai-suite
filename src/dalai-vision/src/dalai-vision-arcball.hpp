/*
 *  dalai-suite - vision
 *
 *      Nils Hamel - nils.hamel@alumni.epfl.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
 *      Copyright (c) 2020 Republic and Canton of Geneva
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
     *  dalai-suite - vision - arcball
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

    /*! \class dl_arcball_t
     *  \brief Arcball class
     *
     *  This class implements a standard and simple model of arcball allowing
     *  to track mouse motion and to update model point of view.
     *
     *  The class stores basic information about the interface size and holds
     *  the current state of the arcball matrix applied to rendering matrix.
     *
     *  \var dl_arcball_t::ab_width
     *  Interface width, in pixels
     *  \var dl_arcball_t::ab_height
     *  Interface height, in pixels
     *  \var dl_arcball_t::ab_size
     *  Maximum value of interface width and height
     *  \var dl_arcball_t::ab_view
     *  Arcball rotation matrix
     */

    class dl_arcball_t {

    private:

        le_size_t ab_width;
        le_size_t ab_height;
        le_real_t ab_size;
        le_real_t ab_view[16];

    public:

        /*! \brief constructor/destructor methods
         *
         *  The constructor simply initialises members with default values and
         *  sets the arcball matrix to identity.
         *
         *  \param dl_width  Interface width, in pixels
         *  \param dl_height Interface height, in pixels
         */

        dl_arcball_t( le_size_t const dl_width, le_size_t dl_height ) ;

        /*! \brief accessor methods
         *
         *  This function applies the arcball matrix on the current OpenGL
         *  rendering matrix (modelview).
         */

        le_void_t ab_get_rotate( le_void_t );

    private:

        /*! \brief mutator methods
         *
         *  This function simply sets the arcball matrix to identity.
         */

        le_void_t ab_set_identity( le_void_t );

        /*! \brief mutator methods
         *
         *  This function performs the matrix multiplication between the matrix
         *  provided as parameter and the arcball matrix.
         *
         *  The provided matrix has to be a 4 by 4 matrix stored in a linear
         *  16 entries array.
         *
         *  \param dl_matrix Matrix coefficients array
         */

        le_void_t ab_set_multiply( le_real_t * dl_matrix );

    public:

        /*! \brief mutator methods
         *
         *  This function update the arcball matrix according to the mouse click
         *  start and stop position.
         *
         *  The standard arcball model is implemented by the function using a
         *  mouse trajectory perpendicular vector to compute a rotation matrix
         *  along this axis. The computed matrix is then composed with the
         *  current state of the arcball matrix.
         *
         *  \param dl_init_x Mouse click initial position
         *  \param dl_init_y Mouse click initial position
         *  \param dl_step_x Mouse click final position
         *  \param dl_step_y Mouse click final position
         */

        le_void_t ab_set_update( le_size_t const dl_init_x, le_size_t const dl_init_y, le_size_t const dl_step_x, le_size_t const dl_step_y );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

