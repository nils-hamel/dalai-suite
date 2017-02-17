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

    /*! \file   dalai-vision.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - vision
     */

    /*! \mainpage dalai-suite
     *
     *  \section _1 dalai-suite
     *
     *  The _dalai-suite_ is dedicated to the gathering and processing of
     *  geographical 3-dimensional information. It allows to considers the most
     *  common file formats and to convert them in a standardised and simple
     *  format.
     *
     *  This standardised format allows to use the suite tools for colour
     *  edition, model cleaning and model hashing. In addition, the standard
     *  format is also expected by the _eratosthene-suite_ implementing the EPFL
     *  CDH DHLAB indexation server and its geographical 3-dimensional data
     *  injection tools.
     *
     *  \section _2 Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2017 EPFL CDH DHLAB
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_VISION__
    # define __DL_VISION__

/*
    header - internal includes
 */

    # include "dalai-vision-model.hpp"

/*
    header - external includes
 */

    # include <iostream>
    # include <cstdlib>
    # include <cmath>
    # include <GL/gl.h>
    # include <GL/glu.h>
    # include <SDL2/SDL.h>
    # include <common-include.hpp>

/*
    header - preprocessor definitions
 */

    /* define mouse motion factors */
    # define DL_INERTIA_ANGLE  ( 0.002 )
    # define DL_INERTIA_TRANS  ( 0.15 )
    # define DL_INERTIA_WZOOM  ( 20.0 )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

    class dl_vision_t {

    private:
        SDL_Window *  vs_window;
        SDL_GLContext vs_context;
        SDL_Event     vs_event;

        int           vs_width;
        int           vs_height;
        float         vs_near;
        float         vs_far;

        bool          vs_state;

        float         vs_angle_x;
        float         vs_angle_y;
        float         vs_angle_z;
        float         vs_trans_x;
        float         vs_trans_y;
        float         vs_trans_z;

        int           vs_mouse_x;
        int           vs_mouse_y;

        GLdouble      vs_modelview[16];
        GLdouble      vs_projection[16];
        GLint         vs_viewport[4];

    public:
        dl_vision_t();
        ~dl_vision_t();

    public:
        bool vs_get_click( int const dl_mx, int const dl_my, double * const dl_px, double * const dl_py, double * const dl_pz );

    public:
        void vs_set_projection( dl_model_t & dl_model );
        void vs_set_viewpoint( dl_model_t & dl_model );

    public:
        void vs_execution( dl_model_t & dl_model );

    private:
        void vs_keydown( SDL_KeyboardEvent vs_event, dl_model_t & dl_model );
        void vs_button( SDL_MouseButtonEvent vs_event, dl_model_t & dl_model );
        void vs_motion( SDL_MouseMotionEvent vs_event, dl_model_t & dl_model );
        void vs_wheel( SDL_MouseWheelEvent vs_event, dl_model_t & dl_model );

    };

/*
    header - function prototypes
 */

    /*! \brief main function
     *
     *
     *  \param argc Standard parameter
     *  \param argv Standard parameter
     *
     *  \return Returns standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

