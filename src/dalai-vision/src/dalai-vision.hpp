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
     *  This standardised format allows to use the suite tools for color
     *  edition, model cleaning and model hashing. In addition, the standard
     *  format is also expected by the _eratosthene-suite_ implementing the EPFL
     *  CDH DHLAB indexation server and its geographical 3-dimensional data
     *  injection tools.
     *
     *  \section _2 Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2018 DHLAB, EPFL
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

    # include "dalai-vision-arcball.hpp"
    # include "dalai-vision-model.hpp"

/*
    header - external includes
 */

    # include <iostream>
    # include <cstdlib>
    # include <cstring>
    # include <cmath>
    # include <GL/gl.h>
    # include <GL/glu.h>
    # include <SDL2/SDL.h>
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

    /*! \class dl_vision_t
     *  \brief Vision class
     *
     *  This class holds the information and methods required to operate the
     *  graphical user interface.
     *
     *  It offers methods for the initialisation and deletion of the graphical
     *  context, events management and rendering. It also holds the user point
     *  of view related information.
     *
     *  \var dl_vision_t::vs_window
     *  Interface window
     *  \var dl_vision_t::vs_context
     *  OpenGL graphical context
     *  \var dl_vision_t::vs_execute
     *  Execution loop operation switch
     *  \var dl_vision_t::vs_width
     *  Width, in pixels, of the interface
     *  \var dl_vision_t::vs_height
     *  Height, in pixels, of the interface
     *  \var dl_vision_t::vs_init_x
     *  Mouse initial click position
     *  \var dl_vision_t::vs_init_y
     *  Mouse initial click position
     *  \var dl_vision_t::vs_dist_z
     *  Distance of point of view to model rotation center
     */

    class dl_vision_t {

    private:

        SDL_Window *  vs_window;
        SDL_GLContext vs_context;
        bool          vs_execute;
        int           vs_width;
        int           vs_height;
        le_size_t     vs_init_x;
        le_size_t     vs_init_y;
        le_real_t     vs_dist_z;

    public:

        /*! \brief constructor methods
         *
         *  The constructor method sets the interface graphical context using
         *  sdl. After sdl initialisation, it creates the interface windows and
         *  configures the opengl graphical context.
         *
         *  The provided model span is used to initialise the point of view.
         *
         *  \param dl_model_span Model maximum diameter
         */

        dl_vision_t( le_real_t const dl_model_span );

        /*! \brief destructor methods
         *
         *  The destructor method simply deletes the interface window and the
         *  opengl graphical context. It finally uninitialise sdl.
         */

        ~dl_vision_t();

    public:

        /*! \brief accessor methods
         *
         *  This function returns the width in pixel of the interface.
         *
         *  \return Interface width, in pixels
         */

        le_size_t vs_get_width( le_void_t );

        /*! \brief accessor methods
         *
         *  This function returns the height in pixels of the interface.
         *
         *  \return Interface height, in pixels
         */

        le_size_t vs_get_height( le_void_t );

        /*! \brief mutator methods
         *
         *  This function sets the opengl viewport and projection matrix based
         *  on the interface width and height.
         *
         *  It also specifies the opengl near and far planes by considering the
         *  model maximal diameter.
         *
         *  \param dl_model Model class
         */

        le_void_t vs_set_projection( dl_model_t & dl_model );

        /*! \brief mutator methods
         *
         *  This function update the model rotation center using the point of
         *  the model below the click.
         *
         *  \param dl_click_x Mouse click position
         *  \param dl_click_y Mouse click position
         *  \param dl_arcball Arcball class
         *  \param dl_model   Model class
         */

        le_void_t vs_set_center( le_size_t const dl_click_x, le_size_t const dl_click_y, dl_arcball_t & dl_arcball, dl_model_t & dl_model );

    public:

        /*! \brief execution methods
         *
         *  This function holds the interface execution loop. The execution loop
         *  is responsible of maintaining the software execution.
         *
         *  In addition, it has to manage the user interface events and it is
         *  responsible of the model and interface rendering.
         *
         *  \param dl_arcball Arcball class
         *  \param dl_model   Model class
         */

        le_void_t vs_execution( dl_arcball_t & dl_arcball, dl_model_t & dl_model );

    private:

        /*! \brief event methods
         *
         *  This function is responsible of the management of the user interface
         *  events coming from the keyboard.
         *
         *  \param dl_event   Event structure
         *  \param dl_arcball Arcball class
         *  \param dl_model   Model class
         */

        le_void_t vs_keydown( SDL_KeyboardEvent dl_event, dl_arcball_t & dl_arcball, dl_model_t & dl_model );

        /*! \brief event methods
         *
         *  This function is responsible of the management of the user interface
         *  events coming from the mouse click.
         *
         *  \param dl_event   Event structure
         *  \param dl_arcball Arcball class
         *  \param dl_model   Model class
         */

        le_void_t vs_button( SDL_MouseButtonEvent dl_event, dl_arcball_t & dl_arcball, dl_model_t & dl_model );

        /*! \brief event methods
         *
         *  This function is responsible of the management of the user interface
         *  events coming from the mouse motion.
         *
         *  \param dl_event   Event structure
         *  \param dl_arcball Arcball class
         *  \param dl_model   Model class
         */

        le_void_t vs_motion( SDL_MouseMotionEvent dl_event, dl_arcball_t & dl_arcball, dl_model_t & dl_model );

        /*! \brief event methods
         *
         *  This function is responsible of the management of the user interface
         *  events coming from the mouse wheel.
         *
         *  \param dl_event   Event structure
         *  \param dl_arcball Arcball class
         *  \param dl_model   Model class
         */

        le_void_t vs_wheel( SDL_MouseWheelEvent dl_event, dl_arcball_t & dl_arcball, dl_model_t & dl_model );

    };

/*
    header - function prototypes
 */

    /*! \brief main function
     *
     *  This software allows to import uv3 models and to manipulate them through
     *  a simple graphical interface :
     *
     *      ./dalai-vision --uv3/-i [uv3 file path]
     *
     *  The interface allows to manipulate the model using the mouse : by
     *  maintaining the right click, the motion of the mouse allows to rotate
     *  the model. Playing with the mouse wheel allows to update the distance to
     *  the model.
     *
     *  The double-click on a point of the model sets it as the model center of
     *  rotation. By clicking the mouse wheel, the center of rotation is pushed
     *  on the estimation points stack of the highlighted surface. If the point
     *  is already in the stack, it is removed by this click.
     *
     *  Pressing [q], [w] or [e] allows the highlight respectively the first,
     *  second and third model surfaces. Pressing the keys [a], [s] and [d]
     *  trigger the automatic (re)selection of the estimation points of the
     *  highlighted surface. The automatic selection only make sense as at least
     *  four estimation points have already been pushed manually to the stack.
     *
     *  As three surfaces are defined, pressing the return key allows to compute
     *  and display on the standard output the coordinates of the point at the
     *  intersection of the three surface.
     *
     *  Pressing the tabulation key allows to show or hide the model surfaces.
     *
     *  Finally, pressing the backspace key allows to clear the estimation
     *  points stack of the highlighted surface.
     *
     *  The escape key allows to stop the execution loop which causes the
     *  software to quit.
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

