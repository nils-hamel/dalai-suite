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
     *  This standardised format allows to use the suite tools for color
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

    /*! \class dl_vision_t
     *  \brief Graphical interface class
     *
     *  This class holds the information and methods required to operate the
     *  software graphical user interface.
     *
     *  It offers methods for the initialisation and deletion of the graphical
     *  context, events management and rendering. It also holds the user point
     *  of view related information.
     *
     *  The class also holds copies of the opengl model and projection matrix
     *  and the viewport vector. These elements are usually used for model
     *  click interpretation.
     *
     *  \var dl_vision_t::vs_window
     *  Graphical context window
     *  \var dl_vision_t::vs_context
     *  Graphical context opengl context
     *  \var dl_vision_t::dl_event
     *  Event management structure
     *  \var dl_vision_t::vs_width
     *  Width, in pixels, of the interface screen
     *  \var dl_vision_t::vs_height
     *  Height, in pixels, of the interface screen
     *  \var dl_vision_t::vs_near
     *  Opengl near plane distance
     *  \var dl_vision_t::vs_far
     *  Opengl far plane distance
     *  \var dl_vision_t::vs_state
     *  Execution loop state
     *  \var dl_vision_t::vs_angle_x
     *  Rotation angle of the point of view along the x-axis
     *  \var dl_vision_t::vs_angle_y
     *  Rotation angle of the point of view along the y-axis
     *  \var dl_vision_t::vs_angle_z
     *  Rotation angle of the point of view along the z-axis
     *  \var dl_vision_t::vs_trans_x
     *  Translation of the point of view along the x-axis
     *  \var dl_vision_t::vs_trans_y
     *  Translation of the point of view along the y-axis
     *  \var dl_vision_t::vs_trans_z
     *  Translation of the point of view along the z-axis
     *  \var dl_vision_t::vs_mouse_x
     *  Mouse position along screen x-axis
     *  \var dl_vision_t::vs_mouse_y
     *  Mouse position along screen y-axis
     *  \var dl_vision_t::vs_modelview
     *  Modelview matrix of opengl
     *  \var dl_vision_t::vs_projection
     *  Projection matrix of opengl
     *  \var dl_vision_t::vs_viewport
     *  Viewport vector of the opengl render screen
     */

    class dl_vision_t {

    private:
        SDL_Window *  vs_window;
        SDL_GLContext vs_context;
        SDL_Event     dl_event;

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

        /*! \brief constructor methods
         *
         *  The constructor method sets the interface graphical context using
         *  sdl. After sdl initialisation, it creates the interface windows and
         *  configures the opengl graphical context.
         */

        dl_vision_t();

        /*! \brief destructor methods
         *
         *  The destructor method simply deletes the interface window and the
         *  opengl graphical context. It finally uninitialise sdl.
         */

        ~dl_vision_t();

    public:

        /*! \brief accessor methods
         *
         *  This function converts the provided mouse click coordinates into the
         *  coordinates of a 3d point that belong to the model and corresponding
         *  to the click position.
         *
         *  It returns true as a model point is found under the mouse click,
         *  false otherwise.
         *
         *  \param  dl_mx Mouse click x-position
         *  \param  dl_my Mouse click y-position
         *  \param  dl_px Model point x-coordinate
         *  \param  dl_py Model point y-coordinate
         *  \param  dl_pz Model point z-coordinate
         *
         *  \return Returns true on success, false otherwise
         */

        bool vs_get_click( int const dl_mx, int const dl_my, double * const dl_px, double * const dl_py, double * const dl_pz );

    public:

        /*! \brief mutator methods
         *
         *  This function sets the opengl viewport and projection matrix based
         *  on the interface screen sizes.
         *
         *  It also specifies the opengl near and far planes by considering the
         *  model largest diagonal and minimum distance mean value.
         *
         *  \param dl_model Model class
         */

        void vs_set_projection( dl_model_t & dl_model );

        /*! \brief mutator methods
         *
         *  This function sets the opengl initial point of view. It resets the
         *  view angles to zero and resets the view translation using the model
         *  largest diagonal. The initial position of the point of view offers
         *  an overall view of the model.
         *
         *  \param dl_model Model class
         */

        void vs_set_viewpoint( dl_model_t & dl_model );

    public:

        /*! \brief execution methods
         *
         *  This function holds the interface execution loop. The execution loop
         *  is responsible of maintaining the software execution.
         *
         *  In addition, it has to manage the user interface events and it is
         *  responsible of the model and interface rendering.
         *
         *  \param dl_model Model class
         */

        void vs_execution( dl_model_t & dl_model );

    private:

        /*! \brief event methods
         *
         *  This function is responsible of the management of the user interface
         *  events coming from the keyboard.
         *
         *  \param dl_event Event structure
         *  \param dl_model Model class
         */

        void vs_keydown( SDL_KeyboardEvent dl_event, dl_model_t & dl_model );

        /*! \brief event methods
         *
         *  This function is responsible of the management of the user interface
         *  events coming from the mouse click.
         *
         *  \param dl_event Event structure
         *  \param dl_model Model class
         */

        void vs_button( SDL_MouseButtonEvent dl_event, dl_model_t & dl_model );

        /*! \brief event methods
         *
         *  This function is responsible of the management of the user interface
         *  events coming from the mouse motion.
         *
         *  \param dl_event Event structure
         *  \param dl_model Model class
         */

        void vs_motion( SDL_MouseMotionEvent dl_event, dl_model_t & dl_model );

        /*! \brief event methods
         *
         *  This function is responsible of the management of the user interface
         *  events coming from the mouse wheel.
         *
         *  \param dl_event Event structure
         *  \param dl_model Model class
         */

        void vs_wheel( SDL_MouseWheelEvent dl_event, dl_model_t & dl_model );

    };

/*
    header - function prototypes
 */

    /*! \brief main function
     *
     *  The main function creates the graphical user interface, model class
     *  instances and starts execution loop :
     *
     *      ./dalai-vision --model/-m [path to the uf3 file of the model]
     *
     *  The creation of the model class instance triggers the importation and
     *  analysis of the model contained in the provided uf3 file.
     *
     *  The interface allows to manipulate the model using the mouse : by
     *  maintaining the right click, the motion of the mouse allows to rotate
     *  the model. By maintaining the left click, the motion of the mouse allows
     *  to update the translation of the model. Playing with the mouse wheel
     *  allows to update the distance to the model.
     *
     *  The double-click on a point of the model sets it as the model center of
     *  rotation. By clicking the mouse wheel, the center of rotation is pushed
     *  on the estimation points stack of the highlighted surface. If the point
     *  is already in the stack, it is removed by this click.
     *
     *  Pressing [q], [w] or [e] allows the highlight respectively the first,
     *  second and third model surfaces. Pressing the key [a] triggers the
     *  automatic (re)selection of the estimation points of the highlighted
     *  surface. The automatic selection only make sense as at least four
     *  estimation points have already been pushed manually to the stack.
     *
     *  As three surfaces are defined, pressing the return key allows to compute
     *  and display on the error output the coordinates of the point at the
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

