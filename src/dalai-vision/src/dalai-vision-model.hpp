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

    # include "dalai-vision-surface.hpp"

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <cstdlib>
    # include <ctime>
    # include <cmath>
    # include <GL/gl.h>
    # include <GL/glu.h>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define mdmv estimation sample */
    # define DL_MODEL_SAMPLE ( 32 )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

    /*! \class dl_model_t ( revoked )
     *  \brief Model class
     *
     *  This class is used to stored the data of the 3d model manipulated by the
     *  software through the graphical user interface.
     *
     *  As the class is created, the model is opened and loaded in the class
     *  memory. In addition to model importation, the creation of the class also
     *  triggers the analysis of the model. Value such as the model minimum
     *  distance mean value are computed and stored in the class.
     *
     *  In addition, the class also provides management of the surfaces class
     *  associated to the model. It offers methods allowing to compute the
     *  intersection point of three defined surface to export its coordinates.
     *
     *  Finally, the class also offers rendering methods allowing to draw the
     *  model through the opengl api.
     *
     *  \var dl_model_t::ml_size
     *  Number of points composing the model
     *  \var dl_model_t::ml_data
     *  Model points coordinates and colors array
     *  \var dl_model_t::ml_psize
     *  Model rendering point size
     *  \var dl_model_t::ml_hide
     *  If non zero, triggers the display of the surfaces
     *  \var dl_model_t::ml_pflag
     *  If non zero, triggers the display of the surfaces estimation points
     *  \var dl_model_t::ml_xmin
     *  Model minimum x-coordinate
     *  \var dl_model_t::ml_xmax
     *  Model maximum x-coordinate
     *  \var dl_model_t::ml_ymin
     *  Model minimum y-coordinate
     *  \var dl_model_t::ml_ymax
     *  Model maximum y-coordinate
     *  \var dl_model_t::ml_zmin
     *  Model minimum z-coordinate
     *  \var dl_model_t::ml_zmax
     *  Model maximum z-coordinate
     *  \var dl_model_t::ml_mdmv
     *  Model minimum distance mean value
     *  \var dl_model_t::ml_x
     *  Model pseudo-center x-coordinate
     *  \var dl_model_t::ml_y
     *  Model pseudo-center y-coordinate
     *  \var dl_model_t::ml_z
     *  Model pseudo-center z-coordinate
     *  \var dl_model_t::ml_active
     *  Highlighted surface index
     *  \var dl_model_t::ml_surface
     *  Model surfaces array
     */

    class dl_model_t {

    private:
        le_size_t      ml_size;
        le_size_t      ml_real;
        le_byte_t    * ml_data;
        le_real_t      ml_x;
        le_real_t      ml_y;
        le_real_t      ml_z;
        le_enum_t      ml_hide;
        le_real_t      ml_mdmv;
        le_real_t      ml_span;
        GLuint         ml_count[2];
        GLuint       * ml_index[2];
        le_size_t      ml_active;
        dl_surface_t   ml_surface[3];

    public:

        /*! \brief constructor methods ( revoked )
         *
         *  The constructor methods starts by initialising the class members. It
         *  then uses the provided path to open and load the model.
         *
         *  After model reading and importation, the constructor uses the class
         *  methods to compute the characteristic of the model. It then finally
         *  initialise the three surfaces hold by the class.
         *
         *  \param ml_model Path to the uf3 file containing the model
         */

        dl_model_t( le_char_t const * const dl_path );

        /*! \brief destructor methods ( revoked )
         *
         *  The destructor method simply unallocate the memory used to store the
         *  model data and destruct the class instances associated to the model
         *  surfaces.
         */

        ~dl_model_t();

    public:

        /*! \brief accessor methods ( revoked )
         *
         *  This function returns the largest diagonal of the box containing the
         *  whole model.
         *
         *  \return Returns model largest diagonal
         */

        le_real_t ml_get_span( le_void_t );

        /*! \brief accessor methods ( revoked )
         *
         *  This function computes and displays in the error output the three
         *  coordinates of the point at the intersection of the three surfaces
         *  of the model.
         */

        le_void_t ml_get_intersection( le_void_t );

        /* *** */

        le_void_t ml_get_translation( le_void_t );

    public:

        /*! \brief mutator methods ( revoked )
         *
         *  This function allows to reset the model pseudo-center. The model
         *  pseudo-center is interpreted as the center of rotation by the
         *  rendering methods.
         *
         *  \param dl_x Pseudo-center x-coordinate
         *  \param dl_y Pseudo-center y-coordinate
         *  \param dl_z Pseudo-center z-coordinate
         */

        le_void_t ml_set_center( le_real_t const dl_x, le_real_t const dl_y, le_real_t const dl_z );

        /*! \brief mutator methods ( revoked )
         *
         *  This function allows the set the index of the model highlighted
         *  surface. The highlighted surface is the surface on which edition
         *  is possible through the graphical interface.
         *
         *  \param dl_active Highlighted surface index
         */

        le_void_t ml_set_surface( le_size_t const dl_surface );

        /*! \brief mutator methods ( revoked )
         *
         *  This function allows to switch the rendering state of the model
         *  surfaces.
         */

        le_void_t ml_set_switch( le_void_t );

        /*! \brief mutator methods ( revoked )
         *
         *  This function pushes the model pseudo-center to the stack of
         *  estimation points of the highlighted surface.
         *
         *  As explained in the documentation of \b dl_surface_t, the model
         *  pseudo-center can also be used to remove points from the stack of
         *  estimation points.
         */

        le_void_t ml_set_push( le_void_t );

        /*! \brief mutator methods ( revoked )
         *
         *  This function triggers the automatic selection of the estimation
         *  points of the highlighted surface.
         */

        le_void_t ml_set_auto( le_size_t const dl_mode );

        /*! \brief mutator methods ( revoked )
         *
         *  This function triggers the clearing of the stack of estimation
         *  points of the highlighted surface.
         */

        le_void_t ml_set_clear( le_void_t );

    private:

        /* *** */

        le_void_t ml_set_analysis( le_void_t );

    public:

        /*! \brief rendering methods ( revoked )
         *
         *  This function renders, using opengl api, the model frame axis. It
         *  considers the model minimum distance mean value for the rendering
         *  of the axis to allow the user to have an idea of this important
         *  value.
         */

        le_void_t ml_ren_frame( le_void_t );

        /*! \brief rendering methods ( revoked )
         *
         *  This function allows to render the model elements through the opengl
         *  api using simple points.
         *
         *  The model pseudo-center is considered as the center of rotation of
         *  the model.
         */

        le_void_t ml_ren_model( le_void_t );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

