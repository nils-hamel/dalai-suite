/*
 *  dalai-suite - vision
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2019 DHLAB, EPFL
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

    /*! \class dl_model_t
     *  \brief Model class
     *
     *  This class is used to stored the data of the model imported from an uv3
     *  stream.
     *
     *  In the first place, the class contains member allowing the importation
     *  of the model proper data, that is a memory pointer and sizes. The class
     *  also holds the definition of the model rotation center and its minimum
     *  distances mean value.
     *
     *  The class also holds the index array that are used to render the model
     *  primitive in an efficient manner.
     *
     *  The class also holds the three surfaces object used to compute optimal
     *  intersection in the model. The highlighted surfaces and their display
     *  mode are also members of this class.
     *
     *  \var dl_model_t::ml_size
     *  Size, in bytes, of the model
     *  \var dl_model_t::ml_real
     *  Size, in records, of the model
     *  \var dl_model_t::ml_data
     *  Model memory storage
     *  \var dl_model_t::ml_norm
     *  Model normal storage
     *  \var dl_model_t::ml_x
     *  Model rotation center
     *  \var dl_model_t::ml_y
     *  Model rotation center
     *  \var dl_model_t::ml_z
     *  Model rotation center
     *  \var dl_model_t::ml_hide
     *  Model surface rendering mode
     *  \var dl_model_t::ml_mdmv
     *  Model minimum distances mean value
     *  \var dl_model_t::ml_span
     *  Model maximum distances to its centroid
     *  \var dl_model_t::ml_rsize
     *  Model primitive count array
     *  \var dl_model_t::ml_rdata
     *  Model primitive index array
     *  \var dl_model_t::ml_active
     *  Model highlighted surface
     *  \var dl_model_t::ml_surface
     *  Model surfaces
     */

    class dl_model_t {

    private:
        le_size_t      ml_size;
        le_size_t      ml_real;
        le_byte_t    * ml_data;
        le_real_t    * ml_norm;
        le_real_t      ml_x;
        le_real_t      ml_y;
        le_real_t      ml_z;
        le_enum_t      ml_hide;
        le_real_t      ml_mdmv;
        le_real_t      ml_span;
        GLuint         ml_rsize[3];
        GLuint       * ml_rdata;
        le_size_t      ml_active;
        dl_surface_t   ml_surface[3];

    public:

        /*! \brief constructor methods
         *
         *  The constructor methods starts by initialising the class members. It
         *  then uses the provided path to open and load the model contained in
         *  in an uv3 stream.
         *
         *  The three surfaces members are initialised before the constructor
         *  invokes specialised member function to analyse the imported model.
         *
         *  \param dl_path Model uv3 stream path
         */

        dl_model_t( le_char_t const * const dl_path );

        /*! \brief destructor methods
         *
         *  The destructor method simply unallocate the memory used to store the
         *  model data. It also unallocate the memory used for primitive index
         *  storage.
         */

        ~dl_model_t();

    public:

        /*! \brief accessor methods
         *
         *  This function returns two times the largest distance of the model
         *  points to the model centroid.
         *
         *  \return Returns two times the model radius
         */

        le_real_t ml_get_span( le_void_t );

        /*! \brief accessor methods
         *
         *  This function triggers the computation of the intersection of the
         *  three surfaces using their specific method.
         */

        le_void_t ml_get_intersection( le_void_t );

        /*! \brief accessor methods
         *
         *  This function allows to apply the model translation center to the
         *  current OpenGL matrix. This translation is used to make the model
         *  rotating around its current center.
         */

        le_void_t ml_get_translation( le_void_t );

    public:

        /*! \brief mutator methods
         *
         *  This function allows to update the position of the model center of
         *  rotation.
         *
         *  \param dl_x Rotation center
         *  \param dl_y Rotation center
         *  \param dl_z Rotation center
         */

        le_void_t ml_set_center( le_real_t const dl_x, le_real_t const dl_y, le_real_t const dl_z );

        /*! \brief mutator methods
         *
         *  This function allows the set the index of the model highlighted
         *  surface. The highlighted surface is the surface on which edition
         *  is possible through the graphical interface.
         *
         *  \param dl_surface Highlighted surface index
         */

        le_void_t ml_set_surface( le_size_t const dl_surface );

        /*! \brief mutator methods
         *
         *  This function allows to switch the rendering state of the model
         *  surfaces from transparent to invisible.
         */

        le_void_t ml_set_switch( le_void_t );

        /*! \brief mutator methods
         *
         *  This function pushes the model rotation center to the stack of
         *  estimation points of the highlighted surface.
         *
         *  As explained in the documentation of \b dl_surface_t, the model
         *  rotation center can also be used to remove points from the stack of
         *  the highlighted surface estimation points.
         */

        le_void_t ml_set_push( le_void_t );

        /*! \brief mutator methods
         *
         *  This function triggers the automatic selection of the estimation
         *  points of the highlighted surface.
         *
         *  If the value of \b dl_mode is negative, the growing tolerance is
         *  smaller than the estimation points set radius. If it is positive,
         *  the tolerance is greater, allowing to spread the estimation points
         *  set. If zero is provided, the tolerance is set to the value of the
         *  estimation points set radius.
         *
         *  \param dl_mode Automatic selection mode
         */

        le_void_t ml_set_auto( le_size_t const dl_mode );

        /*! \brief mutator methods
         *
         *  This function triggers the clearing of the stack of estimation
         *  points of the highlighted surface.
         */

        le_void_t ml_set_clear( le_void_t );

    private:

        /*! \brief mutator methods
         *
         *  This function computes the required elements required for model
         *  rendering.
         *
         *  In the first place, the function computes the amount of primitives
         *  in the model. This statistics is used to set the index array used
         *  for efficient rendering.
         *
         *  In addition, the function creates and fills the normal array used
         *  for polygon and model lighting. The computation of the normal is
         *  performed for triangles only.
         */

        le_void_t ml_set_render( le_void_t );

        /*! \brief mutator methods
         *
         *  This function performs the analysis of the imported model. This
         *  function is usually used just after model data importation.
         *
         *  In the first place, the function performs an estimation of the model
         *  vertex minimal separation mean value. This value is used, combined
         *  with a factor, as a tolerance reference for points selection.
         *
         *  In addition, the function performs also the computation of the model
         *  centroid, used as initial model rotation center. The centroid is
         *  also used to shift the model near the origin to avoid GPU simple
         *  simple precision saturation.
         *
         *  In the last place, the function also computes the size of the model
         *  by finding the centroid most distant point.
         */

        le_void_t ml_set_analysis( le_void_t );

    public:

        /*! \brief rendering methods
         *
         *  This function renders through OpenGL the model frame axis.
         *
         *  It considers the minimum distances mean value for the rendering of
         *  the axis to allow the user to have an idea of this important value.
         */

        le_void_t ml_ren_frame( le_void_t );

        /*! \brief rendering methods
         *
         *  This function renders the primitives of the model. Each group of
         *  primitives are rendered using optimised methods.
         *
         *  The function also invokes the surfaces rendering function for the
         *  display of the model surface state.
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

