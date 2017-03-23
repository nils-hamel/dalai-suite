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

    # include "dalai-vision-surface.hpp"

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <limits>
    # include <cmath>
    # include <cstdio>
    # include <GL/gl.h>
    # include <GL/glu.h>
    # include <Eigen/Dense>
    # include <common-include.hpp>

/*
    header - preprocessor definitions
 */

    /* define array mean estimation count */
    # define DL_MODEL_MDMVC  ( 32 )

    /* define opengl arrays types */
    # define DL_MODEL_V_TYPE ( GL_DOUBLE )
    # define DL_MODEL_C_TYPE ( GL_UNSIGNED_BYTE )

    /* define opengl arrays base pointer */
    # define DL_MODEL_V_BASE ( 0 )
    # define DL_MODEL_C_BASE ( sizeof( double ) * 3 )

    /* define opengl arrays stripe */
    # define DL_MODEL_STRIPE ( ( sizeof( double ) + sizeof( char ) ) * 3 )

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
     *  \var dl_model_t::ml_sflag
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
     *  \var dl_model_t::ml_xcen
     *  Model pseudo-center x-coordinate
     *  \var dl_model_t::ml_ycen
     *  Model pseudo-center y-coordinate
     *  \var dl_model_t::ml_zcen
     *  Model pseudo-center z-coordinate
     *  \var dl_model_t::ml_sact
     *  Highlighted surface index
     *  \var dl_model_t::ml_s
     *  Model surfaces array
     */

    class dl_model_t {

    private:
        long long    ml_size;
        char *       ml_data;

        long long    ml_psize;
        long long    ml_sflag;
        long long    ml_pflag;

        double       ml_xmin;
        double       ml_xmax;
        double       ml_ymin;
        double       ml_ymax;
        double       ml_zmin;
        double       ml_zmax;

        double       ml_mdmv;

        double       ml_xcen;
        double       ml_ycen;
        double       ml_zcen;

        long long    ml_sact;
        dl_surface_t ml_s[3];

    public:

        /*! \brief constructor methods
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

        dl_model_t( char * ml_model );

        /*! \brief destructor methods
         *
         *  The destructor method simply unallocate the memory used to store the
         *  model data and destruct the class instances associated to the model
         *  surfaces.
         */

        ~dl_model_t();

    public:

        /*! \brief accessor methods
         *
         *  This function simply returns the model minimum distance mean value.
         *
         *  \return Returns model minimum distance mean value
         */

        double ml_get_mdmv( void );

        /*! \brief accessor methods
         *
         *  This function returns the largest diagonal of the box containing the
         *  whole model.
         *
         *  \return Returns model largest diagonal
         */

        double ml_get_span( void );

        /*! \brief accessor methods
         *
         *  This function computes and displays in the error output the three
         *  coordinates of the point at the intersection of the three surfaces
         *  of the model.
         */

        void ml_get_intersect( void );

    public:

        /*! \brief mutator methods
         *
         *  This function allows to reset the model pseudo-center. The model
         *  pseudo-center is interpreted as the center of rotation by the
         *  rendering methods.
         *
         *  \param dl_x Pseudo-center x-coordinate
         *  \param dl_y Pseudo-center y-coordinate
         *  \param dl_z Pseudo-center z-coordinate
         */

        void ml_set_center( double const dl_x, double const dl_y, double const dl_z );

        /*! \brief mutator methods
         *
         *  This function allows the set the index of the model highlighted
         *  surface. The highlighted surface is the surface on which edition
         *  is possible through the graphical interface.
         *
         *  \param dl_active Highlighted surface index
         */

        void ml_set_surface( long long const dl_active );

        /*! \brief mutator methods
         *
         *  This function allows to switch the rendering state of the model
         *  surfaces.
         */

        void ml_set_surface_switch( void );

        /*! \brief mutator methods
         *
         *  This function allows to set the size of the points used to render
         *  the class model. The provided size is interpreted in pixels.
         *
         *  \param dl_pointsize Size, in pixels, of the rendering points
         */

        void ml_set_pointsize( long long const dl_pointsize );

        /*! \brief mutator methods
         *
         *  This function pushes the model pseudo-center to the stack of
         *  estimation points of the highlighted surface.
         *
         *  As explained in the documentation of \b dl_surface_t, the model
         *  pseudo-center can also be used to remove points from the stack of
         *  estimation points.
         */

        void ml_set_point_push( void );

        /*! \brief mutator methods
         *
         *  This function triggers the automatic selection of the estimation
         *  points of the highlighted surface.
         */

        void ml_set_point_auto( void );

        /*! \brief mutator methods
         *
         *  This function triggers the clearing of the stack of estimation
         *  points of the highlighted surface.
         */

        void ml_set_point_clear( void );

    private:

        /*! \brief mutator methods
         *
         *  This function performs the analysis of the model imported in the
         *  class. It is usually used after model importation.
         *
         *  In addition to the detection of the lowest and largest coordinates
         *  of the model in each direction, this function also computes an
         *  estimation of the model minimum distance mean value.
         */

        void ml_set_analysis( void );

    public:

        /*! \brief rendering methods
         *
         *  This function allows to render the model elements through the opengl
         *  api using simple points.
         *
         *  The model pseudo-center is considered as the center of rotation of
         *  the model.
         */

        void ml_ren_model( void );

        /*! \brief rendering methods
         *
         *  This function invoke the rendering of the model surfaces and their
         *  associated estimation points by calling their respective rendering
         *  methods. The rendering flags are checked by this function.
         */

        void ml_ren_surface( void );

        /*! \brief rendering methods
         *
         *  This function renders, using opengl api, the model frame axis. It
         *  considers the model minimum distance mean value for the rendering
         *  of the axis to allow the user to have an idea of this important
         *  value.
         */

        void ml_ren_frame( void );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

