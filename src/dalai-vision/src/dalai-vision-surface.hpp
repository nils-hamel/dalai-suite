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

    /*! \file   dalai-vision-surface.hpp
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
    # include <common-include.hpp>
    # include <eratosthene-include.h>

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

    /*! \class dl_surface_t
     *  \brief Surface class
     *
     *  This class holds the definition of a mathematical plane. In addition to
     *  the plane definition, through its normal vector and a constant, it also
     *  holds two orthonormal vectors providing a basis of the plane. The class
     *  also holds a color used to represent the plane.
     *
     *  The equation of the plane is estimated from a set of estimation points :
     *  the \b sf_set_equation() class method computes the plane that minimise
     *  its distance to all estimation points. In other words, the function
     *  tries to consider the plane that fits the best the provided estimation
     *  points.
     *
     *  Because the equation of the plane is derived from a set of estimation
     *  points, the class offers methods to add and remove such points.
     *
     *  Finally, the class offers functions allowing to display a representation
     *  of the plane and its estimation points through the opengl api.
     *
     *  \var dl_surface_t::sf_cx
     *  Centroid of the surface estimation points - x-coordinate
     *  \var dl_surface_t::sf_cy
     *  Centroid of the surface estimation points - y-coordinate
     *  \var dl_surface_t::sf_cz
     *  Centroid of the surface estimation points - z-coordinate
     *  \var dl_surface_t::sf_px
     *  Surface normal vector - x coefficient
     *  \var dl_surface_t::sf_py
     *  Surface normal vector - y coefficient
     *  \var dl_surface_t::sf_pz
     *  Surface normal vector - z coefficient
     *  \var dl_surface_t::sf_pc
     *  Surface normal vector - constant coefficient
     *  \var dl_surface_t::sf_ux
     *  Surface base vector - x-coordinate
     *  \var dl_surface_t::sf_uy
     *  Surface base vector - y-coordinate
     *  \var dl_surface_t::sf_uz
     *  Surface base vector - z-coordinate
     *  \var dl_surface_t::sf_vx
     *  Surface base vector - x-coordinate
     *  \var dl_surface_t::sf_vy
     *  Surface base vector - y-coordinate
     *  \var dl_surface_t::sf_vz
     *  Surface base vector - z-coordinate
     *  \var dl_surface_t::sf_cr
     *  Surface red component
     *  \var dl_surface_t::sf_cg
     *  Surface green component
     *  \var dl_surface_t::sf_cb
     *  Surface blue component
     *  \var dl_surface_t::sf_size
     *  Number of estimation points
     *  \var dl_surface_t::sf_virt
     *  Estimation points array size
     *  \var dl_surface_t::sf_data
     *  Estimation points array data
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

        /*! \brief constructor methods
         *
         *  The constructor simply initialises the class members. In addition,
         *  it also initialise the estimation points array.
         */

        dl_surface_t( void );

        /*! \brief destructor methods
         *
         *  The desctructor simply releases the memory allocated to handle the
         *  plane estimation points.
         */

        ~dl_surface_t( void );

    public:

        /*! \brief accessor methods
         *
         *  This function allows to retrieve the plane equation parameter. The
         *  normal vector components are placed in the first three components of
         *  the provided array when the constant is placed in the fourth one.
         *
         *  \param dl_equation Array receiving the plane equation parameters
         */

        void sf_get_equation( double dl_equation[4] );

    public:

        /*! \brief mutator methods
         *
         *  This function allows to push a new estimation point to the class
         *  stack. As the new point is pushed, the function asks the class to
         *  make a new estimation of the plane.
         *
         *  The \b dl_limit value indicates the minimum distance between two
         *  estimation points. If the pushed point is less distant to an already
         *  pushed point, the function removes this last point from the stack
         *  and the provided point is discarded.
         *
         *  \param dl_x     Pushed point x-coordinate
         *  \param dl_y     Pushed point y-coordinate
         *  \param dl_z     Pushed point z-coordinate
         *  \param dl_limit Estimation point minimal separation
         */

        void sf_set_point_push( double const dl_x, double const dl_y, double const dl_z, double const dl_limit );

        /*! \brief mutator methods
         *
         *  This function is used to automatically choose the point of the model
         *  that are good candidates for the estimation of the plane. It then
         *  asks the class to recompute the equation of the plane with the
         *  automatically chosen estimation points.
         *
         *  With a set of manually added estimation points, the class is able to
         *  compute the parameters of the plane. With the plane and the model
         *  points, this function is able to select in the model the points that
         *  are able to be considered as part of the plane. The \b dl_limit
         *  parameter is used as a threshold distance to select the model
         *  points.
         *
         *  This procedure allows to consider a much larger set of estimation
         *  points to compute the equation of the plane. In other words, this
         *  procedure allows to refine the estimation points set, and so, the
         *  equation of the plane that estimate a surface of the model.
         *
         *  \param dl_data  Model points array
         *  \param dl_size  Model points count
         *  \param dl_limit Distance threshold for the selection of the points
         */

        void sf_set_point_auto( char const * const dl_data, long long const dl_size, double const dl_limit );

        /*! \brief mutator methods
         *
         *  This function clears the surface estimation points array.
         */

        void sf_set_point_clear( void );

    private:

        /*! \brief mutator methods
         *
         *  This function considers a potential pushed estimation point and
         *  checks in the plane estimation points array if the pushed point is
         *  less distant to an already pushed point that the \b dl_limit value.
         *  If so, the found point is removed from the stack and the pushed
         *  candidates is discarded.
         *
         *  This function allows to remove an estimation point from the surface
         *  array just by provided an estimation of its position.
         *
         *  If a point is removed from the surface estimation points array, the
         *  function returns true, allowing subsequent processes to discard the
         *  pushed candidate.
         *
         *  \param  dl_x     Estimation point candidate x-coordinate
         *  \param  dl_y     Estimation point candidate y-coordinate
         *  \param  dl_z     Estimation point candidate z-coordinate
         *  \param  dl_limit Distance threshold for point removal
         *
         *  \return Returns true if a point has been removed, false otherwise.
         */

        bool sf_set_point_remove( double const dl_x, double const dl_y, double const dl_z, double const dl_limit );

    public:

        /*! \brief mutator methods
         *
         *  This function allows to specify the color of the plane.
         *
         *  \param dl_cr Color red component
         *  \param dl_cg Color green component
         *  \param dl_cb Color blue component
         */

        void sf_set_color( float const dl_cr, float const dl_cg, float const dl_cb );

    private:

        /*! \brief mutator methods
         *
         *  This function is called to compute the plane parameters based on the
         *  estimation points of the surface.
         *
         *  The function computes the u matrix of the svd factorisation of the
         *  matrix build with the estimation points compoents. It then extracts
         *  the plane normal from the u matrix and uses the estimation points
         *  centroid to compute the plane constant parameter.
         *
         *  It also recompute the u and v vector that offer an orthonormal basis
         *  of the computed plane.
         */

        void sf_set_equation( void );

        /*! \brief mutator methods
         *
         *  This function is used to handle the surface estimation points array
         *  memory. It is typically called as an estimation point is pushed on
         *  the stack. The function checks the memory availability for the new
         *  point and reallocate the memory when necessary.
         */

        void sf_set_memory( void );

        /*! \brief mutator methods
         *
         *  This function simply release the memory used to store the plane
         *  estimation points.
         */

        void sf_set_release( void );

    public:

        /*! \brief render methods
         *
         *  This function renders, through opengl api, the plane hold by the
         *  class. It represents the plane using a simple gl_quad built using
         *  the estimation points centroid and the two orthonormal vectors hold
         *  by the class. The surface color is used as the gl_quad color in
         *  addition to transparency (blending).
         *
         *  The \b dl_w value is used as a multiplier of the unit vectors used
         *  to compute the summits of the plane gl_quand representation.
         *
         *  \param dl_w Size of the plane representation
         */

        void sf_ren_surface( double const dl_w );

        /*! \brief render methods
         *
         *  This function is used to render, through opengl api, the estimation
         *  points of the plane. It simply renders the points using the plane
         *  color.
         */

        void sf_ren_point( void );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

