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
    # include <cmath>
    # include <GL/gl.h>
    # include <GL/glu.h>
    # include <Eigen/Dense>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define memory segment */
    # define DL_SURFACE_STEP ( 6561 )

    /* define estimation minimum */
    # define DL_SURFACE_MIN  ( 12 )

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
     *  of the plane and its estimation points through the OpenGL API.
     *
     *  \var dl_surface_t::sf_cx
     *  Estimation point centroid
     *  \var dl_surface_t::sf_cy
     *  Estimation point centroid
     *  \var dl_surface_t::sf_cz
     *  Estimation point centroid
     *  \var dl_surface_t::sf_px
     *  Surface normal vector
     *  \var dl_surface_t::sf_py
     *  Surface normal vector
     *  \var dl_surface_t::sf_pz
     *  Surface normal vector
     *  \var dl_surface_t::sf_pc
     *  Surface equation constant
     *  \var dl_surface_t::sf_ux
     *  Surface base vector
     *  \var dl_surface_t::sf_uy
     *  Surface base vector
     *  \var dl_surface_t::sf_uz
     *  Surface base vector
     *  \var dl_surface_t::sf_vx
     *  Surface base vector
     *  \var dl_surface_t::sf_vy
     *  Surface base vector
     *  \var dl_surface_t::sf_vz
     *  Surface base vector
     *  \var dl_surface_t::sf_radius
     *  Estimation point maximum distance from centroid
     *  \var dl_surface_t::sf_r
     *  Surface color red component
     *  \var dl_surface_t::sf_g
     *  Surface color green component
     *  \var dl_surface_t::sf_b
     *  Surface color blue component
     *  \var dl_surface_t::sf_size
     *  Number of estimation points
     *  \var dl_surface_t::sf_virt
     *  Estimation points array size
     *  \var dl_surface_t::sf_data
     *  Estimation points array data
     */

    class dl_surface_t {

    private:
        le_real_t   sf_cx;
        le_real_t   sf_cy;
        le_real_t   sf_cz;
        le_real_t   sf_px;
        le_real_t   sf_py;
        le_real_t   sf_pz;
        le_real_t   sf_pc;
        le_real_t   sf_ux;
        le_real_t   sf_uy;
        le_real_t   sf_uz;
        le_real_t   sf_vx;
        le_real_t   sf_vy;
        le_real_t   sf_vz;
        le_real_t   sf_radius;
        le_real_t   sf_r;
        le_real_t   sf_g;
        le_real_t   sf_b;
        le_size_t   sf_size;
        le_size_t   sf_virt;
        le_real_t * sf_data;

    public:

        /*! \brief constructor methods
         *
         *  The constructor simply initialises the class members to default
         *  values.
         */

        dl_surface_t( void );

        /*! \brief destructor methods
         *
         *  The desctructor simply releases the memory allocated to handle the
         *  surface estimation points.
         */

        ~dl_surface_t( void );

    public:

        /*! \brief accessor methods
         *
         *  This function is used to compute the intersection point between the
         *  current surface and the two provided ones. The intersection is then
         *  written in the standard output.
         *
         *  \param dl_s2 Surface object
         *  \param dl_s3 Surface object
         */

        le_void_t sf_get_intersection( dl_surface_t & dl_s2, dl_surface_t & dl_s3 );

    public:

        /*! \brief mutator methods
         *
         *  This function allows to push a new estimation point to the object
         *  stack. As the new point is pushed, the function asks the object to
         *  make a new estimation of the plane.
         *
         *  The \b dl_tolerance value indicates the minimum distance between two
         *  estimation points. If the pushed point is less distant to an already
         *  pushed point, the function removes this last point from the stack
         *  and the provided point is discarded.
         *
         *  \param dl_x         Pushed point coordinate
         *  \param dl_y         Pushed point coordinate
         *  \param dl_z         Pushed point coordinate
         *  \param dl_tolerance Minimal estimation points separation
         */

        le_void_t sf_set_point_push( le_real_t const dl_x, le_real_t const dl_y, le_real_t const dl_z, le_real_t const dl_tolerance );

        /*! \brief mutator methods
         *
         *  This function is used to automatically choose points of the model
         *  that are good candidates for the estimation of the plane. It then
         *  asks the object to recompute the equation of the plane with the
         *  automatically chosen estimation points.
         *
         *  With a set of manually added estimation points, the object is able
         *  to compute the parameters of the plane. With the plane and the model
         *  points, this function is able to select in the model the points that
         *  are able to be considered as part of the plane. The \b dl_tolerance
         *  parameter is used as a threshold distance to select the model
         *  points.
         *
         *  The provided \b dl_grow parameter is the 'delta' distance added to
         *  the estimation points maximal distance to centroid that allow a
         *  a automatic selection. If the value is greater than zero, the area
         *  of the estimation set should grow.
         *
         *  This procedure allows to consider a much larger set of estimation
         *  points to compute the equation of the plane. In other words, this
         *  procedure allows to refine the estimation points set, and so, the
         *  equation of the plane that estimate a surface in the model.
         *
         *  \param dl_data      Model points array
         *  \param dl_size      Model points count
         *  \param dl_tolerance Point selection distance threshold
         *  \param dl_grow      Selection condition tolerance
         */

        le_void_t sf_set_point_auto( le_byte_t const * const dl_data, le_size_t const dl_size, le_real_t const dl_tolerance, le_real_t const dl_grow );

    private:

        /*! \brief mutator methods
         *
         *  This function considers a potential pushed estimation point and
         *  checks in the surface estimation points array if the pushed point is
         *  less distant to an already pushed point than the \b dl_tolerance
         *  value. If so, the found point is removed from the stack and the
         *  pushed candidate is discarded.
         *
         *  This function allows to remove an estimation point from the surface
         *  array just by provided an estimation of its position.
         *
         *  If a point is removed from the surface estimation points array, the
         *  function returns true, allowing subsequent processes to discard the
         *  pushed candidate.
         *
         *  \param dl_x         Pushed point coordinate
         *  \param dl_y         Pushed point coordinate
         *  \param dl_z         Pushed point coordinate
         *  \param dl_tolerance Point selection distance threshold
         *
         *  \return Returns true if a point has been removed, false otherwise.
         */

        bool sf_set_point_remove( le_real_t const dl_x, le_real_t const dl_y, le_real_t const dl_z, le_real_t dl_tolerance );

    public:

        /*! \brief mutator methods
         *
         *  This function clears the surface estimation points array.
         */

        le_void_t sf_set_point_clear( le_void_t );

        /*! \brief mutator methods
         *
         *  This function allows to specify the color of the surface.
         *
         *  \param dl_r Color red component
         *  \param dl_g Color green component
         *  \param dl_b Color blue component
         */

        le_void_t sf_set_color( le_real_t const dl_r, le_real_t const dl_g, le_real_t const dl_b );

    private:

        /*! \brief mutator methods
         *
         *  This function is called to compute the plane parameters based on the
         *  estimation points of the surface.
         *
         *  The function computes the u matrix of the svd factorisation of the
         *  matrix build with the estimation points components. It then extracts
         *  the plane normal from the u matrix and uses the estimation points
         *  centroid to compute the plane constant parameter.
         *
         *  It also recompute the u and v vector that offer an orthonormal basis
         *  of the computed plane.
         */

        le_void_t sf_set_equation( le_void_t );

        /*! \brief mutator methods
         *
         *  This function is used to handle the surface estimation points array
         *  memory. It is typically called as an estimation point is pushed on
         *  the stack. The function checks the memory availability for the new
         *  point and re-allocate the memory when necessary.
         *
         *  \param dl_add Number of element to push
         */

        le_void_t sf_set_memory( le_size_t const dl_add );

        /*! \brief mutator methods
         *
         *  This function simply release the memory used to store the plane
         *  estimation points.
         */

        le_void_t sf_set_release( le_void_t );

        /*! \brief mutator methods
         *
         *  This function simply updates the OpenGL point display size using the
         *  provided factor.
         *
         *  \param dl_factor Point size multiplication factor
         */

        le_void_t sf_set_pointsize( le_real_t const dl_factor );

    public:

        /*! \brief rendering methods
         *
         *  This function is used to render, through OpenGL API, the surface
         *  hold by the object. The function checks in the first place if the
         *  equation of the surface is computed (sufficient amount of estimation
         *  points). The surface is then rendered.
         *
         *  This function is responsible of non-transparent element rendering.
         */

        le_void_t sf_ren_surface( le_void_t );

        /*! \brief rendering methods
         *
         *  This function is used to render, through OpenGL API, the surface
         *  hold by the object. The function checks in the first place if the
         *  equation of the surface is computed (sufficient amount of estimation
         *  points). The surface is then rendered.
         *
         *  This function is responsible of transparent element rendering.
         */

        le_void_t sf_ren_blend( le_void_t );

    };

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

