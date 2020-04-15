/*
 *  dalai-suite - uf3-uv3
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
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

    /*! \file   dalai-uf3-uv3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - uf3-uv3
     */

    /*! \mainpage dalai-suite
     *
     *  \section overview Overview
     *
     *  The _dalai-suite_ offers a toolbox for large scale 3D models
     *  manipulation, processing, conversion and geographical registration. It
     *  is mainly designed to offers solutions suitable for models that can
     *  weight hundreds of gigabytes and that are used in the domain of
     *  environment survey and land registering. It was initially developed to
     *  offer the required tools needed to allows massive data processing and
     *  preparation for the _Eratosthene Project_.
     *
     *  The _dalai-suite_ uses a common and very simple format for all the
     *  implemented tools that is also used for the _Eratosthene Project_. This
     *  format offers a simple way of storing massive amount of graphical
     *  primitives. This allows the _dalai-suite_ to manipulate all sort of 3D
     *  models, from large scale point-based models to more refined vector
     *  models.
     *
     *  The _dalai-suite_ comes also with tools dedicated to models
     *  visualization and large scale models processing such as automatic
     *  segmentation, cleaning, geographical transformation and assisted
     *  geographical registration.
     *
     *  The _dalai-suite_ also comes with a set of tools dedicated to file
     *  format conversion that allows data coming in their specific formats to
     *  be converted in the _suite_ format.
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2020 DHLAB, EPFL
     *
     *  This program is licensed under the terms of the GNU GPLv3. Documentation
     *  and illustrations are licensed under the terms of the CC BY 4.0.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_UF3_UV3__
    # define __DL_UF3_UV3__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define uf3 element */
    # define DL_UF3_POSE   ( sizeof( le_real_t ) * 3 )
    # define DL_UF3_DATA   ( sizeof( le_data_t ) * 3 )

    /* define uf3 record */
    # define DL_UF3_RECORD ( DL_UF3_POSE + DL_UF3_DATA )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

/*
    header - function prototypes
 */

    /*! \brief main function
     *
     *  The main function reads the provided uf3 file and converts it into an
     *  uv3 stream :
     *
     *      ./dalai-uf3-uv3 --input/-i [uf3 input file]
     *                      --output/-o [uv3 output file]
     *
     *  The main function starts by creating the two input and output streams
     *  before to start the conversion. The input file is read by chunks and
     *  each chunk is converted from uf3 to uv3 format. The converted chunks are
     *  then exported in the output file.
     *
     *  This software is design to ensure compatibility with the previous uf3
     *  format used by the dalai-suite.
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

