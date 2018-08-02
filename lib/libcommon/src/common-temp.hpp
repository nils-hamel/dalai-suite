/*
 *  dalai-suite - common library
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

    /*! \file   common-temp.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - temporary
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_TEMP__
    # define __LC_TEMP__

/*
    header - internal includes
 */

    # include "common-error.hpp"

/*
    header - external includes
 */

    # include <iostream>
    # include <cstdlib>
    # include <unistd.h>

/*
    header - preprocessor definitions
 */

    /* define mode */
    # define LC_TEMP_CREATE ( 0 )
    # define LC_TEMP_DELETE ( 1 )

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

    /*! \brief temporary methods
     *
     *  This function allows to create a temporary directory. If the provided
     *  mode value is set to \b LC_TEMP_CREATE, the directory is created,
     *  deleted otherwise.
     *
     *  The \b lc_root directory path specify in which directory to create the
     *  temporary one. If a null is provided, the function assume the standard
     *  UNIX path as root directory (/tmp/).
     *
     *  The path of the created temporary directory is set in the \b lc_path
     *  variable, allowing to use the created directory.
     *
     *  \param lc_root Temporary directory location
     *  \param lc_path Path of the created temporary directory
     *  \param lc_mode Temporary directory creation / deletion flag
     */

    void lc_temp_directory( char const * const lc_root, char * const lc_path, int const lc_mode );

/*
    header - inclusion guard
 */

    # endif

