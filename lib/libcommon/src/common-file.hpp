/*
 *  dalai-suite - common library
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

    /*! \file   common-temp.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - file
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_FILE__
    # define __LC_FILE__

/*
    header - internal includes
 */

    # include "common-error.hpp"

/*
    header - external includes
 */

    # include <sys/stat.h>

/*
    header - preprocessor definitions
 */

    /* define filesystem basics */
    # define LC_OTHER     ( 0 )
    # define LC_FILE      ( 1 )
    # define LC_DIRECTORY ( 2 )

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

    /*! \brief filesystem methods
     *
     *  This function detects if the provided path points to a regular file or
     *  to a directory. If the path to a regular file, the function returns the
     *  \b LC_FILE value. In case of a directory, the \b LC_DIRECTORY value is
     *  returned.
     *
     *  The the pointed entity is neither a regular file nor directory, the
     *  function returns the \b LC_OTHER value.
     *
     *  \param lc_path Entity path
     *
     *  \return Returns LC_FILE on file and LC_DIRECTORY on directory
     */

    int lc_file_detect( char const * const lc_path );

/*
    header - inclusion guard
 */

    # endif

