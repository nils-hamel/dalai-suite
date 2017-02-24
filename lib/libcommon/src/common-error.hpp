/*
 *  dalai-suite - common library
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

    /*! \file   common-args.h
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - arguments and parameters module
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_ERROR__
    # define __LC_ERROR__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>

/*
    header - preprocessor definitions
 */

    /* define errors code */
    # define LC_ERROR_NONE      ( 0 )
    # define LC_ERROR_MEMORY    ( 1 )
    # define LC_ERROR_IO_ACCESS ( 2 )
    # define LC_ERROR_IO_READ   ( 3 )
    # define LC_ERROR_IO_WRITE  ( 4 )
    # define LC_ERROR_IO_REMOVE ( 5 )
    # define LC_ERROR_FORMAT    ( 6 )

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

    void lc_error( int lc_code );

/*
    header - inclusion guard
 */

    # endif

