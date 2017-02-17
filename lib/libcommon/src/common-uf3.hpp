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

    # ifndef __LC_UF3__
    # define __LC_UF3__

/*
    header - internal includes
 */

    # include "common.hpp"

/*
    header - external includes
 */

    # include <cstdint>

/*
    header - preprocessor definitions
 */

    /* define uf3 records length */
    # define LC_UF3_RECLEN ( ( sizeof( double ) + sizeof( uint8_t ) ) * 3u )

    /* define uf3 records offsets */
    # define LC_UF3_POSE   ( 0 )
    # define LC_UF3_DATA   ( sizeof( double ) * 3u )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

    /* define uf3 types */
    typedef double  lc_uf3p_t;
    typedef uint8_t lc_uf3d_t;

/*
    header - structures
 */

/*
    header - function prototypes
 */

/*
    header - inclusion guard
 */

    # endif

