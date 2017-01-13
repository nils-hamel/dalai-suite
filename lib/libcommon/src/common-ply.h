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

    /*! \file   common-ply.h
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - common library - ply module
     */

/*
    header - inclusion guard
 */

    # ifndef __LC_PLY__
    # define __LC_PLY__

/*
    header - C/C++ compatibility
 */

    # ifdef __cplusplus
    extern "C" {
    # endif

/*
    header - internal includes
 */

    # include "common.h"

/*
    header - external includes
 */

    # include <stdio.h>
    # include <stdlib.h>
    # include <string.h>

/*
    header - preprocessor definitions
 */

    /* define pseudo-constructor */
    # define LC_PLY_C           { NULL, 0, 0, 0, { 0 }, { 0 }, 0, NULL, LC_TRUE }

    /* define ply format */
    # define LC_PLY_ASCII       ( 1 )
    # define LC_PLY_BINARY_LE   ( 2 )
    # define LC_PLY_BINARY_BE   ( 3 )

    /* define header reading modes */
    # define LC_PLY_RM_VALIDATE ( 0 )
    # define LC_PLY_RM_DETECT   ( 1 )
    # define LC_PLY_RM_STANDARD ( 2 )
    # define LC_PLY_RM_FORMAT   ( 3 )
    # define LC_PLY_RM_ELEMENT  ( 4 )
    # define LC_PLY_RM_VERTEX   ( 5 )
    # define LC_PLY_RM_PROPERTY ( 6 )
    # define LC_PLY_RM_DATA     ( 7 )

    /* define data types */
    # define LC_PLY_TP_NONE     ( 0  )
    # define LC_PLY_TP_DOUBLE   ( 1  )
    # define LC_PLY_TP_FLOAT    ( 2  )
    # define LC_PLY_TP_UCHAR    ( 3  )
    # define LC_PLY_TP_CHAR     ( 4  )
    # define LC_PLY_TP_USHORT   ( 5  )
    # define LC_PLY_TP_SHORT    ( 6  )
    # define LC_PLY_TP_UINT     ( 7  )
    # define LC_PLY_TP_INT      ( 8  )
    # define LC_PLY_TP_ULONG    ( 9  )
    # define LC_PLY_TP_LONG     ( 10 )

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

    typedef struct lc_ply_header_struct {

        FILE *        ph_handle;

        int           ph_format;

        long long int ph_vertex;

        int           ph_vsize;
        //int           ph_vpush[6];
        int           ph_vtype[6];
        int           ph_vdata[6];

        long long int ph_position;

        char *        ph_chunk;

    int _status; } lc_ply_header_t, lc_ply_t;

/*
    header - function prototypes
 */

    lc_ply_t lc_ply_create( unsigned char const * const lc_path );

    void lc_ply_delete( lc_ply_t * const lc_ply );

    double lc_ply_get_x( lc_ply_t const * const lc_ply, long long int lc_index );

    double lc_ply_get_y( lc_ply_t const * const lc_ply, long long int lc_index );

    double lc_ply_get_z( lc_ply_t const * const lc_ply, long long int lc_index );

    char lc_ply_get_red( lc_ply_t const * const lc_ply, long long int lc_index );

    char lc_ply_get_green( lc_ply_t const * const lc_ply, long long int lc_index );

    char lc_ply_get_blue( lc_ply_t const * const lc_ply, long long int lc_index );

    int lc_ply_io_i_header( lc_ply_t * const lc_ply );

    long long int lc_ply_io_i_chunk( lc_ply_t * const lc_ply, long long int const lc_size );

/*
    header - C/C++ compatibility
 */

    # ifdef __cplusplus
    }
    # endif

/*
    header - inclusion guard
 */

    # endif

