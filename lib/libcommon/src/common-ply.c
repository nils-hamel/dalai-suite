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

    # include "common-ply.h"

/*
    source - constructor/destructor methods
 */

    lc_ply_t lc_ply_create( unsigned char const * const lc_path ) {

        /* created structure variables */
        lc_ply_t lc_ply = LC_PLY_C;

        /* create ply handle */
        if ( ( lc_ply.ph_handle = fopen( ( char * ) lc_path, "rb" ) ) == NULL ) {

            /* send message */
            return( lc_ply._status = LC_FALSE, lc_ply );

        }

        /* read ply header */
        if ( lc_ply_io_i_header( & lc_ply ) == LC_FALSE ) {

            /* send message */
            return( lc_ply._status = LC_FALSE, lc_ply );

        }

        /* return created structure */
        return( lc_ply );

    }

    void lc_ply_delete( lc_ply_t * const lc_ply ) {

        /* deleted structure variables */
        lc_ply_t lc_delete = LC_PLY_C;

        /* check ply handle */
        if ( lc_ply->ph_handle != NULL ) {

            /* close ply handle */
            fclose( lc_ply->ph_handle );

        }

        /* check chunk allocation */
        if ( lc_ply->ph_chunk != NULL ) {

            /* release memory */
            free( lc_ply->ph_chunk );

        }

        /* delete structure */
        ( * lc_ply ) = lc_delete;

    }

/*
    source - accessor methods
 */

    double lc_ply_get_x( lc_ply_t const * const lc_ply, long long int lc_index ) {

        /* return x-value of chunk */
        return( * ( ( float * ) ( lc_ply->ph_chunk + ( lc_index * lc_ply->ph_vsize ) + lc_ply->ph_vdata[0] ) ) );

    }

    double lc_ply_get_y( lc_ply_t const * const lc_ply, long long int lc_index ) {

        /* return y-value of chunk */
        return( * ( ( float * ) ( lc_ply->ph_chunk + ( lc_index * lc_ply->ph_vsize ) + lc_ply->ph_vdata[1] ) ) );

    }

    double lc_ply_get_z( lc_ply_t const * const lc_ply, long long int lc_index ) {

        /* return z-value of chunk */
        return( * ( ( float * ) ( lc_ply->ph_chunk + ( lc_index * lc_ply->ph_vsize ) + lc_ply->ph_vdata[2] ) ) );

    }

    char lc_ply_get_red( lc_ply_t const * const lc_ply, long long int lc_index ) {

        /* return red component of chunk */
        return( * ( ( char * ) ( lc_ply->ph_chunk + ( lc_index * lc_ply->ph_vsize ) + lc_ply->ph_vdata[3] ) ) );

    }

    char lc_ply_get_green( lc_ply_t const * const lc_ply, long long int lc_index ) {

        /* return red component of chunk */
        return( * ( ( char * ) ( lc_ply->ph_chunk + ( lc_index * lc_ply->ph_vsize ) + lc_ply->ph_vdata[4] ) ) );

    }

    char lc_ply_get_blue( lc_ply_t const * const lc_ply, long long int lc_index ) {

        /* return red component of chunk */
        return( * ( ( char * ) ( lc_ply->ph_chunk + ( lc_index * lc_ply->ph_vsize ) + lc_ply->ph_vdata[5] ) ) );

    }

/*
    source - i/o methods
 */

    int lc_ply_io_i_header( lc_ply_t * const lc_ply ) {

        /* token buffer variables */
        unsigned char lc_token[256] = { 0 };

        /* token pointer variables */
        unsigned char * lc_accum = lc_token;

        /* char variables */
        int lc_char = 0;

        /* mode variables */
        int lc_mode = LC_PLY_RM_DETECT;
        int lc_smod = LC_PLY_RM_STANDARD;

        /* type stack */
        int lc_type = LC_PLY_TP_NONE;

        int lc_property = 0;

        /* check consistency */
        if ( lc_ply->ph_handle == NULL ) {

            /* send message */
            return( LC_FALSE );

        }

        /* reading header */
        while ( LC_TRUE ) {

            /* read header char */
            lc_char = fgetc( lc_ply->ph_handle );

            /* check and append char */
            if ( lc_char > 0x20 ) ( * lc_accum ++ ) = lc_char;

            /* check end of word */
            if ( ( lc_char == 0x20 ) || ( lc_char == 0x0a ) ) {

                /* validate token */
                ( * lc_accum ) = 0x00;

                /* check mode */
                if ( lc_mode == LC_PLY_RM_DETECT ) {

                    /* analyse token */
                    if ( strcmp( ( char * ) lc_token, "ply" ) != 0 ) {

                        /* send message */
                        return( LC_FALSE );

                    }

                    /* update mode */
                    lc_mode = LC_PLY_RM_STANDARD;

                } else if ( lc_mode == LC_PLY_RM_STANDARD ) {

                    /* analyse token */
                    if ( strcmp( ( char * ) lc_token, "end_header" ) == 0 ) {

                        /* initialise position */
                        lc_ply->ph_position = ftell( lc_ply->ph_handle );

                        /* send message */
                        return( LC_TRUE );

                    } else if ( strcmp( ( char * ) lc_token, "format" ) == 0 ) {

                        /* update mode */
                        lc_mode = LC_PLY_RM_FORMAT;

                    } else if ( strcmp( ( char * ) lc_token, "element" ) == 0 ) {

                        /* update mode */
                        lc_mode = LC_PLY_RM_ELEMENT;

                    } else if ( strcmp( ( char * ) lc_token, "property" ) == 0 ) {

                        /* update mode */
                        lc_mode = LC_PLY_RM_PROPERTY;

                    }

                } else if ( lc_mode == LC_PLY_RM_FORMAT ) {

                    /* analyse token */
                    if ( strcmp( ( char * ) lc_token, "binary_little_endian" ) == 0 ) {

                        /* update header structure */
                        lc_ply->ph_format = LC_PLY_LEBIN;

                    } else {

                        /* send message */
                        return( LC_FALSE );

                    }

                    /* update mode */
                    lc_mode = LC_PLY_RM_STANDARD;

                } else if ( lc_mode == LC_PLY_RM_ELEMENT ) {

                    if ( strcmp( ( char * ) lc_token, "vertex" ) == 0 ) {

                        /* update mode */
                        lc_mode = LC_PLY_RM_VERTEX;

                        /* update sub-mode */
                        lc_smod = LC_PLY_RM_VERTEX;

                        lc_property = 0;

                    } else {

                        /* update mode */
                        lc_mode = LC_PLY_RM_STANDARD;

                        /* update sub-mode */
                        lc_smod = LC_PLY_RM_STANDARD;

                    }

                } else if ( lc_mode == LC_PLY_RM_VERTEX ) {

                    /* converts token */
                    lc_ply->ph_vertex = strtoll( ( char * ) lc_token, NULL, 10 );

                    /* update mode */
                    lc_mode = LC_PLY_RM_STANDARD;

                } else if ( lc_mode == LC_PLY_RM_PROPERTY ) {

                    if ( strcmp( ( char * ) lc_token, "double" ) == 0 ) {

                        /* push type */
                        lc_type = LC_PLY_TP_DOUBLE;

                        /* update mode */
                        lc_mode = LC_PLY_RM_DATA;

                    } else if ( strcmp( ( char * ) lc_token, "float" ) == 0 ) {

                        /* push type */
                        lc_type = LC_PLY_TP_FLOAT;

                        /* update mode */
                        lc_mode = LC_PLY_RM_DATA;

                    } else if ( strcmp( ( char * ) lc_token, "uchar" ) == 0 ) {

                        /* push type */
                        lc_type = LC_PLY_TP_UCHAR;

                        /* update mode */
                        lc_mode = LC_PLY_RM_DATA;

                    } else {

                        /* update mode */
                        lc_mode = LC_PLY_RM_STANDARD;

                    }

                } else if ( ( lc_mode == LC_PLY_RM_DATA ) && ( lc_smod == LC_PLY_RM_VERTEX ) ) {

                    if ( strcmp( ( char * ) lc_token, "x" ) == 0 ) {

                        lc_ply->ph_vpush[0] = lc_property;
                        lc_ply->ph_vtype[0] = lc_type;
                        lc_ply->ph_vdata[0] = lc_ply->ph_vsize;

                    } else if ( strcmp( ( char * ) lc_token, "y" ) == 0 ) {

                        lc_ply->ph_vpush[1] = lc_property;
                        lc_ply->ph_vtype[1] = lc_type;
                        lc_ply->ph_vdata[1] = lc_ply->ph_vsize;

                    } else if ( strcmp( ( char * ) lc_token, "z" ) == 0 ) {

                        lc_ply->ph_vpush[2] = lc_property;
                        lc_ply->ph_vtype[2] = lc_type;
                        lc_ply->ph_vdata[2] = lc_ply->ph_vsize;

                    } else if ( strcmp( ( char * ) lc_token, "red" ) == 0 ) {

                        lc_ply->ph_vpush[3] = lc_property;
                        lc_ply->ph_vtype[3] = lc_type;
                        lc_ply->ph_vdata[3] = lc_ply->ph_vsize;

                    } else if ( strcmp( ( char * ) lc_token, "green" ) == 0 ) {

                        lc_ply->ph_vpush[4] = lc_property;
                        lc_ply->ph_vtype[4] = lc_type;
                        lc_ply->ph_vdata[4] = lc_ply->ph_vsize;

                    } else if ( strcmp( ( char * ) lc_token, "blue" ) == 0 ) {

                        lc_ply->ph_vpush[5] = lc_property;
                        lc_ply->ph_vtype[5] = lc_type;
                        lc_ply->ph_vdata[5] = lc_ply->ph_vsize;

                    } else if ( strcmp( ( char * ) lc_token, "alpha" ) == 0 ) {

                        lc_ply->ph_vpush[6] = lc_property;
                        lc_ply->ph_vtype[6] = lc_type;
                        lc_ply->ph_vdata[6] = lc_ply->ph_vsize;

                    } else {

                        return( LC_FALSE );

                    }

                    /* update size */
                    switch ( lc_type ) {

                        case LC_PLY_TP_DOUBLE : { lc_ply->ph_vsize += 8; } break;
                        case LC_PLY_TP_FLOAT  : { lc_ply->ph_vsize += 4; } break;
                        case LC_PLY_TP_UCHAR  : { lc_ply->ph_vsize += 1; } break;

                    };

                    /* update property count */
                    lc_property ++;

                    /* update mode */
                    lc_mode = LC_PLY_RM_STANDARD;

                }

                /* reset token */
                lc_accum = lc_token;

            }

        }

        /* send message */
        return( LC_TRUE );

    }

    long long int lc_ply_io_i_chunk( lc_ply_t * const lc_ply, long long int const lc_size ) {

        /* returned value variables */
        long long int lc_return = 0;

        /* memory swap variables */
        unsigned char * lc_swap = NULL;

        /* check consistency */
        if ( lc_ply->ph_position >= ( lc_ply->ph_vertex * lc_ply->ph_vsize ) ) {

            /* send message */
            return( 0 );

        }

        /* create chuck allocation */
        if ( ( lc_swap = realloc( lc_ply->ph_chunk, lc_size * lc_ply->ph_vsize ) ) == NULL ) {

            /* send message */
            return( 0 );

        }

        /* assign allocated memory */
        lc_ply->ph_chunk = lc_swap;

        /* seek to position */
        fseek( lc_ply->ph_handle, lc_ply->ph_position, SEEK_SET );

        /* read and check chunck */
        lc_return = fread( lc_ply->ph_chunk, 1, lc_size * lc_ply->ph_vsize, lc_ply->ph_handle );

        /* update stream position */
        lc_ply->ph_position += lc_return;

        /* returned readed records */
        return( lc_return / lc_ply->ph_vsize );

    }

