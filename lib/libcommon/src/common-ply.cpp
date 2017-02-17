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

    # include "common-ply.hpp"

/*
    source - constructor/destructor methods
 */

    lc_ply_t lc_ply_create( unsigned char const * const lc_path ) {

        /* created structure variables */
        lc_ply_t lc_ply = LC_PLY_C;

        /* create and check stream handle */
        if ( ( lc_ply.ph_handle = fopen( ( char * ) lc_path, "rb" ) ) == NULL ) {

            /* send message */
            return( lc_ply._status = LC_FALSE, lc_ply );

        }

        /* read and check stream header */
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

        /* check stream handle */
        if ( lc_ply->ph_handle != NULL ) {

            /* close ply handle */
            fclose( lc_ply->ph_handle );

        }

        /* check chunk memory allocation */
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

        /* returned value variables */
        int lc_return = LC_TRUE;

        /* char variables */
        unsigned char lc_char = 0;

        /* token buffer variables */
        unsigned char lc_token[256] = { 0 };

        /* token pointer variables */
        unsigned char * lc_accum = lc_token;

        /* properties type variables */
        int lc_type = LC_PLY_TP_NONE;

        /* reading mode variables */
        int lc_mode = LC_PLY_RM_DETECT;
        int lc_smod = LC_PLY_RM_STANDARD;

        /* check consistency */
        if ( lc_ply->ph_handle == NULL ) {

            /* send message */
            return( LC_FALSE );

        }

        /* reading header */
        while ( lc_mode != LC_PLY_RM_VALIDATE ) {

            /* read stream character */
            lc_char = fgetc( lc_ply->ph_handle );

            /* check and append character */
            if ( lc_char > 0x20 ) ( * lc_accum ++ ) = lc_char;

            /* search for separation characters */
            if ( ( lc_char != 0x20 ) && ( lc_char != 0x0a ) ) continue;

            /* validate token */
            ( * lc_accum ) = 0x00;

            /* switch on mode */
            if ( lc_mode == LC_PLY_RM_DETECT ) {

                /* analyse validated token */
                if ( strcmp( ( char * ) lc_token, "ply" ) != 0 ) {

                    /* send message */
                    return( LC_FALSE );

                }

                /* update mode */
                lc_mode = LC_PLY_RM_STANDARD;

            } else
            if ( lc_mode == LC_PLY_RM_STANDARD ) {

                /* analyse validated token */
                if ( strcmp( ( char * ) lc_token, "end_header" ) == 0 ) {

                    /* stream data offset */
                    lc_ply->ph_position = ftell( lc_ply->ph_handle );

                    /* update mode */
                    lc_mode = LC_PLY_RM_VALIDATE;

                } else
                if ( strcmp( ( char * ) lc_token, "format" ) == 0 ) {

                    /* update mode */
                    lc_mode = LC_PLY_RM_FORMAT;

                } else
                if ( strcmp( ( char * ) lc_token, "element" ) == 0 ) {

                    /* update mode */
                    lc_mode = LC_PLY_RM_ELEMENT;

                } else
                if ( strcmp( ( char * ) lc_token, "property" ) == 0 ) {

                    /* check element type */
                    if ( lc_smod == LC_PLY_RM_VERTEX ) {

                        /* update mode */
                        lc_mode = LC_PLY_RM_PROPERTY;

                    }

                }

            } else
            if ( lc_mode == LC_PLY_RM_FORMAT ) {

                /* analyse validated token */
                if ( strcmp( ( char * ) lc_token, "binary_little_endian" ) == 0 ) {

                    /* retrieve stream format */
                    lc_ply->ph_format = LC_PLY_BINARY_LE;

                    /* update mode */
                    lc_mode = LC_PLY_RM_STANDARD;

                } else {

                    /* push message */
                    lc_return = LC_FALSE;

                    /* updated mode */
                    lc_mode = LC_PLY_RM_VALIDATE;

                }

            } else
            if ( lc_mode == LC_PLY_RM_ELEMENT ) {

                /* analyse validated token */
                if ( strcmp( ( char * ) lc_token, "vertex" ) == 0 ) {

                    /* update mode */
                    lc_mode = LC_PLY_RM_VERTEX;

                    /* update sub-mode */
                    lc_smod = LC_PLY_RM_VERTEX;

                } else {

                    /* update mode */
                    lc_mode = LC_PLY_RM_STANDARD;

                    /* update sub-mode */
                    lc_smod = LC_PLY_RM_STANDARD;

                }

            } else
            if ( lc_mode == LC_PLY_RM_VERTEX ) {

                /* retrieve vertex count */
                lc_ply->ph_vertex = strtoll( ( char * ) lc_token, NULL, 10 );

                /* update mode */
                lc_mode = LC_PLY_RM_STANDARD;

            } else
            if ( lc_mode == LC_PLY_RM_PROPERTY ) {

                /* analyse validated token */
                if ( ( strcmp( ( char * ) lc_token, "double" ) == 0 ) || ( strcmp( ( char * ) lc_token, "float64" ) == 0 ) )  {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_DOUBLE;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "float" ) == 0 ) || ( strcmp( ( char * ) lc_token, "float32" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_FLOAT;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "uchar" ) == 0 ) || ( strcmp( ( char * ) lc_token, "uint8" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_UCHAR;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "char" ) == 0 ) || ( strcmp( ( char * ) lc_token, "int8" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_CHAR;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "ushort" ) == 0 ) || ( strcmp( ( char * ) lc_token, "uint16" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_USHORT;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "short" ) == 0 ) || ( strcmp( ( char * ) lc_token, "int16" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_SHORT;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "uint" ) == 0 ) || ( strcmp( ( char * ) lc_token, "uint32" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_UINT;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "int" ) == 0 ) || ( strcmp( ( char * ) lc_token, "int32" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_INT;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "ulong" ) == 0 ) || ( strcmp( ( char * ) lc_token, "uint64" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_ULONG;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else
                if ( ( strcmp( ( char * ) lc_token, "long" ) == 0 ) || ( strcmp( ( char * ) lc_token, "int64" ) == 0 ) ) {

                    /* retrieve property type */
                    lc_type = LC_PLY_TP_LONG;

                    /* update mode */
                    lc_mode = LC_PLY_RM_DATA;

                } else {

                    /* push message */
                    lc_return = LC_FALSE;

                    /* update mode */
                    lc_mode = LC_PLY_RM_VALIDATE;

                }

            } else
            if ( lc_mode == LC_PLY_RM_DATA ) {

                /* analyse validated token */
                if ( strcmp( ( char * ) lc_token, "x" ) == 0 ) {

                    /* retrieve data type */
                    lc_ply->ph_vtype[0] = lc_type;

                    /* retrieve data record offset */
                    lc_ply->ph_vdata[0] = lc_ply->ph_vsize;

                } else
                if ( strcmp( ( char * ) lc_token, "y" ) == 0 ) {

                    /* retrieve data type */
                    lc_ply->ph_vtype[1] = lc_type;

                    /* retrieve data record offset */
                    lc_ply->ph_vdata[1] = lc_ply->ph_vsize;

                } else
                if ( strcmp( ( char * ) lc_token, "z" ) == 0 ) {

                    /* retrieve data type */
                    lc_ply->ph_vtype[2] = lc_type;

                    /* retrieve data record offset */
                    lc_ply->ph_vdata[2] = lc_ply->ph_vsize;

                } else
                if ( strcmp( ( char * ) lc_token, "red" ) == 0 ) {

                    /* retrieve data type */
                    lc_ply->ph_vtype[3] = lc_type;

                    /* retrieve data record offset */
                    lc_ply->ph_vdata[3] = lc_ply->ph_vsize;

                } else
                if ( strcmp( ( char * ) lc_token, "green" ) == 0 ) {

                    /* retrieve data type */
                    lc_ply->ph_vtype[4] = lc_type;

                    /* retrieve data record offset */
                    lc_ply->ph_vdata[4] = lc_ply->ph_vsize;

                } else
                if ( strcmp( ( char * ) lc_token, "blue" ) == 0 ) {

                    /* retrieve data type */
                    lc_ply->ph_vtype[5] = lc_type;

                    /* retrieve data record offset */
                    lc_ply->ph_vdata[5] = lc_ply->ph_vsize;

                }

                /* switch on data type */
                switch ( lc_type ) {

                    /* update record size */
                    case LC_PLY_TP_DOUBLE : { lc_ply->ph_vsize += 8; } break;
                    case LC_PLY_TP_FLOAT  : { lc_ply->ph_vsize += 4; } break;
                    case LC_PLY_TP_UCHAR  : { lc_ply->ph_vsize += 1; } break;
                    case LC_PLY_TP_CHAR   : { lc_ply->ph_vsize += 1; } break;
                    case LC_PLY_TP_USHORT : { lc_ply->ph_vsize += 2; } break;
                    case LC_PLY_TP_SHORT  : { lc_ply->ph_vsize += 2; } break;
                    case LC_PLY_TP_UINT   : { lc_ply->ph_vsize += 4; } break;
                    case LC_PLY_TP_INT    : { lc_ply->ph_vsize += 4; } break;
                    case LC_PLY_TP_ULONG  : { lc_ply->ph_vsize += 8; } break;
                    case LC_PLY_TP_LONG   : { lc_ply->ph_vsize += 8; } break;

                };

                /* update mode */
                lc_mode = LC_PLY_RM_STANDARD;

            }

            /* reset token */
            lc_accum = lc_token;

        }

        /* send message */
        return( lc_return );

    }

    long long int lc_ply_io_i_chunk( lc_ply_t * const lc_ply, long long int const lc_size ) {

        /* memory swap variables */
        char * lc_swap = NULL;

        /* returned value variables */
        long long int lc_return = 0;

        /* check chunk reading consistency */
        if ( lc_ply->ph_position >= ( lc_ply->ph_vertex * lc_ply->ph_vsize ) ) {

            /* send message */
            return( 0 );

        }

        /* chunk memory allocation management */
        if ( ( lc_swap = ( char * ) realloc( lc_ply->ph_chunk, lc_size * lc_ply->ph_vsize ) ) == NULL ) {

            /* send message */
            return( 0 );

        }

        /* assign chunk memory allocation */
        lc_ply->ph_chunk = lc_swap;

        /* set reading pointer to chunk */
        fseek( lc_ply->ph_handle, lc_ply->ph_position, SEEK_SET );

        /* read stream chunk */
        lc_return = fread( lc_ply->ph_chunk, 1, lc_size * lc_ply->ph_vsize, lc_ply->ph_handle );

        /* update stream chunk position */
        lc_ply->ph_position += lc_return;

        /* return amount of read records */
        return( lc_return / lc_ply->ph_vsize );

    }

