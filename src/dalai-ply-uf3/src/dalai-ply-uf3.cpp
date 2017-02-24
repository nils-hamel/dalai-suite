/*
 *  dalai-suite - ply-uf3
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

    # include "dalai-ply-uf3.hpp"

/*
    source - constructor/destructor methods
 */

    dl_header_t::dl_header_t( char const * const dl_path )

        : hd_vertex( 0 )
        , hd_position( 0 )
        , hd_size( 0 )
        , hd_chunk( nullptr )

    {

        /* token buffer variable */
        unsigned char dl_token[256];

        /* token pointer variable */
        unsigned char * dl_accum( dl_token );

        /* properties type variable */
        int dl_type( LC_PLY_NONE );

        /* header processing variable */
        int dl_mode( LC_PLY_DETECT   );
        int dl_subm( LC_PLY_STANDARD );

        /* create input stream */
        hd_stream.open( dl_path, std::ios::in | std::ios::binary );

        /* check input stream */
        if ( hd_stream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* header analysis */
        while ( dl_mode != LC_PLY_VALIDATE ) {

            /* read stream */
            ( * dl_accum ) = hd_stream.get();

            /* character validation */
            if ( ( * dl_accum ) > 0x20 ) {

                /* validate read char */
                dl_accum ++;

                /* continue stream reading */
                continue;

            } else {

                /* detect end of token */
                if ( ( ( * dl_accum ) != 0x20 ) && ( ( * dl_accum ) != 0x0a ) ) {

                    /* continue stream reading */
                    continue;

                } else {

                    /* validate token string */
                    ( * dl_accum ) = 0x00;

                }


            }

            /* switch on mode */
            switch ( dl_mode ) {

                /* mode : format detection */
                case ( LC_PLY_DETECT ) : {

                    /* analyse token */
                    if ( std::strcmp( ( char * ) dl_token, "ply" ) != 0 ) {

                        /* send message */
                        throw( LC_ERROR_FORMAT );

                    }

                    /* update mode */
                    dl_mode = LC_PLY_STANDARD;

                } break;

                /* mode : standard reading */
                case ( LC_PLY_STANDARD ) : {

                    /* analyse token */
                    if ( std::strcmp( ( char * ) dl_token, "end_header" ) == 0 ) {

                        /* input stream data offset */
                        hd_position = hd_stream.tellg();

                        /* update mode */
                        dl_mode = LC_PLY_VALIDATE;

                    } else if ( std::strcmp( ( char * ) dl_token, "format" ) == 0 ) {

                        /* update mode */
                        dl_mode = LC_PLY_FORMAT;

                    } else if ( std::strcmp( ( char * ) dl_token, "element" ) == 0 ) {

                        /* update mode */
                        dl_mode = LC_PLY_ELEMENT;

                    } else if ( std::strcmp( ( char * ) dl_token, "property" ) == 0 ) {

                        /* check sub-mode */
                        if ( dl_subm == LC_PLY_VERTEX ) {

                            /* update mode */
                            dl_mode = LC_PLY_PROPERTY;

                        }

                    }

                } break;

                /* mode : format reading */
                case ( LC_PLY_FORMAT ) : {

                    /* analyse token */
                    if ( std::strcmp( ( char * ) dl_token, "binary_little_endian" ) != 0 ) {

                        /* send message */
                        throw( LC_ERROR_FORMAT );

                    }

                    /* update mode */
                    dl_mode = LC_PLY_STANDARD;

                } break;

                /* mode : element analysis */
                case ( LC_PLY_ELEMENT ) : {

                    /* analyse token */
                    if ( std::strcmp( ( char * ) dl_token, "vertex" ) == 0 ) {

                        /* update mode */
                        dl_subm = ( dl_mode = LC_PLY_VERTEX );

                    } else {

                        /* update mode */
                        dl_subm = ( dl_mode = LC_PLY_STANDARD );

                    }

                } break;

                /* mode : vertex analysis */
                case ( LC_PLY_VERTEX ) : {

                    /* import vertex count from token */
                    hd_vertex = strtoll( ( char * ) dl_token, NULL, 10 );

                    /* update mode */
                    dl_mode = LC_PLY_STANDARD;

                } break;

                /* mode : property analysis */
                case ( LC_PLY_PROPERTY ) : {

                    /* detect properties type */
                    if ( ( dl_type = hd_det_type( ( char * ) dl_token ) ) == LC_PLY_NONE ) {

                        /* send message */
                        throw( LC_ERROR_FORMAT );

                    }

                    /* update mode */
                    dl_mode = LC_PLY_DATA;

                } break;

                /* mode : data analysis */
                case ( LC_PLY_DATA ) : {

                    /* set data offset */
                    hd_set_data( ( char * ) dl_token, hd_size );

                    /* update offset */
                    hd_size += hd_det_size( dl_type );

                    /* update mode */
                    dl_mode = LC_PLY_STANDARD;

                } break;

            };

            /* reset token parser */
            dl_accum = dl_token;

        }

    }

    dl_header_t::~dl_header_t() {

        /* check input stream */
        if ( hd_stream.is_open() == true ) {

            /* delete input stream */
            hd_stream.close();

        }

        /* check buffer memory */
        if ( hd_chunk != nullptr ) {

            /* release buffer memory */
            delete [] hd_chunk;

        }

    }

/*
    source - detection methods
 */

    int dl_header_t::hd_det_type( char const * const dl_token ) {

        /* type detection */
        if ( std::strcmp( dl_token, "double" ) == 0 ) {

            /* return type */
            return( LC_PLY_DOUBLE );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "float64" ) == 0 ) {

            /* return type */
            return( LC_PLY_DOUBLE );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "float" ) == 0 ) {

            /* return type */
            return( LC_PLY_FLOAT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "float32" ) == 0 ) {

            /* return type */
            return( LC_PLY_FLOAT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "uchar" ) == 0 ) {

            /* return type */
            return( LC_PLY_UCHAR );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "uint8" ) == 0 ) {

            /* return type */
            return( LC_PLY_UCHAR );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "char" ) == 0 ) {

            /* return type */
            return( LC_PLY_CHAR );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "int8" ) == 0 ) {

            /* return type */
            return( LC_PLY_CHAR );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "ushort" ) == 0 ) {

            /* return type */
            return( LC_PLY_USHORT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "uint16" ) == 0 ) {

            /* return type */
            return( LC_PLY_USHORT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "short" ) == 0 ) {

            /* return type */
            return( LC_PLY_SHORT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "int16" ) == 0 ) {

            /* return type */
            return( LC_PLY_SHORT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "uint" ) == 0 ) {

            /* return type */
            return( LC_PLY_UINT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "uint32" ) == 0 ) {

            /* return type */
            return( LC_PLY_UINT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "int" ) == 0 ) {

            /* return type */
            return( LC_PLY_INT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "int32" ) == 0 ) {

            /* return type */
            return( LC_PLY_INT );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "ulong" ) == 0 ) {

            /* return type */
            return( LC_PLY_ULONG );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "uint64" ) == 0 ) {

            /* return type */
            return( LC_PLY_ULONG );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "long" ) == 0 ) {

            /* return type */
            return( LC_PLY_LONG );

        }

        /* type detection */
        if ( std::strcmp( dl_token, "int64" ) == 0 ) {

            /* return type */
            return( LC_PLY_LONG );

        }

        /* return invalid type */
        return( LC_PLY_NONE );

    }

    int dl_header_t::hd_det_size( int dl_type ) {

        /* switch on type */
        switch ( dl_type ) {

            /* return type size */
            case LC_PLY_DOUBLE : { return( 8 ); } break;
            case LC_PLY_FLOAT  : { return( 4 ); } break;
            case LC_PLY_UCHAR  : { return( 1 ); } break;
            case LC_PLY_CHAR   : { return( 1 ); } break;
            case LC_PLY_USHORT : { return( 2 ); } break;
            case LC_PLY_SHORT  : { return( 2 ); } break;
            case LC_PLY_UINT   : { return( 4 ); } break;
            case LC_PLY_INT    : { return( 4 ); } break;
            case LC_PLY_ULONG  : { return( 8 ); } break;
            case LC_PLY_LONG   : { return( 8 ); } break;

        };

        /* send message */
        throw( LC_ERROR_FORMAT );

    }

/*
    source - accessor methods
 */

    double dl_header_t::hd_get_x( long long int lc_index ) {

        /* return element value */
        return( * ( float * ) ( hd_chunk + ( lc_index * hd_size ) + hd_data[0] ) );

    }

    double dl_header_t::hd_get_y( long long int lc_index ) {

        /* return element value */
        return( * ( float * ) ( hd_chunk + ( lc_index * hd_size ) + hd_data[1] ) );

    }

    double dl_header_t::hd_get_z( long long int lc_index ) {

        /* return element value */
        return( * ( float * ) ( hd_chunk + ( lc_index * hd_size ) + hd_data[2] ) );

    }

    char dl_header_t::hd_get_red( long long int lc_index ) {

        /* return element value */
        return( * ( char * ) ( hd_chunk + ( lc_index * hd_size ) + hd_data[3] ) );

    }

    char dl_header_t::hd_get_green( long long int lc_index ) {

        /* return element value */
        return( * ( char * ) ( hd_chunk + ( lc_index * hd_size ) + hd_data[4] ) );

    }

    char dl_header_t::hd_get_blue( long long int lc_index ) {

        /* return element value */
        return( * ( char * ) ( hd_chunk + ( lc_index * hd_size ) + hd_data[5] ) );

    }

/*
    source - mutator methods
 */

    void dl_header_t::hd_set_data( char const * const dl_token, long long int const dl_offset ) {

        /* data detection */
        if ( std::strcmp( dl_token, "x" ) == 0 ) {

            /* assign data offset */
            hd_data[0] = dl_offset;

            /* abort detection */
            return;

        }

        /* data detection */
        if ( std::strcmp( dl_token, "y" ) == 0 ) {

            /* assign data offset */
            hd_data[1] = dl_offset;

            /* abort detection */
            return;

        }

        /* data detection */
        if ( std::strcmp( dl_token, "z" ) == 0 ) {

            /* assign data offset */
            hd_data[2] = dl_offset;

            /* abort detection */
            return;

        }

        /* data detection */
        if ( std::strcmp( dl_token, "red" ) == 0 ) {

            /* assign data offset */
            hd_data[3] = dl_offset;

            /* abort detection */
            return;

        }

        /* data detection */
        if ( std::strcmp( dl_token, "green" ) == 0 ) {

            /* assign data offset */
            hd_data[4] = dl_offset;

            /* abort detection */
            return;

        }

        /* data detection */
        if ( std::strcmp( dl_token, "blue" ) == 0 ) {

            /* assign data offset */
            hd_data[5] = dl_offset;

            /* abort detection */
            return;

        }

    }

    long long int dl_header_t::hd_set_chunk( long long int const dl_size ) {

        /* check buffer memory */
        if ( hd_chunk != nullptr ) {

            /* release buffer memory */
            delete [] hd_chunk;

        }

        /* allocate and check buffer memory */
        if ( ( hd_chunk = new ( std::nothrow ) char[dl_size * hd_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* stream offset management */
        hd_stream.seekg( hd_position, std::ios_base::beg );

        /* read chunk from file */
        hd_stream.read( hd_chunk, dl_size * hd_size );

        /* update stream offset */
        hd_position += hd_stream.gcount();

        /* chunk record count */
        return( hd_stream.gcount() / hd_size );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream variable */
        std::ofstream dl_ostream;

        /* chunk variable */
        long long int dl_size( 0 );

        /* conversion variable */
        long long int dl_index( 0 );

        /* buffer variable */
        char * dl_buffer( nullptr );

        /* buffer mapping variable */
        lc_uf3p_t * dl_pose( nullptr );
        lc_uf3d_t * dl_data( nullptr );

    /* error management */
    try {

        /* stream variable */
        dl_header_t dl_istream( lc_read_string( argc, argv, "--ply", "-i" ) );

        /* allocate and check buffer memory */
        if ( ( dl_buffer = new ( std::nothrow ) char[LC_UF3_RECLEN * LC_UF3_CHUNK] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--uf3", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* parsing input stream chunks */
        while ( ( dl_size = dl_istream.hd_set_chunk( LC_UF3_CHUNK ) ) > 0 ) {

            /* initialise counter */
            dl_index = 0;

            /* parsing input chunk elements */
            for ( long long int dl_parse( 0 ); dl_parse < dl_size; dl_parse ++ ) {

                /* compose buffer mapping */
                dl_pose = ( lc_uf3p_t * ) ( dl_buffer + dl_index );
                dl_data = ( lc_uf3d_t * ) ( dl_pose + 3 );

                /* convert chunk elements */
                dl_pose[0] = dl_istream.hd_get_x( dl_parse );
                dl_pose[1] = dl_istream.hd_get_y( dl_parse );
                dl_pose[2] = dl_istream.hd_get_z( dl_parse );

                /* vertex filtering - nan removal */
                if ( dl_pose[0] != dl_pose[0] ) continue;
                if ( dl_pose[1] != dl_pose[1] ) continue;
                if ( dl_pose[2] != dl_pose[2] ) continue;

                /* convert chunk elements */
                dl_data[0] = dl_istream.hd_get_red  ( dl_parse );
                dl_data[1] = dl_istream.hd_get_green( dl_parse );
                dl_data[2] = dl_istream.hd_get_blue ( dl_parse );

                /* update counter */
                dl_index += LC_UF3_RECLEN;

            }

            /* export converted chunk */
            dl_ostream.write( dl_buffer, dl_index );

        }

        /* delete output stream */
        dl_ostream.close();

        /* release buffer memory */
        delete [] dl_buffer;

    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

