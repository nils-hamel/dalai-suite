/*
 *  dalai-suite - ply-uv3
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

    # include "dalai-ply-uv3.hpp"

/*
    source - constructor/destructor methods
 */

    dl_ply_t::dl_ply_t( le_char_t const * const dl_path )

        : py_vcount( 0 )
        , py_fcount( 0 )
        , py_offset( 0 )
        , py_vsize( 0 )

    {

        /* initialise arrays */
        for ( le_size_t dl_parse( 0 ); dl_parse < 6; dl_parse ++ ) {

            /* initialise size */
            py_vdata[dl_parse] = 0;

            /* initialise type */
            py_vtype[dl_parse] = DL_TYPE_NONE;

            /* asynchronous size condition */
            if ( dl_parse >= 2 ) continue;

            /* initialise size */
            py_fdata[dl_parse] = 0;

            /* initialise type */
            py_ftype[dl_parse] = DL_TYPE_NONE;

        }

        /* create stream */
        py_stream.open( ( char * ) dl_path, std::ios::in | std::ios::binary );

        /* check stream */
        if ( py_stream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* ply-header analysis */
        dl_ply_header();

    }

    dl_ply_t::~dl_ply_t() {

        /* delete stream */
        py_stream.close();

    }

/*
    source - type methods
 */

    le_enum_t dl_ply_t::dl_ply_type_read( std::string & dl_word ) {

        /* switch on litteral type */
        if ( ( dl_word == "float" ) || ( dl_word == "float32" ) ) {

            /* return type */
            return( DL_TYPE_FLOAT );

        } else if ( ( dl_word == "double" ) || ( dl_word == "float64" ) ) {

            /* return type */
            return( DL_TYPE_DOUBLE );

        } else if ( ( dl_word == "char" ) || ( dl_word == "int8" ) ) {

            /* return type */
            return( DL_TYPE_CHAR );

        } else if ( ( dl_word == "uchar" ) || ( dl_word == "uint8" ) ) {

            /* return type */
            return( DL_TYPE_UCHAR );

        } else if ( ( dl_word == "short" ) || ( dl_word == "int16" ) ) {

            /* return type */
            return( DL_TYPE_SHORT );

        } else if ( ( dl_word == "ushort" ) || ( dl_word == "uint16" ) ) {

            /* return type */
            return( DL_TYPE_USHORT );

        } else if ( ( dl_word == "int" ) || ( dl_word == "int32" ) ) {

            /* return type */
            return( DL_TYPE_INT );

        } else if ( ( dl_word == "uint" ) || ( dl_word == "uint32" ) ) {

            /* return type */
            return( DL_TYPE_UINT );

        } else if ( ( dl_word == "long" ) || ( dl_word == "int64" ) ) {

            /* return type */
            return( DL_TYPE_LONG );

        } else {

            /* send message */
            throw( LC_ERROR_FORMAT );

        }

    }

    le_size_t dl_ply_t::dl_ply_type_length( le_enum_t const dl_type ) {

        /* switch on type */
        switch ( dl_type ) {

            /* return type length */
            case ( DL_TYPE_FLOAT  ) : return( 4 );
            case ( DL_TYPE_DOUBLE ) : return( 8 );
            case ( DL_TYPE_CHAR   ) :
            case ( DL_TYPE_UCHAR  ) : return( 1 );
            case ( DL_TYPE_SHORT  ) :
            case ( DL_TYPE_USHORT ) : return( 2 );
            case ( DL_TYPE_INT    ) :
            case ( DL_TYPE_UINT   ) : return( 4 );
            case ( DL_TYPE_LONG   ) : return( 8 );

        };

        /* send message */
        throw( LC_ERROR_FORMAT );

    }

/*
    source - header methods
 */

    le_void_t dl_ply_t::dl_ply_header() {

        /* mode variable */
        le_enum_t dl_mode( DL_MODE_PLY );

        /* primitive variable */
        le_enum_t dl_prim( DL_PRIM_NONE );

        /* type variable */
        le_enum_t dl_type( DL_TYPE_NONE );

        /* type variable */
        le_enum_t dl_list( DL_TYPE_NONE );

        /* flag variable */
        bool dl_flag( true );

        /* token variable */
        std::string dl_word;

        /* header analysis */
        while ( dl_flag == true ) {

            /* read word */
            py_stream >> dl_word;

            /* header mode */
            switch ( dl_mode ) {

                case ( DL_MODE_NONE ) : {

                    /* switch on token */
                    if ( dl_word == "end_header" ) {

                        /* push vertex offset */
                        py_offset = 1 + py_stream.tellg();

                        /* abort header analysis */
                        dl_flag = false;

                    } else if ( dl_word == "format" ) {

                        /* update mode */
                        dl_mode = DL_MODE_FORMAT;

                    } else if ( dl_word == "element" ) {

                        /* update mode */
                        dl_mode = DL_MODE_ELEMENT;

                    } else if ( dl_word == "property" ) {

                        /* update mode */
                        dl_mode = DL_MODE_PROPERTY;

                    }

                } break;

                case ( DL_MODE_PLY ) : {

                    /* check format */
                    if ( dl_word != "ply" ) {

                        /* send message */
                        throw( LC_ERROR_FORMAT );

                    }

                    /* update mode */
                    dl_mode = DL_MODE_NONE;

                } break;

                case ( DL_MODE_FORMAT ) : {

                    /* check format */
                    if ( dl_word != "binary_little_endian" ) {

                        /* send message */
                        throw( LC_ERROR_FORMAT );

                    }

                    /* read version number */
                    py_stream >> dl_word;

                    /* update mode */
                    dl_mode = DL_MODE_NONE;

                } break;

                case ( DL_MODE_ELEMENT ) : {

                    /* switch on token - element */
                    if ( dl_word == "vertex" ) {

                        /* read count */
                        py_stream >> py_vcount;

                        /* update primitve */
                        dl_prim = DL_PRIM_VERTEX;

                    } else if ( dl_word == "face" ) {

                        /* read count */
                        py_stream >> py_fcount;

                        /* update primitve */
                        dl_prim = DL_PRIM_FACE;

                    } else {

                        /* send message */
                        throw( LC_ERROR_FORMAT );

                    }

                    /* update mode */
                    dl_mode = DL_MODE_NONE;

                } break;

                case ( DL_MODE_PROPERTY ) : {

                    /* switch on token - primitive */
                    if ( dl_prim == DL_PRIM_VERTEX ) {

                        /* type interpretation */
                        dl_type = dl_ply_type_read( dl_word );

                        /* read property name */
                        py_stream >> dl_word;

                        /* property interpretation */
                        dl_ply_header_vertex( dl_type, dl_word );

                    } else if ( dl_prim == DL_PRIM_FACE ) {

                        /* check consistency */
                        if ( dl_word != "list" ) {

                            /* send message */
                            throw( LC_ERROR_FORMAT );

                        }

                        /* read count type */
                        py_stream >> dl_word;

                        /* type interpretation */
                        dl_list = dl_ply_type_read( dl_word );

                        /* read list type */
                        py_stream >> dl_word;

                        /* type interpretation */
                        dl_type = dl_ply_type_read( dl_word );

                        /* read list element */
                        py_stream >> dl_word;

                        /* check consistency */
                        if ( dl_word != "vertex_indices" ) {

                            /* send message */
                            throw( LC_ERROR_FORMAT );

                        }

                        /* property interpretation */
                        dl_ply_header_face( dl_list, dl_type );

                    } else {

                        /* send message */
                        throw( LC_ERROR_FORMAT );

                    }

                    /* update mode */
                    dl_mode = DL_MODE_NONE;

                } break;

            };

        }

    }

    le_void_t dl_ply_t::dl_ply_header_vertex( le_enum_t const dl_type, std::string & dl_word ) {

        /* switch on property */
        if ( dl_word == "x" ) {

            /* assign type */
            py_vtype[DL_VERTEX_X] = dl_type;

            /* assign offset */
            py_vdata[DL_VERTEX_X] = py_vsize;

        } else if ( dl_word == "y" ) {

            /* assign type */
            py_vtype[DL_VERTEX_Y] = dl_type;

            /* assign offset */
            py_vdata[DL_VERTEX_Y] = py_vsize;

        } else if ( dl_word == "z" ) {

            /* assign type */
            py_vtype[DL_VERTEX_Z] = dl_type;

            /* assign offset */
            py_vdata[DL_VERTEX_Z] = py_vsize;

        } else if ( dl_word == "red" ) {

            /* assign type */
            py_vtype[DL_VERTEX_R] = dl_type;

            /* assign offset */
            py_vdata[DL_VERTEX_R] = py_vsize;

        } else if ( dl_word == "green" ) {

            /* assign type */
            py_vtype[DL_VERTEX_G] = dl_type;

            /* assign offset */
            py_vdata[DL_VERTEX_G] = py_vsize;

        } else if ( dl_word == "blue" ) {

            /* assign type */
            py_vtype[DL_VERTEX_B] = dl_type;

            /* assign offset */
            py_vdata[DL_VERTEX_B] = py_vsize;

        }

        /* update record size */
        py_vsize += dl_ply_type_length( dl_type );

    }

    le_void_t dl_ply_t::dl_ply_header_face( le_enum_t const dl_list, le_enum_t const dl_type ) {

        /* assign type */
        py_ftype[DL_FACE_LIST] = dl_list;

        /* assign offset */
        py_fdata[DL_FACE_LIST] = 0;

        /* assign type */
        py_ftype[DL_FACE_VERTEX] = dl_type;

        /* assign offset */
        py_fdata[DL_FACE_VERTEX] = dl_ply_type_length( dl_list );

    }

/*
    source - i/o methods
 */

    long long int dl_ply_t::dl_ply_io_integer( le_enum_t const dl_type ) {

        /* switch on type */
        switch ( dl_type ) {

            case ( DL_TYPE_CHAR ) : {

                /* reading variable */
                int8_t dl_read( 0 );

                /* read variable */
                py_stream.read( ( char * ) ( & dl_read ), sizeof( int8_t ) );

                /* return read value */
                return( dl_read );

            } break;

            case ( DL_TYPE_UCHAR ) : {

                /* reading variable */
                uint8_t dl_read( 0 );

                /* read variable */
                py_stream.read( ( char * ) ( & dl_read ), sizeof( uint8_t ) );

                /* return read value */
                return( dl_read );

            } break;

            case ( DL_TYPE_SHORT ) : {

                /* reading variable */
                int16_t dl_read( 0 );

                /* read variable */
                py_stream.read( ( char * ) ( & dl_read ), sizeof( int16_t ) );

                /* return read value */
                return( dl_read );

            } break;

            case ( DL_TYPE_USHORT ) : {

                /* reading variable */
                uint16_t dl_read( 0 );

                /* read variable */
                py_stream.read( ( char * ) ( & dl_read ), sizeof( uint16_t ) );

                /* return read value */
                return( dl_read );

            } break;

            case ( DL_TYPE_INT ) : {

                /* reading variable */
                int32_t dl_read( 0 );

                /* read variable */
                py_stream.read( ( char * ) ( & dl_read ), sizeof( int32_t ) );

                /* return read value */
                return( dl_read );

            } break;

            case ( DL_TYPE_UINT ) : {

                /* reading variable */
                uint32_t dl_read( 0 );

                /* read variable */
                py_stream.read( ( char * ) ( & dl_read ), sizeof( uint32_t ) );

                /* return read value */
                return( dl_read );

            } break;

            case ( DL_TYPE_LONG ) : {

                /* reading variable */
                int64_t dl_read( 0 );

                /* read variable */
                py_stream.read( ( char * ) ( & dl_read ), sizeof( int64_t ) );

                /* return read value */
                return( dl_read );

            } break;


        };

        /* send message */
        throw( LC_ERROR_FORMAT );

    }

    le_void_t dl_ply_t::dl_ply_io_vertex( le_size_t const dl_index, le_byte_t * const dl_buffer ) {

        /* offset variable */
        le_size_t dl_offset( py_stream.tellg() );

        /* update offset */
        py_stream.seekg( py_offset + dl_index * py_vsize, std::ios::beg );

        /* read vertex record */
        py_stream.read( ( char * ) dl_buffer, py_vsize );

        /* update offset */
        py_stream.seekg( dl_offset, std::ios::beg );

    }

/*
    source - convertion methods
 */

    le_void_t dl_ply_t::dl_ply_convert( std::fstream & dl_stream ) {

        /* check primitive count */
        if ( py_fcount == 0 ) {

            /* specific conversion */
            dl_ply_convert_point( dl_stream );

        } else {

            /* specific conversion */
            dl_ply_convert_mesh( dl_stream );

        }

    }

    le_void_t dl_ply_t::dl_ply_convert_point( std::fstream & dl_stream ) {

        /* buffer variable */
        le_byte_t * dl_ibuffer( nullptr );

        /* buffer variable */
        le_byte_t * dl_obuffer( nullptr );

        /* parsing variable */
        le_size_t dl_parse( 0 );

        /* reading variable */
        le_size_t dl_read( 0 );

        /* exportation variable */
        le_size_t dl_export( 0 );

        /* buffer pointer variable */
        le_real_t * dl_uv3p( nullptr );

        /* buffer pointer variable */
        le_data_t * dl_uv3d( nullptr );

        /* allocate buffer memory */
        if ( ( dl_ibuffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * py_vsize] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( dl_obuffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* clear stream */
        py_stream.clear();

        /* offset position */
        py_stream.seekg( py_offset, std::ios::beg );

        /* format conversion */
        while ( dl_parse < ( py_vcount * py_vsize ) ) {

            /* read buffer */
            py_stream.read( ( char * ) dl_ibuffer, LE_UV3_CHUNK * py_vsize );

            /* check buffer */
            if ( ( dl_read = py_stream.gcount() ) > 0 ) {

                /* reset exportation offset */
                dl_export = 0;

                /* chunk conversion */
                for ( le_size_t dl_index( 0 ); dl_index < dl_read; dl_index += py_vsize ) {

                    /* compute buffer pointer */
                    dl_uv3p = ( le_real_t * ) ( dl_obuffer + dl_export );

                    /* compute buffer pointer */
                    dl_uv3d = ( le_data_t * ) ( dl_uv3p + 3 );

                    /* assign position coordinates */
                    dl_uv3p[0] = dl_ply_vertex_float( dl_ibuffer, dl_index, DL_VERTEX_X );
                    dl_uv3p[1] = dl_ply_vertex_float( dl_ibuffer, dl_index, DL_VERTEX_Y );
                    dl_uv3p[2] = dl_ply_vertex_float( dl_ibuffer, dl_index, DL_VERTEX_Z );

                    /* vertex filtering - avoiding nan */
                    if ( dl_uv3p[0] != dl_uv3p[0] ) continue;
                    if ( dl_uv3p[1] != dl_uv3p[1] ) continue;
                    if ( dl_uv3p[2] != dl_uv3p[2] ) continue;

                    /* validate record */
                    dl_export += LE_UV3_RECORD;

                    /* assign primitive type */
                    dl_uv3d[0] = LE_UV3_POINT;

                    /* assign primitive color */
                    dl_uv3d[1] = dl_ply_vertex_integer( dl_ibuffer, dl_index, DL_VERTEX_R );
                    dl_uv3d[2] = dl_ply_vertex_integer( dl_ibuffer, dl_index, DL_VERTEX_G );
                    dl_uv3d[3] = dl_ply_vertex_integer( dl_ibuffer, dl_index, DL_VERTEX_B );

                }

                /* export chunk to stream */
                dl_stream.write( ( char * ) dl_obuffer, dl_export );

            }

            /* update index */
            dl_parse += dl_read;

        }

        /* release buffer memory */
        delete [] dl_obuffer;

        /* release buffer memory */
        delete [] dl_ibuffer;

    }

    le_void_t dl_ply_t::dl_ply_convert_mesh( std::fstream & dl_stream ) {

        /* buffer variable */
        le_byte_t * dl_ibuffer( nullptr );

        /* buffer variable */
        le_byte_t * dl_obuffer( nullptr );

        /* vertex count variable */
        le_size_t dl_vcount( 0 );

        /* vertex index variable */
        le_size_t dl_vindex( 0 );

        /* buffer pointer variable */
        le_real_t * dl_uv3p( nullptr );

        /* buffer pointer variable */
        le_data_t * dl_uv3d( nullptr );

        /* allocate buffer memory */
        if ( ( dl_ibuffer = new ( std::nothrow ) le_byte_t[py_vsize] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( dl_obuffer = new ( std::nothrow ) le_byte_t[LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* create buffer pointer */
        dl_uv3p = ( le_real_t * ) ( dl_obuffer );

        /* create buffer pointer */
        dl_uv3d = ( le_data_t * ) ( dl_uv3p + 3 );

        /* clear stream */
        py_stream.clear();

        /* offset position */
        py_stream.seekg( py_offset + ( py_vcount * py_vsize ), std::ios::beg );

        /* parsing faces */
        for ( le_size_t dl_parse( 0 ); dl_parse < py_fcount; dl_parse ++ ) {

            /* read face vertex count */
            dl_vcount = dl_ply_io_integer( py_ftype[DL_FACE_LIST] );

            /* read face vertex value */
            for ( le_size_t dl_vertex( 0 ); dl_vertex < dl_vcount; dl_vertex ++ ) {

                /* read vertex index */
                dl_vindex = dl_ply_io_integer( py_ftype[DL_FACE_VERTEX] );

                /* read vertex */
                dl_ply_io_vertex( dl_vindex, dl_ibuffer );

                /* assign position coordinates */
                dl_uv3p[0] = dl_ply_vertex_float( dl_ibuffer, 0, DL_VERTEX_X );
                dl_uv3p[1] = dl_ply_vertex_float( dl_ibuffer, 0, DL_VERTEX_Y );
                dl_uv3p[2] = dl_ply_vertex_float( dl_ibuffer, 0, DL_VERTEX_Z );

                /* assign primitive type */
                dl_uv3d[0] = dl_vcount;

                /* assign primitive color */
                dl_uv3d[1] = dl_ply_vertex_integer( dl_ibuffer, 0, DL_VERTEX_R );
                dl_uv3d[2] = dl_ply_vertex_integer( dl_ibuffer, 0, DL_VERTEX_G );
                dl_uv3d[3] = dl_ply_vertex_integer( dl_ibuffer, 0, DL_VERTEX_B );

                /* export record to stream */
                dl_stream.write( ( char * ) dl_obuffer, LE_UV3_RECORD );

            }

        }

        /* release buffer memory */
        delete [] dl_obuffer;

        /* release buffer memory */
        delete [] dl_ibuffer;

    }

    double dl_ply_t::dl_ply_vertex_float( le_byte_t const * const dl_buffer, le_size_t const dl_offset, le_enum_t const dl_vertex ) {

        /* switch on type */
        switch ( py_vtype[dl_vertex] ) {

            case ( DL_TYPE_FLOAT ) : {

                /* retrieve value */
                return( * ( ( float * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_DOUBLE ) : {

                /* retrieve value */
                return( * ( ( double * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_CHAR ) : {

                /* retrieve value */
                return( * ( ( int8_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_UCHAR ) : {

                /* retrieve value */
                return( * ( ( uint8_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_SHORT ) : {

                /* retrieve value */
                return( * ( ( int16_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_USHORT ) : {

                /* retrieve value */
                return( * ( ( uint16_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_INT ) : {

                /* retrieve value */
                return( * ( ( int32_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_UINT ) : {

                /* retrieve value */
                return( * ( ( uint32_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

        };

        /* send message */
        throw( LC_ERROR_FORMAT );

    }

    long long int dl_ply_t::dl_ply_vertex_integer( le_byte_t const * const dl_buffer, le_size_t const dl_offset, le_enum_t const dl_vertex ) {

        /* switch on type */
        switch ( py_vtype[dl_vertex] ) {

            case ( DL_TYPE_CHAR ) : {

                /* retrieve value */
                return( * ( ( int8_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_UCHAR ) : {

                /* retrieve value */
                return( * ( ( uint8_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_SHORT ) : {

                /* retrieve value */
                return( * ( ( int16_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_USHORT ) : {

                /* retrieve value */
                return( * ( ( uint16_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_INT ) : {

                /* retrieve value */
                return( * ( ( int32_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

            case ( DL_TYPE_UINT ) : {

                /* retrieve value */
                return( * ( ( uint32_t * ) ( dl_buffer + dl_offset + py_vdata[dl_vertex] ) ) );

            } break;

        };

        /* send message */
        throw( LC_ERROR_FORMAT );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream variable */
        std::fstream dl_ostream;

    /* error management */
    try {

        /* ply-object variable */
        dl_ply_t dl_istream( ( le_char_t * ) lc_read_string( argc, argv, "--ply", "-i" ) );

        /* create stream */
        dl_ostream.open( lc_read_string( argc, argv, "--uv3", "-o" ), std::ios::out | std::ios::binary );

        /* check stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* ply-object conversion */
        dl_istream.dl_ply_convert( dl_ostream );

        /* delete stream */
        dl_ostream.close();

    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );    

    }

