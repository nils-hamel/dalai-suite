/*
 *  dalai-suite - uv3-ply
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

    # include "dalai-uv3-ply.hpp"

/*
    source - header methods
 */

    le_void_t dl_uv3_ply_header( std::fstream & dl_stream, le_size_t const dl_vertex, le_size_t const dl_face ) {

        /* export header */
        dl_stream << "ply" << std::endl;
        dl_stream << "format binary_little_endian 1.0" << std::endl;
        dl_stream << "element vertex " << dl_vertex << std::endl;
        dl_stream << "property float x" << std::endl;
        dl_stream << "property float y" << std::endl;
        dl_stream << "property float z" << std::endl;
        dl_stream << "property uchar red" << std::endl;
        dl_stream << "property uchar green" << std::endl;
        dl_stream << "property uchar blue" << std::endl;

        /* check primitive */
        if ( dl_face != 0 ) {

            /* export header */
            dl_stream << "element face " << dl_face << std::endl;
            dl_stream << "property list uchar int vertex_indices" << std::endl;

        }

        /* export header */
        dl_stream << "end_header" << std::endl;

    }

/*
    source - statistical methods
 */

    le_size_t dl_uv3_ply_primitive( std::fstream & dl_stream ) {

        /* buffer variable */
        le_byte_t * dl_buffer( nullptr );

        /* reading variable */
        le_size_t dl_read( 1 );

        /* count variable */
        le_size_t dl_lcount( 0 );

        /* count variable */
        le_size_t dl_tcount( 0 );

        /* allocate buffer memory */
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* clear stream state */
        dl_stream.clear();

        /* begining of stream */
        dl_stream.seekg( 0, std::ios::beg );

        /* primitive analysis */
        while ( dl_read != 0 ) {

            /* read stream chunk */
            dl_stream.read( ( char * ) dl_buffer, LE_UV3_CHUNK * LE_UV3_RECORD );

            /* read byte count */
            dl_read = dl_stream.gcount();

            /* parsing read chunk */
            for ( le_size_t dl_parse( LE_UV3_POSE ); dl_parse < dl_read; dl_parse += LE_UV3_RECORD ) {

                /* check primitive type */
                if ( dl_buffer[dl_parse] == LE_UV3_LINE ) {

                    /* update primitive count */
                    dl_lcount ++;

                } else if ( dl_buffer[dl_parse] == LE_UV3_TRIANGLE ) {

                    /* update primitive count */
                    dl_tcount ++;

                }

            }

        }

        /* release buffer memory */
        delete [] dl_buffer;

        /* return primitive count */
        return( ( dl_lcount / 2 ) + ( dl_tcount / 3 ) );

    }

/*
    source - conversion methods
 */

    le_void_t dl_uv3_ply_vertex( std::fstream & dl_istream, std::fstream & dl_ostream ) {

        /* buffer variable */
        le_byte_t * dl_ibuffer( nullptr );

        /* buffer variable */
        le_byte_t * dl_obuffer( nullptr );

        /* reading variable */
        le_size_t dl_read( 1 );

        /* pointer variable */
        le_real_t * dl_uv3p( nullptr );
        le_data_t * dl_uv3d( nullptr );

        /* pointer variable */
        float   * dl_plyp( nullptr );
        uint8_t * dl_plyd( nullptr );

        /* allocate buffer memory */
        if ( ( dl_ibuffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( dl_obuffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * DL_PLY_VERTEX] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* clear stream state */
        dl_istream.clear();

        /* stream offset */
        dl_istream.seekg( 0, std::ios::beg );

        /* parse stream */
        while ( dl_read != 0 ) {

            /* read stream chunk */
            dl_istream.read( ( char * ) dl_ibuffer, LE_UV3_CHUNK * LE_UV3_RECORD );

            /* read byte count */
            dl_read = dl_istream.gcount();

            /* parsing read chunk */
            for ( le_size_t dl_parse( 0 ), dl_index( 0 ); dl_parse < dl_read; dl_parse += LE_UV3_RECORD, dl_index += DL_PLY_VERTEX ) {

                /* compute buffer pointer */
                dl_uv3p = ( le_real_t * ) ( dl_ibuffer + dl_parse );

                /* compute buffer pointer */
                dl_uv3d = ( le_data_t * ) ( dl_uv3p + 3 );

                /* compute buffer pointer */
                dl_plyp = ( float * ) ( dl_obuffer + dl_index );

                /* compute buffer pointer */
                dl_plyd = ( uint8_t * ) ( dl_plyp + 3 );

                /* convert vertex coordinates */
                dl_plyp[0] = dl_uv3p[0];
                dl_plyp[1] = dl_uv3p[1];
                dl_plyp[2] = dl_uv3p[2];

                /* convert color components */
                dl_plyd[0] = dl_uv3d[1];
                dl_plyd[1] = dl_uv3d[2];
                dl_plyd[2] = dl_uv3d[3];

            }

            /* export buffer */
            dl_ostream.write( ( char * ) dl_obuffer, ( dl_read / LE_UV3_RECORD ) * DL_PLY_VERTEX );

        }

        /* release buffer memory */
        delete [] dl_obuffer;

        /* release buffer memory */
        delete [] dl_ibuffer;

    }

    le_void_t dl_uv3_ply_face( std::fstream & dl_istream, std::fstream & dl_ostream ) {

        /* buffer variable */
        le_byte_t * dl_ibuffer( nullptr );

        /* buffer variable */
        le_byte_t dl_lbuffer[DL_PLY_LINE];

        /* buffer variable */
        le_byte_t dl_tbuffer[DL_PLY_FACE];

        /* primitive module variable */
        le_size_t dl_lmodule( 0 );
        le_size_t dl_tmodule( 0 );

        /* reading variable */
        le_size_t dl_read( 1 );

        /* offset variable */
        le_size_t dl_offset( 0 );

        /* buffer pointer variable */
        le_data_t * dl_uv3d( nullptr );

        /* allocate buffer memory */
        if ( ( dl_ibuffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* clear stream state */
        dl_istream.clear();

        /* stream offset */
        dl_istream.seekg( 0, std::ios::beg );

        /* initialise primitive buffer */
        ( * dl_lbuffer ) = 2;
        ( * dl_tbuffer ) = 3;

        /* parse input stream */
        while ( dl_read != 0 ) {

            /* read stream chunk */
            dl_istream.read( ( char * ) dl_ibuffer, LE_UV3_CHUNK * LE_UV3_RECORD );

            /* retrieve byte count */
            dl_read = dl_istream.gcount();

            /* parsing stream chunk */
            for ( le_size_t dl_parse( 0 ); dl_parse < dl_read; dl_parse += LE_UV3_RECORD ) {

                /* compute buffer pointer */
                dl_uv3d = ( le_data_t * ) ( dl_ibuffer + dl_parse + LE_UV3_POSE );

                /* check primitive type */
                if ( ( * dl_uv3d ) == LE_UV3_LINE ) {

                    /* assign vertex index */
                    ( ( int32_t * ) ( dl_lbuffer + 1 ) )[dl_lmodule] = dl_offset + ( dl_parse / LE_UV3_RECORD );

                    /* update primitive module */
                    if ( ( ++ dl_lmodule ) == 2 ) {

                        /* reset module */
                        dl_lmodule = 0;
    
                        /* export buffer */
                        dl_ostream.write( ( char * ) dl_lbuffer, DL_PLY_LINE );

                    }

                } else if ( ( * dl_uv3d ) == LE_UV3_TRIANGLE ) {

                    /* assign vertex index */
                    ( ( int32_t * ) ( dl_tbuffer + 1 ) )[dl_tmodule] = dl_offset + ( dl_parse / LE_UV3_RECORD );

                    /* update primitive module */
                    if ( ( ++ dl_tmodule ) == 3 ) {

                        /* reset module */
                        dl_tmodule = 0;

                        /* export buffer */
                        dl_ostream.write( ( char * ) dl_tbuffer, DL_PLY_FACE );

                    }

                }

            }

            /* update offset */
            dl_offset += ( dl_read / LE_UV3_RECORD );

        }

        /* release buffer memory */
        delete [] dl_ibuffer;

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream variable */
        std::fstream dl_istream;

        /* stream variable */
        std::fstream dl_ostream;

        /* primitive variable */
        le_size_t dl_vertex( 0 );

        /* primitive variable */
        le_size_t dl_face( 0 );

    /* error management */
    try {

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--uv3", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* check consistency */
        if ( ( dl_vertex = ( dl_istream.tellg() / LE_UV3_RECORD ) ) == 0 ) {

            /* send message */
            throw( LC_ERROR_FORMAT );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--ply", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* conversion mode */
        if ( lc_read_flag( argc, argv, "--vertex-only", "-v" ) == false ) {

            /* compute primitive count */
            dl_face = dl_uv3_ply_primitive( dl_istream );

            /* export ply header */
            dl_uv3_ply_header( dl_ostream, dl_vertex, dl_face );

            /* export primitive */
            dl_uv3_ply_vertex( dl_istream, dl_ostream );

            /* check primitive count */
            if ( dl_face > 0 ) {

                /* export primitive */
                dl_uv3_ply_face( dl_istream, dl_ostream );

            }

        } else {

            /* export ply header */
            dl_uv3_ply_header( dl_ostream, dl_vertex, 0 );

            /* export primitive */
            dl_uv3_ply_vertex( dl_istream, dl_ostream );

        }

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

    /* error management */
    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );            

    }

