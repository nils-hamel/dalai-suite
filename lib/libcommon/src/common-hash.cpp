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

    # include "common-hash.hpp"

/*
    source - hashing methods
 */

    le_void_t lc_hash( std::ifstream & lc_istream, le_char_t const * const lc_opath, le_real_t const lc_param, le_real_t const lc_mean ) {

        /* stream variable */
        std::ofstream lc_ostream;

        /* path variable */
        le_char_t lc_file[_LE_USE_PATH];

        /* buffer variable */
        le_byte_t * lc_buffer( nullptr );

        /* hashing variable */
        le_size_t lc_xhash( 0 );
        le_size_t lc_yhash( 0 );
        le_size_t lc_zhash( 0 );

        /* reading variable */
        le_size_t lc_read( 1 );

        /* parameter variable */
        le_real_t lc_segment( lc_param * lc_mean );

        /* buffer pointer variable */
        le_real_t * lc_uv3p( nullptr );


        /* */
        le_size_t lc_stack( LE_UV3_POINT );


        /* allocate buffer memory */
        if ( ( lc_buffer = new ( std::nothrow ) le_byte_t[LE_UV3_RECORD * LE_UV3_CHUNK] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* reset input stream */
        lc_istream.clear();

        /* input stream offset to begining */
        lc_istream.seekg( 0, std::ios::beg );

        /* stream chunk reading */
        while ( lc_read > 0 ) {

            /* read stream chunk */
            lc_istream.read( ( char * ) lc_buffer, LE_UV3_RECORD * LE_UV3_CHUNK );

            /* retrieve read byte count */
            lc_read = lc_istream.gcount();

            /* parsing stream chunk */
            for ( le_size_t lc_parse( 0 ); lc_parse < lc_read; lc_parse += LE_UV3_RECORD ) {

                /* check primitive stack */
                if ( ( -- lc_stack ) == 0 ) {

                    /* compute buffer pointer */
                    lc_uv3p = ( le_real_t * ) ( lc_buffer + lc_parse );

                    /* compute hash index */
                    lc_xhash = floor( lc_uv3p[0] / le_real_t( lc_segment ) );
                    lc_yhash = floor( lc_uv3p[1] / le_real_t( lc_segment ) );
                    lc_zhash = floor( lc_uv3p[2] / le_real_t( lc_segment ) );

                    /* compose stream path */
                    sprintf( ( char * ) lc_file, "%s/%+" _LE_SIZE_P "_%+" _LE_SIZE_P "_%+" _LE_SIZE_P ".uv3", lc_opath, lc_xhash, lc_yhash, lc_zhash );

                    /* update primitive stack */
                    lc_stack = * ( ( le_byte_t * ) ( lc_uv3p + 3 ) );

                }

                /* create output stream */
                lc_ostream.open( ( char * ) lc_file, std::ios::out | std::ios::app | std::ios::binary );

                /* check output stream */
                if ( lc_ostream.is_open() == false ) {

                    /* send message */
                    throw( LC_ERROR_IO_WRITE );

                }

                /* export record to stream */
                lc_ostream.write( ( char * ) ( lc_buffer + lc_parse ), LE_UV3_RECORD );

                /* close output stream */
                lc_ostream.close();

            }

        }

        /* release buffer memory */
        delete [] lc_buffer;

    }

