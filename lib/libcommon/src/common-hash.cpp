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

    void lc_hash( std::ifstream & lc_istream, char const * const lc_opath, double const lc_param, double const lc_mean ) {

        /* hashing parameter variables */
        double lc_segment( lc_param * lc_mean );

        /* hashing index variables */
        int64_t lc_xhash( 0 );
        int64_t lc_yhash( 0 );
        int64_t lc_zhash( 0 );

        /* stream path variables */
        char lc_hpath[256];

        /* stream variables */
        std::ofstream lc_ostream;

        /* buffer variables */
        char * lc_buffer( nullptr );

        /* allocate and check buffer memory */
        if ( ( lc_buffer = new ( std::nothrow ) char[LE_UV3_CHUNK * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* clear input stream */
        lc_istream.clear();

        /* set input stream offset */
        lc_istream.seekg( 0, std::ios::beg );

        /* parsing input stream chunks */
        do {

            /* read input stream chunk */
            lc_istream.read( lc_buffer, LE_UV3_CHUNK * LE_UV3_RECORD );

            /* parsing input stream chunk */
            for ( char * lc_parse( lc_buffer ), * lc_limit( lc_buffer + lc_istream.gcount() ); lc_parse < lc_limit; lc_parse += LE_UV3_RECORD ) {

                /* check primitive type */
                if ( * ( lc_parse + LE_UV3_POSE ) == LE_UV3_POINT ) {

                    /* compute hash index */
                    lc_xhash = floor( ( ( le_real_t * ) ( lc_parse ) )[0] / lc_segment );
                    lc_yhash = floor( ( ( le_real_t * ) ( lc_parse ) )[1] / lc_segment );
                    lc_zhash = floor( ( ( le_real_t * ) ( lc_parse ) )[2] / lc_segment );

                    /* compute stream path */
                    sprintf( lc_hpath, "%s/%+" PRId64 "_%+" PRId64 "_%+" PRId64 ".uf3", lc_opath, lc_xhash, lc_yhash, lc_zhash );

                    /* create output stream */
                    lc_ostream.open( lc_hpath, std::ios::app | std::ios::out | std::ios::binary );

                    /* check output stream */
                    if ( lc_ostream.is_open() == false ) {

                        /* send message */
                        throw( LC_ERROR_IO_ACCESS );

                    }

                    /* export chunk element */
                    lc_ostream.write( lc_parse, LE_UV3_RECORD );

                    /* delete output stream */
                    lc_ostream.close();

                }

            }

        /* parsing input stream chunks */
        } while ( lc_istream.gcount() > 0 );

        /* release buffer memory */
        delete [] lc_buffer;

    }

