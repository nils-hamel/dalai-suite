/*
 *  dalai-suite - shift
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2019 DHLAB, EPFL
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

    # include "dalai-shift.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream buffer variables */
        le_byte_t * dl_buffer( nullptr );

        /* shift variable */
        le_real_t dl_xshift( lc_read_double( argc, argv, "--x", "-x", 0.0 ) );
        le_real_t dl_yshift( lc_read_double( argc, argv, "--y", "-y", 0.0 ) );
        le_real_t dl_zshift( lc_read_double( argc, argv, "--z", "-z", 0.0 ) );

        /* reading variable */
        le_size_t dl_read( 1 );

        /* stream variable */
        std::ifstream dl_istream;

        /* stream variable */
        std::ofstream dl_ostream;

    /* error management */
    try {

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() != true ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() != true ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* allocate buffer memory */
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * LE_ARRAY_DATA] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* stream reading */
        while ( dl_read != 0 ) {

            /* read stream chunk */
            dl_istream.read( ( char * ) dl_buffer, LE_UV3_CHUNK * LE_ARRAY_DATA );

            /* check read bytes */
            if ( ( dl_read = dl_istream.gcount() ) > 0 ) {

                /* parsing chunk */
                for ( le_size_t dl_parse( 0 ); dl_parse < dl_read; dl_parse += LE_ARRAY_DATA ) {

                    /* apply shift on coordinates */
                    ( ( le_real_t * ) ( dl_buffer + dl_parse ) )[0] += dl_xshift;
                    ( ( le_real_t * ) ( dl_buffer + dl_parse ) )[1] += dl_yshift;
                    ( ( le_real_t * ) ( dl_buffer + dl_parse ) )[2] += dl_zshift;

                }

                /* write stream chunk */
                dl_ostream.write( ( char * ) dl_buffer, dl_read );

            }

        }

        /* delete stream buffer */
        delete[] dl_buffer;

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

