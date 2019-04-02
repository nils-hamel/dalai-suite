/*
 *  dalai-suite - convert
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

    # include "dalai-convert.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* conversion string variable */
        le_char_t * dl_fconv( ( le_char_t * ) lc_read_string( argc, argv, "--from", "-f" ) );

        /* conversion string variable */
        le_char_t * dl_tconv( ( le_char_t * ) lc_read_string( argc, argv, "--to", "-t" ) );

        /* geoid path variable */
        le_char_t * dl_geoid( ( le_char_t * ) lc_read_string( argc, argv, "--geoid", "-g" ) );

        /* reading variable */
        le_size_t dl_read( 1 );

        /* buffer variable */
        le_byte_t * dl_buffer( nullptr );

        /* stream variable */
        std::ifstream dl_istream;

        /* stream variable */
        std::ofstream dl_ostream;

    /* error management */
    try {

        /* conversion structure variable */
        pm_convert_t dl_convert = pm_convert_create( ( char * ) dl_fconv, ( char * ) dl_tconv, ( char * ) dl_geoid );

        /* check conversion status */
        if ( dl_convert._status != 0 ) {

            /* send message */
            throw( LC_ERROR_CONVERT );

        }

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::binary );

        /* check stream */
        if ( dl_istream.is_open() != true ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

        /* check stream */
        if ( dl_ostream.is_open() != true ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* allocate and check buffer memory */
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * LE_ARRAY_DATA] ) == nullptr ) {

            /* send message */
            return( LC_ERROR_MEMORY );

        }

        /* read input stream by chunk */
        do {

            /* read stream chunk */
            dl_istream.read( ( char * ) dl_buffer, LE_UV3_CHUNK * LE_ARRAY_DATA );

            /* check read chunk size */
            if ( ( dl_read = dl_istream.gcount() ) > 0 ) {

                /* convert chunk */
                pm_convert_cvt( & dl_convert, ( char * ) dl_buffer, dl_read, LE_ARRAY_DATA );

                /* export chunk in output stream */
                dl_ostream.write( ( char * ) dl_buffer, dl_read );

            }

        } while ( dl_read > 0 );

        /* release buffer memory */
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

