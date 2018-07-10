/*
 *  dalai-suite - color
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

    # include "dalai-color.hpp"

/*
    source - color mapping methods
 */

    void dl_color( double dl_height, uint8_t * const dl_data, double const dl_ledge, double const dl_hedge ) {

        /* height pre-normalisation */
        dl_height = ( dl_height - dl_ledge ) / ( dl_hedge - dl_ledge );

        /* height periodic range clamping */
        dl_height = dl_height - floor( dl_height );

        /* height range mirroring */
        dl_height = dl_height > 0.5 ? 1.0 - dl_height : dl_height;

        /* height normalisation */
        dl_height = ( 0.3 + 0.7 * dl_height * 2 ) * 3.14;

        /* compute element color */
        dl_data[0] = 127.0 + 128.0 * sin( dl_height );
        dl_data[1] = 127.0 + 128.0 * cos( dl_height );
        dl_data[2] = 127.0 - 128.0 * sin( dl_height );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* color mapping variable */
        double dl_ledge( lc_read_double( argc, argv, "--minimum", "-m",  0.0 ) );
        double dl_hedge( lc_read_double( argc, argv, "--maximum", "-x", 50.0 ) );

        /* stream variable */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* stream buffer variable */
        le_byte_t * dl_buffer( nullptr );

        /* stream buffer variable */
        le_byte_t * dl_bound( nullptr );

    /* error management */
    try {

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            return( LC_ERROR_IO_READ );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            return( LC_ERROR_IO_WRITE );

        }

        /* allocate and check memory */
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[DL_CHUNK * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            return( LC_ERROR_MEMORY );

        }

        do {

            /* read input stream */
            dl_istream.read( ( char * ) dl_buffer, DL_CHUNK * LE_UV3_RECORD );

            /* compute buffer boundary */
            dl_bound = dl_buffer + dl_istream.gcount();

            /* parsing buffer elements */
            for ( le_byte_t * dl_parse = dl_buffer; dl_parse < dl_bound; dl_parse += LE_UV3_RECORD ) {

                /* compute element color */
                dl_color( ( ( le_real_t * ) dl_parse )[2], ( le_data_t * ) ( dl_parse + LE_UV3_POSE + LE_UV3_TYPE ), dl_ledge, dl_hedge );

            }

            /* write output stream */
            dl_ostream.write( ( char * ) dl_buffer, dl_istream.gcount() );

        /* reading stream chunks */
        } while ( dl_istream.gcount() > 0 );

        /* release buffer memory */
        delete[] dl_buffer;

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

