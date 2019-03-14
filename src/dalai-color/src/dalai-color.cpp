/*
 *  dalai-suite - color
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

    # include "dalai-color.hpp"

/*
    source - color mapping methods
 */

    le_void_t dl_color( le_real_t dl_height, le_data_t * const dl_data, le_real_t const dl_ledge, le_real_t const dl_hedge ) {

        /* height pre-normalisation */
        dl_height = ( dl_height - dl_ledge ) / ( dl_hedge - dl_ledge );

        /* height periodic range clamping */
        dl_height = dl_height - floor( dl_height );

        /* height range miroring */
        dl_height = ( dl_height > 0.5 ) ? 1.0 - dl_height : dl_height;

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

        /* colormap boundary variable */
        le_real_t dl_ledge( lc_read_double( argc, argv, "--minimum", "-m",  0.0 ) );

        /* colormap boundary variable */
        le_real_t dl_hedge( lc_read_double( argc, argv, "--maximum", "-x", 50.0 ) );

        /* reading variable */
        le_size_t dl_read( 1 );

        /* stream buffer variable */
        le_byte_t * dl_buffer( nullptr );

        /* buffer pointer variable */
        le_real_t * dl_uv3p( nullptr );

        /* buffer pointer variable */
        le_data_t * dl_uv3d( nullptr );

        /* stream variable */
        std::ifstream dl_istream;

        /* stream variable */
        std::ofstream dl_ostream;

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
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * LE_ARRAY_DATA] ) == nullptr ) {

            /* send message */
            return( LC_ERROR_MEMORY );

        }

        /* stream reading */
        while ( dl_read != 0 ) {

            /* read stream chunk */
            dl_istream.read( ( char * ) dl_buffer, LE_UV3_CHUNK * LE_ARRAY_DATA );

            /* check read bytes */
            if ( ( dl_read = dl_istream.gcount() ) != 0 ) {

                /* parsing stream chunk */
                for ( le_size_t dl_parse( 0 ); dl_parse < dl_read; dl_parse += LE_ARRAY_DATA ) {

                    /* compute buffer pointer */
                    dl_uv3p = ( le_real_t * ) ( dl_buffer + dl_parse );

                    /* compute buffer pointer */
                    dl_uv3d = ( le_data_t * ) ( dl_uv3p + 3 );

                    /* assign element color */
                    dl_color( dl_uv3p[2], dl_uv3d + 1, dl_ledge, dl_hedge );

                }

                /* write stream chunk */
                dl_ostream.write( ( char * ) dl_buffer, dl_read );

            }

        }

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

