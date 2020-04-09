/*
 *  dalai-suite - cat
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
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

    # include "dalai-cat.hpp"

/*
    source - address methods
 */

    le_void_t dl_cat_address( le_real_t * const dl_pose, le_byte_t const dl_length ) {

        /* address structure variable */
        le_address_t dl_address = LE_ADDRESS_C_SIZE( dl_length );

        /* address string variable */
        le_char_t dl_string[_LE_USE_DEPTH + 1] = { 0 };

        /* convert position to address */
        le_address_set_pose( & dl_address, dl_pose );

        /* compose address string */
        for ( le_size_t dl_parse( 0 ); dl_parse < dl_length; dl_parse ++ ) {

            /* digit to char conversion */
            dl_string[dl_parse] = le_address_get_digit( & dl_address, dl_parse ) + 48;

        }

        /* add termination character */
        dl_string[dl_length] = 0;

        /* display address string */
        printf( "%s ", dl_string );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream buffer variable */
        le_byte_t dl_buffer[LE_ARRAY_DATA];

        /* buffer pointer variable */
        le_real_t * dl_uv3p( nullptr );

        /* buffer pointer variable */
        le_data_t * dl_uv3d( nullptr );

        /* display format variable */
        le_byte_t dl_index( lc_read_unsigned( argc, argv, "--index", "-x", 0 ) );

        /* reading variable */
        le_size_t dl_read( 1 );

        /* stream variables */
        std::ifstream dl_stream;

    /* error management */
    try {

        /* create input stream */
        dl_stream.open( lc_read_string( argc, argv, "--uv3", "-i" ), std::ios::in | std::ios::binary );

        /* check stream */
        if ( dl_stream.is_open() != true ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* compute buffer pointer */
        dl_uv3p = ( le_real_t * ) ( dl_buffer );

        /* compute buffer pointer */
        dl_uv3d = ( le_data_t * ) ( dl_uv3p + 3 );

        /* stream reading */
        while ( dl_read != 0 ) {

            /* read stream record */
            dl_stream.read( ( char * ) dl_buffer, LE_ARRAY_DATA );

            /* check read record */
            if ( ( dl_read = dl_stream.gcount() ) == LE_ARRAY_DATA ) {

                /* check display format */
                if ( dl_index == 0 ) {

                    /* display geographic coordinates */
                    printf( "%+e %+e %+e ", dl_uv3p[0], dl_uv3p[1], dl_uv3p[2] );

                } else {

                    /* display geographic index */
                    dl_cat_address( dl_uv3p, dl_index );

                }

                /* display record data */
                printf( "%02x %02x %02x %02x\n", dl_uv3d[0], dl_uv3d[1], dl_uv3d[2], dl_uv3d[3] );

            }

        }

        /* delete stream */
        dl_stream.close();

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

