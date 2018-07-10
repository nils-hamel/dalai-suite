/*
 *  dalai-suite - cat
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

    # include "dalai-cat.hpp"

/*
    source - address methods
 */

    void dl_cat_address( le_address_t const * const dl_addr ) {

        /* string variable */
        char dl_string[_LE_USE_DEPTH + 1] =  { 0 };

        /* address size variable */
        int dl_size = le_address_get_size( dl_addr );

        /* convert address to string */
        for ( int dl_parse( 0 ); dl_parse < dl_size; dl_parse ++ ) {

            /* assign digit */
            dl_string[dl_parse] = le_address_get_digit( dl_addr, dl_parse ) + 48;

        }

        /* display address index */
        printf( "%s ", dl_string );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream record pointer variable */
        le_real_t * dl_pose( nullptr );

        /* stream record pointer variable */
        le_byte_t * dl_type( nullptr );

        /* stream record pointer variable */
        le_data_t * dl_data( nullptr );

        /* stream buffer variable */
        le_byte_t dl_buffer[LE_UV3_RECORD];

        /* format switch and parameter variable */
        le_byte_t dl_index( lc_read_unsigned( argc, argv, "--index", "-x", 0 ) );

        /* address variable */
        le_address_t dl_address = LE_ADDRESS_C_SIZE( dl_index );

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

        /* create record pointer */
        dl_pose = ( le_real_t * ) ( dl_buffer );

        /* create record pointer */
        dl_type = ( le_byte_t * ) ( dl_buffer + LE_UV3_POSE );

        /* create record pointers */
        dl_data = ( le_data_t * ) ( dl_buffer + LE_UV3_POSE + LE_UV3_TYPE );

        /* stream reading loop */
        do {

            /* read stream chunk */
            dl_stream.read( ( char * ) dl_buffer, LE_UV3_RECORD );

            /* check reading */
            if ( dl_stream.gcount() == LE_UV3_RECORD ) {

                /* check format switch */
                if ( dl_index != 0 ) {

                    /* convert record to address */
                    le_address_set_pose( & dl_address, dl_pose );

                    /* display address index */
                    dl_cat_address( & dl_address );

                } else {

                    /* display position in geographic coordinates */
                    printf( "%e %e %e ", dl_pose[0], dl_pose[1], dl_pose[2] );

                }

                /* display data */
                printf( "%02x %02x %02x %02x\n", dl_type[0], dl_data[0], dl_data[1], dl_data[2] );

            }

        } while ( dl_stream.gcount() > 0 );

        /* delete stream */
        dl_stream.close();

    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

