/*
 *  dalai-suite - uf3-uv3
 *
 *      Nils Hamel - nils.hamel@alumni.epfl.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
 *      Copyright (c) 2020 Republic and Canton of Geneva
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

    # include "dalai-uf3-uv3.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* buffer variable */
        le_byte_t * dl_ibuffer( nullptr );

        /* buffer variable */
        le_byte_t * dl_obuffer( nullptr );

        /* parsing variable */
        le_size_t dl_parse( 0 );

        /* parsing variable */
        le_size_t dl_index( 0 );

        /* reading variable */
        le_size_t dl_read( 1 );

        /* stream variable */
        std::fstream dl_istream;

        /* stream variable */
        std::fstream dl_ostream;

    /* error management */
    try {

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* allocate buffer memory */
        if ( ( dl_ibuffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * DL_UF3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( dl_obuffer = new ( std::nothrow ) le_byte_t[LE_UV3_CHUNK * LE_ARRAY_DATA] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* stream conversion */
        while ( dl_read != 0 ) {

            /* read stream chunk */
            dl_istream.read( ( char * ) dl_ibuffer, LE_UV3_CHUNK * DL_UF3_RECORD );

            /* check stream reading */
            if ( ( dl_read = dl_istream.gcount() ) > 0 ) {

                /* reset index */
                dl_parse = 0, dl_index = 0;

                /* parsing stream chunk */
                while ( dl_parse < dl_read ) {

                    /* assign primitive coordinates */
                    std::memcpy( dl_obuffer + dl_index, dl_ibuffer + dl_parse, LE_ARRAY_DATA_POSE );

                    /* update index */
                    dl_parse += DL_UF3_POSE, dl_index += LE_ARRAY_DATA_POSE;

                    /* assign primitive type */
                    ( * ( dl_obuffer + dl_index ) ) = LE_UV3_POINT;

                    /* update index */
                    dl_index += LE_ARRAY_DATA_TYPE;

                    /* assign primitive data */
                    std::memcpy( dl_obuffer + dl_index, dl_ibuffer + dl_parse, LE_ARRAY_DATA_DATA );

                    /* update index */
                    dl_parse += DL_UF3_DATA, dl_index += LE_ARRAY_DATA_DATA;

                }

                /* export converted records */
                dl_ostream.write( ( char * ) dl_obuffer, ( dl_read / DL_UF3_RECORD ) * LE_ARRAY_DATA );

            }

        }

        /* release buffer memory */
        delete [] dl_obuffer;

        /* release buffer memory */
        delete [] dl_ibuffer;

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

