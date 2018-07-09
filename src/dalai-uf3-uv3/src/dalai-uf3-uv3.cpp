/*
 *  dalai-suite - uf3-uv3
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

    # include "dalai-uf3-uv3.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream variable */
        std::fstream dl_istream;
        std::fstream dl_ostream;

        /* buffer variable */
        le_byte_t * dl_ibuffer( nullptr );
        le_byte_t * dl_obuffer( nullptr );

        /* reading variable */
        le_size_t dl_read( 0 );

    /* error management */
    try {

        /* allocate buffer memory */
        if ( ( dl_ibuffer = new ( std::nothrow ) le_byte_t[DL_CHUNK * DL_UF3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( dl_obuffer = new ( std::nothrow ) le_byte_t[DL_CHUNK * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* create stream */
        dl_istream.open( lc_read_string( argc, argv, "--uf3", "-i" ), std::ios::in | std::ios::binary );

        /* check stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* create stream */
        dl_ostream.open( lc_read_string( argc, argv, "--uv3", "-o" ), std::ios::out | std::ios::binary );

        /* check stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* reading loop */
        do {

            /* read stream chunk */
            dl_istream.read( ( char * ) dl_ibuffer, DL_CHUNK * LE_UV3_RECORD );

            /* check stream reading */
            if ( ( dl_read = dl_istream.gcount() ) > 0 ) {

                /* record conversion */
                for ( le_size_t dl_parse( 0 ), dl_index( 0 ); dl_parse < dl_read; dl_parse += DL_UF3_RECORD, dl_index += LE_UV3_RECORD ) {

                    /* convert record */
                    std::memcpy( dl_obuffer + dl_index, dl_ibuffer + dl_parse, LE_UV3_POSE );

                    /* convert record */
                    dl_obuffer[dl_index + LE_UV3_POSE] = LE_UV3_POINT;

                    /* conver record */
                    dl_obuffer[dl_index + LE_UV3_POSE + 1] = dl_ibuffer[dl_parse + DL_UF3_POSE];
                    dl_obuffer[dl_index + LE_UV3_POSE + 2] = dl_ibuffer[dl_parse + DL_UF3_POSE + 1];
                    dl_obuffer[dl_index + LE_UV3_POSE + 3] = dl_ibuffer[dl_parse + DL_UF3_POSE + 2];

                }

                /* export converted records */
                dl_ostream.write( ( char * ) dl_obuffer, ( dl_read / DL_UF3_RECORD ) * LE_UV3_RECORD );

            }

        /* reading loop condition */
        } while ( dl_read > 0 );

        /* delete stream */
        dl_ostream.close();

        /* delete stream */
        dl_istream.close();

        /* release buffer memory */
        delete [] dl_obuffer;

        /* release buffer memory */
        delete [] dl_ibuffer;

    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

