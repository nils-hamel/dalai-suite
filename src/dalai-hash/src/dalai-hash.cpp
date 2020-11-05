/*
 *  dalai-suite - hash
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

    # include "dalai-hash.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* sampling variable */
        le_size_t dl_count( lc_read_signed( argc, argv, "--count", "-c", 64 ) );

        /* hashing variable */
        le_real_t dl_param( lc_read_double( argc, argv, "--parameter", "-p", 250.0 ) );

        /* minimum distance mean variable */
        le_real_t dl_mean( 0.0 );

        /* input variable */
        std::ifstream dl_istream;

    /* error management */
    try {

        /* create stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::binary );

        /* check stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* compute mean value */
        dl_mean = lc_statistic_mdmv( dl_istream, dl_count );

        /* model hashing */
        lc_hash( dl_istream, ( le_char_t * ) lc_read_string( argc, argv, "--output", "-o" ), dl_param, dl_mean );

        /* delete stream */
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

