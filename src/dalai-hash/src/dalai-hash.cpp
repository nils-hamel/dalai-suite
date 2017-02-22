/*
 *  dalai-suite - hash
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2017 EPFL CDH DHLAB
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

        /* sampling count variables */
        int64_t dl_count( lc_read_signed( argc, argv, "--count", "-c", 64 ) );

        /* hashing parameter variables */
        double dl_param( lc_read_double( argc, argv, "--param", "-p", 250.0 ) );

        /* model mean value variables */
        double dl_mean( 0.0 );

        /* input stream variables */
        std::ifstream dl_istream;

    /* error management */
    try {

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--uf3", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* compute model mean value */
        dl_mean = lc_statistic_mdmv( dl_istream, dl_count, LC_UF3_CHUNK );

        /* hash model */
        lc_hash( dl_istream, lc_read_string( argc, argv, "--output", "-o" ), dl_param, dl_mean, LC_UF3_CHUNK );

        /* delete input stream */
        dl_istream.close();

    /* error management */
    } catch ( int dl_code ) {

        /* switch on error code */
        switch ( dl_code ) {

            case ( LC_ERROR_MEMORY ) : {

                /* display message */
                std::cerr << "dalai-suite : error : memory allocation" << std::endl;

            } break;

            case ( LC_ERROR_IO_ACCESS ) : {

                /* display message */
                std::cerr << "dalai-suite : error : stream access" << std::endl;

            } break;

            case ( LC_ERROR_IO_READ ) : {

                /* display message */
                std::cerr << "dalai-suite : error : stream reading" << std::endl;

            } break;

        };

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

