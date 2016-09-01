/*
 *  dalai-suite - geodetic system
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016 EPFL CDH DHLAB
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

        /* Clamp height value */
        dl_height = dl_height < dl_ledge ? dl_ledge : dl_height;
        dl_height = dl_height > dl_hedge ? dl_hedge : dl_height;

        /* Normalise height value */
        dl_height = ( ( ( dl_height - dl_ledge ) / ( dl_hedge - dl_ledge ) ) * 0.7 + 0.3 ) * 3.14;

        /* Compute element color */
        dl_data[0] = 127.0 + 128.0 * sin( dl_height );
        dl_data[1] = 127.0 + 128.0 * cos( dl_height );
        dl_data[2] = 127.0 - 128.0 * sin( dl_height );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* Status variables */
        int dl_status = EXIT_SUCCESS;

        /* Stream size variables */
        size_t dl_size( 0 );

        /* Stream buffer variables */
        char * dl_buffer( NULL );
        char * dl_bound ( NULL );

        /* Stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* Thread count variables */
        int dl_thread( lc_read_uint( argc, argv, "--thread", "-t", 1 ) );

        /* Color mapping variables */
        double dl_ledge( lc_read_double( argc, argv, "--minimum", "-m",  0.0 ) );
        double dl_hedge( lc_read_double( argc, argv, "--maximum", "-x", 50.0 ) );

        /* Create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::ate | std::ios::in | std::ios::binary );

        /* Check input stream */
        if ( dl_istream.is_open() ) {

            /* Retrieve file size */
            dl_size = dl_istream.tellg();

            /* Allocate and check memory */
            if ( ( dl_buffer = new ( std::nothrow ) char [dl_size] ) != NULL ) {

                /* Stream offset to begining */
                dl_istream.seekg( 0, std::ios::beg );

                /* Read input stream */
                dl_istream.read( dl_buffer, dl_size );

                /* Delete input stream */
                dl_istream.close();

                /* Compute stream buffer edge */
                dl_bound = dl_buffer + dl_size;

            # ifdef __OPENMP__
            # pragma omp parallel firstprivate(dl_buffer, dl_bound) num_threads( dl_thread )
            {
            # pragma omp for
            # endif

                /* Parsing stream buffer elements */
                for ( char * dl_parse = dl_buffer; dl_parse < dl_bound; dl_parse += 27 ) {

                    /* Compute element color */
                    dl_color( ( ( double * ) dl_parse )[2], ( uint8_t * ) ( dl_parse + 24 ), dl_ledge, dl_hedge );

                }

            # ifdef __OPENMP__
            }
            #endif

                /* Create output stream */
                dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

                /* Check output stream */
                if ( dl_ostream.is_open() ) {

                    /* Write output stream */
                    dl_ostream.write( dl_buffer, dl_size );

                    /* Delete output stream */
                    dl_ostream.close();                    

                } else {

                    /* Display message */
                    std::cerr << "dalai-suite : error : unable to write output file" << std::endl;

                    /* Push message */
                    dl_status = EXIT_FAILURE;

                }

                /* Unallocate memory */
                delete[] dl_buffer;

            }

        } else {

            /* Display message */
            std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

            /* Push message */
            dl_status = EXIT_FAILURE;

        }

        /* Send message */
        return( dl_status );

    }

