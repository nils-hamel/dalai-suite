/*
 *  dalai-suite - color
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

        /* thread count variables */
        int dl_thread( lc_read_signed( argc, argv, "--thread", "-t", 1 ) );

        /* color mapping variables */
        double dl_ledge( lc_read_double( argc, argv, "--minimum", "-m",  0.0 ) );
        double dl_hedge( lc_read_double( argc, argv, "--maximum", "-x", 50.0 ) );

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* stream buffer variables */
        char * dl_buffer( nullptr );
        char * dl_cbound( nullptr );

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access input stream" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access output stream" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* allocate and check memory */
        if ( ( dl_buffer = new ( std::nothrow ) char [LC_UF3_CHUNK * LC_UF3_RECLEN] ) == nullptr ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to allocate memory" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        do {

            /* read input stream */
            dl_istream.read( dl_buffer, LC_UF3_CHUNK * LC_UF3_RECLEN );

            /* compute buffer boundary */
            dl_cbound = dl_buffer + dl_istream.gcount();

            # ifdef __OPENMP__
            # pragma omp parallel for firstprivate(dl_buffer,dl_cbound,dl_ledge,dl_hedge) num_threads( dl_thread )
            # endif

            /* parsing stream buffer elements */
            for ( char * dl_parse = dl_buffer; dl_parse < dl_cbound; dl_parse += LC_UF3_RECLEN ) {

                /* compute element color */
                dl_color( ( ( lc_uf3p_t * ) dl_parse )[2], ( lc_uf3d_t * ) ( dl_parse + LC_UF3_DATA ), dl_ledge, dl_hedge );

            }

            /* export processed chunk */
            dl_ostream.write( dl_buffer, dl_istream.gcount() );

        /* reading stream chunks */
        } while ( dl_istream.gcount() > 0 );

        /* release buffer memory */
        delete[] dl_buffer;

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

        /* send message */
        return( EXIT_SUCCESS );

    }

