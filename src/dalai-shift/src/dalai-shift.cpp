/*
 *  dalai-suite - shift
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

    # include "dalai-shift.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* i/o variables */
        long long dl_read( 0 );

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* stream buffer variables */
        char * dl_buffer( nullptr );

        /* buffer pointer variables */
        double * dl_pose( nullptr );

        /* shift constant variables */
        double dl_xshift( lc_read_double( argc, argv, "--x", "-x", 0.0 ) );
        double dl_yshift( lc_read_double( argc, argv, "--y", "-y", 0.0 ) );
        double dl_zshift( lc_read_double( argc, argv, "--z", "-z", 0.0 ) );

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() != true ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access input stream" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        } else {

            /* create output stream */
            dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

            /* check output stream */
            if ( dl_ostream.is_open() != true ) {

                /* display message */
                std::cerr << "dalai-suite : error : unable to access output stream" << std::endl;

                /* send message */
                return( EXIT_FAILURE );

            } else {

                /* allocate memory */
                if ( ( dl_buffer = new ( std::nothrow ) char [LC_UF3_CHUNK * LC_UF3_RECLEN] ) == nullptr ) {

                    /* display message */
                    std::cerr << "dalai-suite : error : unable to allocate memory" << std::endl;

                    /* send message */
                    return( EXIT_FAILURE );

                } else {

                    /* stream reading */
                    do {

                        /* read stream chunk */
                        dl_istream.read( dl_buffer, LC_UF3_CHUNK * LC_UF3_RECLEN );

                        /* check chunk reading */
                        if ( ( dl_read = dl_istream.gcount() ) > 0 ) {

                            /* parsing chunk */
                            for ( long long dl_parse( 0 ); dl_parse < dl_read; dl_parse += LC_UF3_RECLEN ) {

                                /* create buffer pointer */
                                dl_pose = ( double * ) ( dl_buffer + dl_parse );

                                /* apply shift on coordinates */
                                dl_pose[0] += dl_xshift;
                                dl_pose[1] += dl_yshift;
                                dl_pose[2] += dl_zshift;

                            }

                            /* write stream chunk */
                            dl_ostream.write( dl_buffer, dl_read );

                        }

                    } while ( dl_read > 0 );

                    /* delete stream buffer */
                    delete[] dl_buffer;

                }

                /* delete output stream */
                dl_ostream.close();

            }

            /* delete input stream */
            dl_istream.close();

        }

        /* send message */
        return( EXIT_SUCCESS );

    }

