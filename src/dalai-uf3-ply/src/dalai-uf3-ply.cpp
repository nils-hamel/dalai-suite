/*
 *  dalai-suite - uf3-ply
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

    # include "dalai-uf3-ply.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* i/o buffer variables */
        char   * dl_ibuffer( nullptr );
        char   * dl_obuffer( nullptr );

        /* i/o buffer mapping variables */
        double * dl_ipose( nullptr );
        float  * dl_opose( nullptr );
        char   * dl_idata( nullptr );
        char   * dl_odata( nullptr );

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* stream size variables */
        long long int dl_size( 0 );

        /* message variables */
        int dl_message( EXIT_SUCCESS );

        /* allocate and check i/o buffer memory */
        if ( ( dl_ibuffer = new char[DL_UF3_PLY_CHUNK * 27] ) != nullptr ) {

            /* allocate and check i/o buffer memory */
            if ( ( dl_obuffer = new char[DL_UF3_PLY_CHUNK * 15] ) != nullptr ) {

                /* create input stream */
                dl_istream.open( lc_read_string( argc, argv, "--uf3", "-i" ), std::ios::ate | std::ios::in | std::ios::binary );

                /* check input stream */
                if ( dl_istream.is_open() == true ) {

                    /* retrieve input stream size */
                    dl_size = dl_istream.tellg() / 27;

                    /* reset input stream position */
                    dl_istream.seekg( 0, std::ios::beg );

                    /* create output stream */
                    dl_ostream.open( lc_read_string( argc, argv, "--ply", "-o" ), std::ios::out | std::ios::binary );

                    /* check output stream */
                    if ( dl_ostream.is_open() == true ) {

                        /* export ply header */
                        dl_ostream << "ply"                             << std::endl;
                        dl_ostream << "format binary_little_endian 1.0" << std::endl;
                        dl_ostream << "element vertex " << dl_size      << std::endl;
                        dl_ostream << "property float x"                << std::endl;
                        dl_ostream << "property float y"                << std::endl;
                        dl_ostream << "property float z"                << std::endl;
                        dl_ostream << "property uchar red"              << std::endl;
                        dl_ostream << "property uchar green"            << std::endl;
                        dl_ostream << "property uchar blue"             << std::endl;
                        dl_ostream << "end_header"                      << std::endl;

                        /* conversion processing loop */
                        do {

                            /* read input stream chunk */
                            dl_istream.read( dl_ibuffer, DL_UF3_PLY_CHUNK * 27 );

                            /* parsing input stream chunk */
                            for ( long long int dl_iparse( 0 ), dl_oparse( 0 ); dl_iparse < dl_istream.gcount(); dl_iparse += 27, dl_oparse += 15 ) {

                                /* create i/o buffer mapping */
                                dl_ipose = ( double * ) ( dl_ibuffer + dl_iparse );
                                dl_opose = ( float  * ) ( dl_obuffer + dl_oparse );
                                dl_idata = ( char   * ) ( dl_ipose + 3 );
                                dl_odata = ( char   * ) ( dl_opose + 3 );

                                /* input stream vertex filtering */
                                if ( dl_ipose[0] != dl_ipose[0] ) continue;
                                if ( dl_ipose[1] != dl_ipose[1] ) continue;
                                if ( dl_ipose[2] != dl_ipose[2] ) continue;

                                /* compose output stream chunk */
                                dl_opose[0] = dl_ipose[0];
                                dl_opose[1] = dl_ipose[1];
                                dl_opose[2] = dl_ipose[2];
                                dl_odata[0] = dl_idata[0];
                                dl_odata[1] = dl_idata[1];
                                dl_odata[2] = dl_idata[2];

                            }

                            /* write output stream chunk */
                            dl_ostream.write( dl_obuffer, ( dl_istream.gcount() / 27 ) * 15 );

                        /* conversion end condition */
                        } while ( dl_istream.gcount() > 0 );

                        /* delete output stream */
                        dl_ostream.close();

                    } else {

                        /* display message */
                        std::cerr << "dalai-suite : error : unable to access output file" << std::endl;

                        /* push message */
                        dl_message = EXIT_FAILURE;

                    }

                    /* delete input stream */
                    dl_istream.close();

                } else {

                    /* display message */
                    std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

                    /* push message */
                    dl_message = EXIT_FAILURE;

                }

                /* delete memory allocation */
                delete [] dl_obuffer;

            } else {

                /* dislay message */
                std::cerr << "dalai-suite : error : unable to allocate memory" << std::endl;

                /* push message */
                dl_message = EXIT_FAILURE;

            }

            /* delete memory allocation */
            delete [] dl_ibuffer;

        } else {

            /* dislay message */
            std::cerr << "dalai-suite : error : unable to allocate memory" << std::endl;

            /* push message */
            dl_message = EXIT_FAILURE;

        }

        /* send message */
        return( dl_message );

    }

