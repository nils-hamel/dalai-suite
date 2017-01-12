/*
 *  dalai-suite - ply-uf3
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

    # include "dalai-ply-uf3.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* output stream variables */
        std::ofstream dl_ostream;

        /* input stream variables */
        lc_ply_t dl_istream = LC_PLY_C;

        /* i/o buffer variables */
        char   * dl_buffer( nullptr );

        /* i/o buffer mapping variables */
        double * dl_pose( nullptr );
        char   * dl_data( nullptr );

        /* parsing variables */
        long long int dl_parse( 0 );
        long long int dl_size ( 0 );

        /* message variables */
        int dl_message( EXIT_SUCCESS );

        /* allocate and check i/o buffer memory */
        if ( ( dl_buffer = new char[DL_PLY_UF3_CHUNK * 27] ) != nullptr ) {

            /* create input stream */
            if ( ( dl_istream = lc_ply_create( ( unsigned char * ) lc_read_string( argc, argv, "--ply", "-i" ) ) )._status == LC_TRUE ) {

                /* create output stream */
                dl_ostream.open( lc_read_string( argc, argv, "--uf3", "-o" ), std::ios::out | std::ios::binary );

                /* check output stream */
                if ( dl_ostream.is_open() == true ) {

                    /* parsing input stream */
                    while ( ( dl_size = lc_ply_io_i_chunk( & dl_istream, DL_PLY_UF3_CHUNK ) ) > 0 ) {

                        /* reset output chunk parser */
                        dl_parse = 0;

                        /* parsing input chunk */
                        for ( long long int dl_index( 0 ); dl_index < dl_size; dl_index ++ ) {

                            /* create i/o buffer mapping */
                            dl_pose = ( double * ) ( dl_buffer + dl_parse );
                            dl_data = ( char   * ) ( dl_pose + 3 );

                            /* compose output stream chunk */
                            dl_pose[0] = lc_ply_get_x( & dl_istream, dl_index );
                            dl_pose[1] = lc_ply_get_y( & dl_istream, dl_index );
                            dl_pose[2] = lc_ply_get_z( & dl_istream, dl_index );

                            /* input stream vertex filtering */
                            if ( dl_pose[0] != dl_pose[0] ) continue;
                            if ( dl_pose[1] != dl_pose[1] ) continue;
                            if ( dl_pose[2] != dl_pose[2] ) continue;

                            /* compose output stream chunk */
                            dl_data[0] = lc_ply_get_red  ( & dl_istream, dl_index );
                            dl_data[1] = lc_ply_get_green( & dl_istream, dl_index );
                            dl_data[2] = lc_ply_get_blue ( & dl_istream, dl_index );

                            /* update offset */
                            dl_parse += 27;

                        }

                        /* write output stream chunk */
                        dl_ostream.write( dl_buffer, dl_parse );

                    }

                    /* delete output stream */
                    dl_ostream.close();

                } else {

                    /* display message */
                    std::cerr << "dalai-suite : error : unable to access output file" << std::endl;

                    /* push message */
                    dl_message = EXIT_FAILURE;

                }

                /* delete input stream */
                lc_ply_delete( & dl_istream );

            } else {

                /* display message */
                std::cerr << "dalai-suite : error : unable to access input file or unsupported ply format" << std::endl;

                /* push message */
                dl_message = EXIT_FAILURE;

            }

            /* release i/o buffer memory */
            delete [] dl_buffer;

        }

        /* send message */
        return( dl_message );

    }

