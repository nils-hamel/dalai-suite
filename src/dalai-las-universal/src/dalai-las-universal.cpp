/*
 *  dalai-suite - geodetic system
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

    # include "dalai-las-universal.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* Status variables */
        int dl_status( EXIT_SUCCESS );

        /* Stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* Stream buffer variables */
        char dl_buffer[27] = { 0 };

        /* Stream buffer pointers variables */
        lc_real_t * dl_pose( ( lc_real_t * ) ( dl_buffer      ) );
        lc_data_t * dl_data( ( lc_data_t * ) ( dl_buffer + 24 ) );

        /* Create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--las", "-i" ), std::ios::in | std::ios::binary );

        /* Check input stream */
        if ( dl_istream.is_open() == true ) {

            /* Create output stream */
            dl_ostream.open( lc_read_string( argc, argv, "--universal", "-o" ), std::ios::out | std::ios::binary );

            /* Check output stream */
            if ( dl_ostream.is_open() == true ) {

                /* Reader class variables */
                liblas::Reader dl_las( dl_istream );

                /* Parsing input stream */
                while ( dl_las.ReadNextPoint() == true ) {

                    /* Point class variables */
                    liblas::Point const & dl_point = dl_las.GetPoint();

                    /* Assign point coordinates */
                    dl_pose[0] = dl_point.GetX();
                    dl_pose[1] = dl_point.GetY();
                    dl_pose[2] = dl_point.GetZ();

                    /* Vertex filtering */
                    if ( dl_pose[0] != dl_pose[0] ) continue;
                    if ( dl_pose[1] != dl_pose[1] ) continue;
                    if ( dl_pose[2] != dl_pose[2] ) continue;

                    /* Assign point color components */
                    dl_data[0] = dl_point.GetColor().GetRed();
                    dl_data[1] = dl_point.GetColor().GetGreen();
                    dl_data[2] = dl_point.GetColor().GetBlue();

                    /* Output stream exportation */
                    dl_ostream.write( dl_buffer, 27 );

                }

                /* Delete output stream */
                dl_ostream.close();

            } else {

                /* Display message */
                std::cerr << "dalai-suite : error : unable to access output file" << std::endl;

                /* Push message */
                dl_status = EXIT_FAILURE;

            }

            /* Delete input stream */
            dl_istream.close();

        } else {

            /* Display message */
            std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

            /* Push message */
            dl_status = EXIT_FAILURE;

        }

        /* Send message */
        return( dl_status );

    }

