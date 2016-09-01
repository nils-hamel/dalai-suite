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

    # include "dalai-ply-universal.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* Status variables */
        int dl_status( EXIT_SUCCESS );

        /* Stream variables */
        std::ofstream dl_stream;

        /* Reader class variables */
        pcl::PLYReader dl_ply;

        /* Point cloud class variables */
        pcl::PointCloud < pcl::PointXYZRGB > dl_cloud;

        /* Stream buffer variables */
        char dl_buffer[27] = { 0 };

        /* Stream buffer pointers variables */
        lc_real_t * dl_pose( ( lc_real_t * ) ( dl_buffer      ) );
        lc_data_t * dl_data( ( lc_data_t * ) ( dl_buffer + 24 ) );        

        /* Polygone file format content importation */
        if ( dl_ply.read( lc_read_string( argc, argv, "--ply", "-i" ), dl_cloud ) == 0 ) {

            /* Create output stream */
            dl_stream.open( lc_read_string( argc, argv, "--universal", "-o" ), std::ios::out | std::ios::binary );

            /* Check output stream */
            if ( dl_stream.is_open() ) {

                /* Parsing point cloud */
                for ( unsigned long dl_i = 0; dl_i < dl_cloud.size(); dl_i ++ ) {

                    /* Assign point coordinates */
                    dl_pose[0] = dl_cloud.points[dl_i].x;
                    dl_pose[1] = dl_cloud.points[dl_i].y;
                    dl_pose[2] = dl_cloud.points[dl_i].z;

                    /* Vertex filtering */
                    if ( dl_pose[0] != dl_pose[0] ) continue;
                    if ( dl_pose[1] != dl_pose[1] ) continue;
                    if ( dl_pose[2] != dl_pose[2] ) continue;

                    /* Assign point color components */
                    dl_data[0] = dl_cloud.points[dl_i].r;
                    dl_data[1] = dl_cloud.points[dl_i].g;
                    dl_data[2] = dl_cloud.points[dl_i].b;

                    /* Output stream exportation */
                    dl_stream.write( dl_buffer, 27 );

                }

                /* Delete output stream */
                dl_stream.close();

            } else {

                /* Display message */
                std::cerr << "dalai-suite : error : unable to access output file" << std::endl;

                /* Push message */
                dl_status = EXIT_FAILURE;

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

