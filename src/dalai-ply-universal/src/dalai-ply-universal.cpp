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

        /* Universal stream variables */
        std::ofstream dl_stream;

        /* Polygone File Format reader variables */
        pcl::PLYReader dl_ply;

        /* Point cloud structure variables */
        pcl::PointCloud < pcl::PointXYZRGB > dl_cloud;

        /* I/O buffer variables */
        char dl_buffer[27] = { 0 };

        /* I/O buffer pointers variables */
        double  * dl_pose = ( double  * ) ( dl_buffer      );
        uint8_t * dl_data = ( uint8_t * ) ( dl_buffer + 24 );        

        /* Polygone File Format content importation */
        if ( dl_ply.read( lc_read_string( argc, argv, "--ply", "-i" ), dl_cloud ) == 0 ) {

            /* Create universal stream */
            dl_stream.open( lc_read_string( argc, argv, "--universal", "-o" ), std::ios::out | std::ios::binary );

            /* Check universal stream */
            if ( dl_stream.is_open() ) {

                /* Parsing imported point cloud elements */
                for ( unsigned long dl_i = 0; dl_i < dl_cloud.size(); dl_i ++ ) {

                    /* Compose i/o buffer - space components */
                    dl_pose[0] = dl_cloud.points[dl_i].x;
                    dl_pose[1] = dl_cloud.points[dl_i].y;
                    dl_pose[2] = dl_cloud.points[dl_i].z;

                    /* Vertex filtering */
                    if ( dl_pose[0] != dl_pose[0] ) continue;
                    if ( dl_pose[1] != dl_pose[1] ) continue;
                    if ( dl_pose[2] != dl_pose[2] ) continue;

                    /* Compose i/o buffer - color components */
                    dl_data[0] = dl_cloud.points[dl_i].r;
                    dl_data[1] = dl_cloud.points[dl_i].g;
                    dl_data[2] = dl_cloud.points[dl_i].b;

                    /* Write i/o buffer in universal stream */
                    dl_stream.write( dl_buffer, 27 );

                }

                /* Delete universal stream */
                dl_stream.close();

                /* Send message */
                return( EXIT_SUCCESS );

            } else {

                /* Display message */
                std::cerr << "dalai-suite : error : unable to access output file" << std::endl;

                /* Send message */
                return( EXIT_FAILURE );

            }

        } else {

            /* Display message */
            std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

            /* Send message */
            return( EXIT_FAILURE );

        }

    }

