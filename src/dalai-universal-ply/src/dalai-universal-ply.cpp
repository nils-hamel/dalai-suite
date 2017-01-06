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

    # include "dalai-universal-ply.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* Status variables */
        int dl_status( EXIT_SUCCESS );

        /* Stream variables */
        std::ifstream dl_stream;

        /* Writer class variables */
        pcl::PLYWriter dl_ply;

        /* Point cloud class variables */
        pcl::PointCloud < pcl::PointXYZRGB > dl_cloud;

        /* Stream buffer variables */
        char dl_buffer[27] = { 0 };

        /* Stream buffer pointers variables */
        lc_real_t * dl_pose( ( lc_real_t * ) ( dl_buffer      ) );
        lc_data_t * dl_data( ( lc_data_t * ) ( dl_buffer + 24 ) );

        /* Create input stream */
        dl_stream.open( lc_read_string( argc, argv, "--universal", "-i" ), std::ios::ate | std::ios::in | std::ios::binary );

        /* Check input stream */
        if ( dl_stream.is_open() ) {

            /* Creating point cloud memory allocation */
            dl_cloud.points.resize( dl_stream.tellg() / 27 );

            /* Stream offset to begining */
            dl_stream.seekg( 0, std::ios::beg );

            /* Parsing input stream */
            for ( unsigned long dl_parse = 0; dl_parse < dl_cloud.size(); dl_parse ++ ) {

                /* Read input stream */
                dl_stream.read( dl_buffer, 27 );

                /* Assign point coordinates */
                dl_cloud.points[dl_parse].x = dl_pose[0];
                dl_cloud.points[dl_parse].y = dl_pose[1];
                dl_cloud.points[dl_parse].z = dl_pose[2];

                    /* Assign point color components */
                dl_cloud.points[dl_parse].r = dl_data[0];
                dl_cloud.points[dl_parse].g = dl_data[1];
                dl_cloud.points[dl_parse].b = dl_data[2];

            }

            /* Delete universal stream */
            dl_stream.close();

            /* Write output stream */
            if ( dl_ply.write( lc_read_string( argc, argv, "--ply", "-o" ), dl_cloud, true, false ) != 0 ) {

                /* Display message */
                std::cerr << "dalai-suite : error : unable to write output file" << std::endl;

                /* Push message */
                dl_status = EXIT_FAILURE;

            }

            /* Clear point cloud */
            dl_cloud.clear();

        } else {

            /* Display message */
            std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

            /* Push message */
            dl_status = EXIT_FAILURE;

        }

        /* Send message */
        return( dl_status );

    }

