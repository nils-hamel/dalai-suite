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

    # include "dalai-universal-ply.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* Universal stream variables */
        std::ifstream dl_stream;

        /* Writer variables */
        pcl::PLYWriter dl_ply;

        /* Point cloud variables */
        pcl::PointCloud < pcl::PointXYZRGB > dl_cloud;

        /* I/O buffer variables */
        char dl_buffer[27] = { 0 };

        /* I/O buffer pointers variables */
        double  * dl_pose = ( double  * ) ( dl_buffer      );
        uint8_t * dl_data = ( uint8_t * ) ( dl_buffer + 24 );

        /* Create universal stream */
        dl_stream.open( lc_read_string( argc, argv, "--universal", "-i" ), std::ios::in | std::ios::binary );

        /* Check universal stream */
        if ( dl_stream.is_open() ) {

            /* Stream offset to end */
            dl_stream.seekg( 0, std::ios::end );

            /* Allocating point cloud memory */
            dl_cloud.points.resize( dl_stream.tellg() / 27 );

            /* Stream offset to begining */
            dl_stream.seekg( 0, std::ios::beg );

            /* Parsing input stream */
            for ( unsigned long dl_parse = 0; dl_parse < dl_cloud.size(); dl_parse ++ ) {

                /* Read input stream record */
                dl_stream.read( dl_buffer, 27 );

                /* Decompose i/o buffer - space components */
                dl_cloud.points[dl_parse].x = dl_pose[0];
                dl_cloud.points[dl_parse].y = dl_pose[1];
                dl_cloud.points[dl_parse].z = dl_pose[2];

                /* Decompose i/o buffer - color components */
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

                /* Send message */
                return( EXIT_FAILURE );

            } else {

                /* Send message */
                return( EXIT_SUCCESS );

            }

        } else {

            /* Display message */
            std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

            /* Send message */
            return( EXIT_FAILURE );

        }

    }

