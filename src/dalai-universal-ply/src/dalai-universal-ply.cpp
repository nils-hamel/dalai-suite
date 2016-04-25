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
    source - constructor/destructor methods
 */

    int main( int argc, char ** argv ) {

        /* I/O buffer variables */
        char dl_buffer[35] = { 0 };

        /* Universal stream variables */
        std::ifstream dl_stream;

        /* Writer variables */
        pcl::PLYWriter dl_ply;

        /* Point cloud variables */
        pcl::PointCloud < pcl::PointXYZRGB > dl_data;

        /* Path variables */
        std::string dl_path( lc_read_string( argc, argv, "--ply", "-o" ) );

        /* Create universal stream */
        dl_stream.open( lc_read_string( argc, argv, "--universal", "-i" ), std::ios::in | std::ios::binary );

        /* Check universal stream */
        if ( dl_stream.is_open() ) {

            /* Stream offset to end */
            dl_stream.seekg( 0, std::ios::end );

            /* Allocating point cloud memory */
            dl_data.points.resize( dl_stream.tellg() / 35 );

            /* Stream offset to begining */
            dl_stream.seekg( 0, std::ios::beg );

            /* Segmented buffer pointer variables */
            double  * dl_scomp = ( double  * ) ( dl_buffer      );
            uint8_t * dl_dcomp = ( uint8_t * ) ( dl_buffer + 32 );

            /* Parsing input stream */
            for ( unsigned long dl_parse = 0; dl_parse < dl_data.size(); dl_parse ++ ) {

                /* Read input stream record */
                dl_stream.read( dl_buffer, 35 );

                /* Decompose i/o buffer - space components */
                dl_data.points[dl_parse].x = dl_scomp[0];
                dl_data.points[dl_parse].y = dl_scomp[1];
                dl_data.points[dl_parse].z = dl_scomp[2];

                dl_data.points[dl_parse].r = dl_dcomp[0];
                dl_data.points[dl_parse].g = dl_dcomp[1];
                dl_data.points[dl_parse].b = dl_dcomp[2];

            }

            /* Delete universal stream */
            dl_stream.close();

            /* Write output stream */
            dl_ply.write( dl_path, dl_data, true, false );

            /* Send message */
            return( EXIT_SUCCESS );

        } else {

            /* Display message */
            std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

            /* Send message */
            return( EXIT_FAILURE );

        }

    }

