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
    source - constructor/destructor methods
 */

    int main( int argc, char ** argv ) {

        /* Stream variables */
        FILE * dl_stream;

        /* Polygone File Format reader variables */
        pcl::PLYReader dl_ply;

        /* Point cloud structure variables */
        pcl::PointCloud < pcl::PointXYZRGB > dl_data;
    
        /* Polygone File Format content importation */
        if ( dl_ply.read( lc_read_string( argc, argv, "--ply", "-i" ), dl_data ) == 0 ) {

            /* Open output stream */
            dl_stream = fopen( lc_read_string( argc, argv, "--universal", "-o" ), "w" );

            /* Parsing point cloud vertex */
            for ( size_t dh_i = 0; dh_i < dl_data.size(); dh_i ++ ) {

                /* Point cloud exportation - spatial coordinates */
                fprintf( dl_stream, "%.14e %.14e %.14e ", dl_data.points[dh_i].x, dl_data.points[dh_i].y, dl_data.points[dh_i].z );

                /* Point cloud exportation - temporal coordinates */
                fprintf( dl_stream, "%s ", argv[3] );

                /* Point cloud exportation - colorimetry */
                fprintf( dl_stream, "%.4f %.4f %.4f\n", ( float ) dl_data.points[dh_i].r / 255, ( float ) dl_data.points[dh_i].g / 255, ( float ) dl_data.points[dh_i].b / 255 );

            }

            /* Close output stream */
            fclose( dl_stream );

            /* Send message */
            return( EXIT_SUCCESS );

        } else {

            /* Display message */
            std::cerr << "dalai-suite : error : unable to access input ply" << std::endl;

            /* Send message */
            return( EXIT_FAILURE );

        }

    }

