/*
 *  dalai-suite - las-uf3
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

    # include "dalai-las-uf3.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* color availability variables */
        bool dl_color( false );

        /* format variables */
        int dl_format( 0 );

        /* classifcation index variables */
        int dl_class( 0 );

        /* stream buffer variables */
        char dl_buffer[LC_UF3_RECLEN] = { 0 };

        /* buffer mapping variables */
        lc_uf3p_t * dl_pose( ( lc_uf3p_t * ) ( dl_buffer ) );
        lc_uf3d_t * dl_data( ( lc_uf3d_t * ) ( dl_buffer + LC_UF3_DATA ) );

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* classification colormap variables */
        char dl_colormap[19][3] = DL_COLORMAP;

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--las", "-i" ), std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access input stream" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--uf3", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access output stream" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* reader variables */
        liblas::Reader dl_las( dl_istream );

        /* check format consistency */
        if ( ( dl_las.GetHeader().GetVersionMajor() != 1 ) || ( dl_las.GetHeader().GetVersionMinor() > 4 ) ) {

            /* dispaly message */
            std::cerr << "dalai-suite : error : supports las format from 1.0 to 1.4" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* check if classification colormap is forced */
        if ( lc_read_flag( argc, argv, "--classification", "-c" ) == false ) {

            /* retrieve input file format */
            dl_format = dl_las.GetHeader().GetDataFormatId();

            /* check color availability */
            if ( ( dl_format == 2 ) || ( dl_format == 3 ) || ( dl_format == 5 ) ) {

                /* reset color availability */
                dl_color = true;

            }

        }

        /* parsing input stream */
        while ( dl_las.ReadNextPoint() == true ) {

            /* retrieve point coordinates */
            dl_pose[0] = dl_las.GetPoint().GetX();
            dl_pose[1] = dl_las.GetPoint().GetY();
            dl_pose[2] = dl_las.GetPoint().GetZ();

            /* vertex filtering - avoiding nan */
            if ( dl_pose[0] != dl_pose[0] ) continue;
            if ( dl_pose[1] != dl_pose[1] ) continue;
            if ( dl_pose[2] != dl_pose[2] ) continue;

            /* check colorimetry mode */
            if ( dl_color == true ) {

                /* retrieve point color components */
                dl_data[0] = dl_las.GetPoint().GetColor().GetRed();
                dl_data[1] = dl_las.GetPoint().GetColor().GetGreen();
                dl_data[2] = dl_las.GetPoint().GetColor().GetBlue();

            } else {

                /* retrieve point classification */
                dl_class = dl_las.GetPoint().GetClassification().GetClass();

                /* check classification index */
                if ( dl_class > 18 ) dl_class = 0;

                /* assign classification colormap element */
                dl_data[0] = dl_colormap[dl_class][0];
                dl_data[1] = dl_colormap[dl_class][1];
                dl_data[2] = dl_colormap[dl_class][2];

            }

            /* output stream exportation */
            dl_ostream.write( dl_buffer, LC_UF3_RECLEN );

        }

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

        /* send message */
        return( EXIT_SUCCESS );

    }

