/*
 *  dalai-suite - las-uv3
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2018 DHLAB, EPFL
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

    # include "dalai-las-uv3.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream buffer variables */
        le_byte_t dl_buffer[LE_UV3_RECORD] = { 0 };

        /* buffer mapping variables */
        le_real_t * dl_pose( ( le_real_t * ) ( dl_buffer ) );
        le_byte_t * dl_type( ( le_byte_t * ) ( dl_buffer + LE_UV3_POSE ) ); 
        le_data_t * dl_data( ( le_data_t * ) ( dl_buffer + LE_UV3_POSE + LE_UV3_TYPE ) );

        /* color availability variables */
        bool dl_color( false );

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* classification colormap variables */
        char dl_colormap[19][3] = DL_COLORMAP;

    /* error management */
     try {

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--las", "-i" ), std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--uf3", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* reader variables */
        liblas::Reader dl_las( dl_istream );

        /* check format consistency */
        if ( ( dl_las.GetHeader().GetVersionMajor() != 1 ) || ( dl_las.GetHeader().GetVersionMinor() > 4 ) ) {

            /* send message */
            throw( LC_ERROR_FORMAT );

        }

        /* check if classification colormap is forced */
        if ( lc_read_flag( argc, argv, "--classification", "-c" ) == false ) {

            /* retrieve input file format */
            int dl_format( dl_las.GetHeader().GetDataFormatId() );

            /* check color availability */
            if ( ( dl_format == 2 ) || ( dl_format == 3 ) || ( dl_format == 5 ) ) {

                /* reset color availability */
                dl_color = true;

            }

        }

        /* initialise recrod type */
        * ( dl_type ) = LE_UV3_POINT;

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
                int dl_class( dl_las.GetPoint().GetClassification().GetClass() );

                /* check classification index */
                if ( dl_class > 18 ) dl_class = 0;

                /* assign classification colormap element */
                dl_data[0] = dl_colormap[dl_class][0];
                dl_data[1] = dl_colormap[dl_class][1];
                dl_data[2] = dl_colormap[dl_class][2];

            }

            /* output stream exportation */
            dl_ostream.write( ( char * ) dl_buffer, LE_UV3_RECORD );

        }

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

