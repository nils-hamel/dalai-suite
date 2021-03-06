/*
 *  dalai-suite - las-uv3
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
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

        /* intensities factor variable */
        le_real_t dl_factor( lc_read_double( argc, argv, "--factor", "-f", 1.0 ) );

        /* buffer variable */
        le_byte_t dl_buffer[LE_ARRAY_DATA] = { 0 };

        /* buffer pointer variable */
        le_real_t * dl_uv3p( nullptr );
        le_data_t * dl_uv3d( nullptr );

        /* color mapping variable */
        char dl_colormap[19][3] = DL_COLORMAP;

        /* extraction mode variable */
        le_enum_t dl_extract = DL_EXTRACT_CLASS;

        /* stream variable */
        std::ifstream dl_istream;

        /* stream variable */
        std::ofstream dl_ostream;

    /* error management */
     try {

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

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

        /* check classification switch */
        if ( lc_read_flag( argc, argv, "--classification", "-c" ) == true ) {

            /* assign extraction mode */
            dl_extract = DL_EXTRACT_CLASS;

        } else {

            /* check color switch */
            if ( lc_read_flag( argc, argv, "--color", "-r" ) == true ) {

                /* extract LAS format */
                le_enum_t dl_format( dl_las.GetHeader().GetDataFormatId() );

                /* check consistency */
                if ( ( dl_format == 2 ) || ( dl_format == 3 ) || ( dl_format == 5 ) ) {

                    /* assign extraction mode */
                    dl_extract = DL_EXTRACT_COLOR;

                } else {

                    /* send message */
                    throw( LC_ERROR_FORMAT );

                }

            } else {

                /* check intensity switch */
                if ( lc_read_flag( argc, argv, "--intensity", "-e" ) == true ) {

                    /* assign extraction mode */
                    dl_extract = DL_EXTRACT_INTEN;

                }

            }

        }

        /* compute buffer pointer */
        dl_uv3p = ( le_real_t * ) ( dl_buffer );

        /* compute buffer pointer */
        dl_uv3d = ( le_data_t * ) ( dl_uv3p + 3 );

        /* assign primitive type */
        ( * dl_uv3d ) = LE_UV3_POINT;

        /* parsing input stream */
        while ( dl_las.ReadNextPoint() == true ) {

            /* retrieve point coordinates */
            dl_uv3p[0] = dl_las.GetPoint().GetX();
            dl_uv3p[1] = dl_las.GetPoint().GetY();
            dl_uv3p[2] = dl_las.GetPoint().GetZ();

            /* vertex filtering - avoiding nan */
            if ( dl_uv3p[0] != dl_uv3p[0] ) continue;
            if ( dl_uv3p[1] != dl_uv3p[1] ) continue;
            if ( dl_uv3p[2] != dl_uv3p[2] ) continue;

            /* check extraction mode */
            if ( dl_extract == DL_EXTRACT_CLASS ) {

                /* retrieve point classification */
                le_enum_t dl_class( dl_las.GetPoint().GetClassification().GetClass() % 19 );

                /* assign classification colormap */
                dl_uv3d[1] = dl_colormap[dl_class][0];
                dl_uv3d[2] = dl_colormap[dl_class][1];
                dl_uv3d[3] = dl_colormap[dl_class][2];

            } else if ( dl_extract == DL_EXTRACT_COLOR ) {

                /* assign point color components */
                dl_uv3d[1] = dl_las.GetPoint().GetColor().GetRed()   >> 8;
                dl_uv3d[2] = dl_las.GetPoint().GetColor().GetGreen() >> 8;
                dl_uv3d[3] = dl_las.GetPoint().GetColor().GetBlue()  >> 8;

            } else if ( dl_extract == DL_EXTRACT_INTEN ) {

                /* retrieve point intensity */
                le_size_t dl_inten( dl_las.GetPoint().GetIntensity() * dl_factor );

                /* assign point intensity */
                dl_uv3d[1] = dl_inten;
                dl_uv3d[2] = dl_inten;
                dl_uv3d[3] = dl_inten;

            }

            /* output stream exportation */
            dl_ostream.write( ( char * ) dl_buffer, LE_ARRAY_DATA );

        }

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

    /* error management */
    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

