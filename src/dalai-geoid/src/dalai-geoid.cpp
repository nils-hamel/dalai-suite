/*
 *  dalai-suite - geoid
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2019 DHLAB, EPFL
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

    # include "dalai-geoid.hpp"

/*
    source - conversion methods
 */

    le_void_t dl_geoid_height( le_char_t const * const dl_input, le_char_t const * const dl_output, GeographicLib::Geoid & dl_geoid, le_real_t const dl_conversion ) {

        /* data buffer variable */
        le_byte_t dl_buffer[LE_ARRAY_DATA];

        /* data pointer variable */
        le_real_t * dl_pose( nullptr );

        /* readind variable */
        le_size_t dl_read( 1 );

        /* stream variable */
        std::ifstream dl_istream( ( char * ) dl_input, std::ios::binary );

        /* stream variable */
        std::ofstream dl_ostream( ( char * ) dl_output, std::ios::binary );

        /* check stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* check stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* parsing input stream */
        while ( dl_read > 0 ) {

            /* read data chunk */
            dl_istream.read( ( char * ) dl_buffer, LE_ARRAY_DATA );

            /* check read chunk */
            if ( ( dl_read = dl_istream.gcount() ) > 0 ) {

                /* parsing chunk vertex */
                for ( le_size_t dl_parse( 0 ); dl_parse < dl_read; dl_parse += LE_ARRAY_DATA ) {

                    /* create data pointer */
                    dl_pose = ( le_real_t * ) ( dl_buffer + dl_parse );

                    /* vertex conversion */
                    dl_pose[2] = dl_pose[2] + dl_conversion * dl_geoid( dl_pose[1] * LE_R2D, dl_pose[0] * LE_R2D );

                }

                /* export processed chunk */
                dl_ostream.write( ( char * ) dl_buffer, dl_read );

            }

        }

        /* delete stream */
        dl_ostream.close();

        /* delete stream */
        dl_istream.close();

    }

/*
    source - batch methods
 */

    le_void_t dl_geoid_batch( le_char_t const * const dl_input, le_char_t const * const dl_output, GeographicLib::Geoid & dl_geoid, le_real_t const dl_conversion ) {

        /* directory variable */
        DIR * dl_directory( nullptr );

        /* entity variable */
        struct dirent * dl_entity( nullptr );

        /* string conversion */
        std::string dl_inbase( ( char * ) dl_input );

        /* string conversion */
        std::string dl_outbase( ( char * ) dl_output );

        /* check consistency */
        if ( ( dl_directory = opendir( ( char * ) dl_input ) ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* entity enumeration */
        while ( ( dl_entity = readdir( dl_directory ) ) != NULL ) {

            /* check for files */
            if ( dl_entity->d_type == DT_REG ) {

                /* convert string */
                std::string dl_name( dl_entity->d_name );

                /* filter on extension */
                if ( dl_name.substr( dl_name.length() - 4, 4 ) == ".uv3" ) {

                    /* process file */
                    dl_geoid_height( ( le_char_t * ) ( dl_inbase + "/" + dl_name ).c_str(), ( le_char_t * ) ( dl_outbase + "/" + dl_name ).c_str(), dl_geoid, dl_conversion );

                }

            }

        }

        /* close directory */
        closedir( dl_directory );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* geoid name variable */
        le_char_t * dl_name = ( le_char_t * ) lc_read_string( argc, argv, "--name", "-n" );

        /* geoid path variable */
        le_char_t * dl_path = ( le_char_t * ) lc_read_string( argc, argv, "--path", "-p" );

        /* i/o path variable */
        le_char_t * dl_input = ( le_char_t * ) lc_read_string( argc, argv, "--input", "-i" );

        /* i/o path variable */
        le_char_t * dl_output = ( le_char_t * ) lc_read_string( argc, argv, "--output", "-o" );

        /* conversion variable */
        le_real_t dl_conversion( +1.0 );

    /* error management */
    try {

        /* geolib geoid variable */
        GeographicLib::Geoid dl_geoid( std::string( ( char * ) dl_name ), std::string( ( char * ) dl_path ), true );

        /* conversion detection */
        if ( lc_read_flag( argc, argv, "--to-msl", "-m" ) == true ) {

            /* update conversion */
            dl_conversion = -1.0;

        } else if ( lc_read_flag( argc, argv, "--to-ell", "-e" ) == true ) {

            /* update conversion (default) */
            dl_conversion = +1.0;

        }

        /* check execution mode */
        if ( lc_file_detect( ( char * ) dl_input ) == LC_DIRECTORY ) {

            /* check consistency */
            if ( lc_file_detect( ( char * ) dl_output ) != LC_DIRECTORY ) {

                /* send message */
                throw( LC_ERROR_IO_WRITE );

            }

            /* batch processing */
            dl_geoid_batch( dl_input, dl_output, dl_geoid, dl_conversion );

        } else {

            /* process file */
            dl_geoid_height( dl_input, dl_output, dl_geoid, dl_conversion );

        }

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

