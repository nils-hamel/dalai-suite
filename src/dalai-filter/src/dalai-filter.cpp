/*
 *  dalai-suite - filter
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

    # include "dalai-filter.hpp"

/*
    source - filtering methods
 */

    le_void_t dl_filter( std::ofstream & dl_ostream, le_char_t const * const dl_ipath, le_real_t dl_mean, le_real_t const dl_factor, le_size_t const dl_threshold, bool const dl_adaptive ) {

        /* input stream variable */
        std::ifstream dl_istream;

        /* path variable */
        le_char_t dl_fpath[_LE_USE_PATH];

        /* directory structure variable */
        DIR * dl_directory( nullptr );

        /* entity structure variable */
        struct dirent * dl_entity( nullptr );

        /* open and check directory */
        if ( ( dl_directory = opendir( ( char * ) dl_ipath ) ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* directory enumeration */
        while ( ( dl_entity = readdir( dl_directory ) ) != nullptr ) {

            /* filter regular file */
            if ( dl_entity->d_type == DT_REG ) {

                /* compose path */
                sprintf( ( char * ) dl_fpath, "%s/%s", ( char * ) dl_ipath, dl_entity->d_name );

                /* check filtering method */
                if ( dl_adaptive == true ) {

                    /* adaptive filtering */
                    lc_filter_adaptative( dl_fpath, dl_ostream, dl_factor, dl_threshold );

                } else {

                    /* homogeneous filtering */
                    lc_filter_homogeneous( dl_fpath, dl_ostream, dl_mean, dl_factor, dl_threshold );

                }

                /* remove filtered file */
                std::remove( ( char * ) dl_fpath );

            }

        }

        /* close directory */
        closedir( dl_directory );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* filtering factor variable */
        le_real_t dl_factor( lc_read_double( argc, argv, "--factor", "-f", 2.0 ) );

        /* computation count variable */
        le_size_t dl_count( lc_read_signed( argc, argv, "--count", "-c", 64 ) );

        /* filtering threshold variable */
        le_size_t dl_threshold( lc_read_signed( argc, argv, "--threshold", "-t", 2 ) );

        /* filtering mode variable */
        bool dl_mode( lc_read_flag( argc, argv, "--adaptive", "-a" ) );

        /* temporary path variable */
        le_char_t dl_path[_LE_USE_PATH];

        /* minimum distance mean variable */
        le_real_t dl_mean( 0.0 );

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

            /* throw message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* throw message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* create temporary storage */
        lc_temp_directory( lc_read_string( argc, argv, "--temporary", "-y" ), ( char * ) dl_path, LC_TEMP_CREATE );

        /* compute minimum distance mean value */
        dl_mean = lc_statistic_mdmv( dl_istream, dl_count );

        /* create hashed storage */
        lc_hash( dl_istream, dl_path, DL_FILTER_HASH, dl_mean );

        /* filtering process */
        dl_filter( dl_ostream, dl_path, dl_mean, dl_factor, dl_threshold, dl_mode );

        /* delete temporary storage */
        lc_temp_directory( nullptr, ( char * ) dl_path, LC_TEMP_DELETE );

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

