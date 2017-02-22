/*
 *  dalai-suite - filter
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

    # include "dalai-filter.hpp"

/*
    source - storage methods
 */

    void dl_filter_temporary( char * const dl_path, int dl_mode ) {

        /* check mode value */
        if ( dl_mode == DL_FILTER_CREATE ) {

            /* compose temporary storage path */
            strcpy( dl_path, "/tmp/dalai-suite-XXXXXX" );

            /* create temporary storage */
            if ( mkdtemp( dl_path ) == nullptr ) {

                /* throw message */
                throw( LC_ERROR_IO_ACCESS );

            }

        } else {

            /* delete temporary storage */
            if ( rmdir( dl_path ) != 0 ) {

                /* throw message */
                throw( LC_ERROR_IO_REMOVE );

            }

        }

    }

/*
    source - filtering methods
 */

    void dl_filter( std::ofstream & dl_ostream, char const * const dl_ipath, double const dl_mean, double const dl_factor, int64_t const dl_threshold, bool const dl_adaptative ) {

        /* input stream variables */
        std::ifstream dl_istream;

        /* input stream path variables */
        char dl_fpath[256];

        /* directory structure variables */
        DIR * dl_directory( nullptr );

        /* entity structure variables */
        struct dirent * dl_entity( nullptr );

        /* open and check directory */
        if ( ( dl_directory = opendir( dl_ipath ) ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* directory enumeration */
        while ( ( dl_entity = readdir( dl_directory ) ) != nullptr ) {

            /* regular entity filtering */
            if ( dl_entity->d_type != DT_REG ) {

                /* continue enumeration */
                continue;

            }

            /* compose input stream path */
            sprintf( dl_fpath, "%s/%s", dl_ipath, dl_entity->d_name );

            /* create input stream */
            dl_istream.open( dl_fpath, std::ios::in | std::ios::binary );

            /* check input stream */
            if ( dl_istream.is_open() == false ) {

                /* send message */
                throw( LC_ERROR_IO_ACCESS );

            }

            /* select filtering method */
            if ( dl_adaptative == true ) {

                /* filtering method */
                lc_filter_adaptative( dl_istream, dl_ostream, dl_mean, dl_factor, dl_threshold );

            } else {

                /* filtering method */
                lc_filter_homogeneous( dl_istream, dl_ostream, dl_mean, dl_factor, dl_threshold );

            }

            /* delete input stream */
            dl_istream.close();

            /* remove input stream file */
            std::remove( dl_fpath );

        }

        /* close directory */
        closedir( dl_directory );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* filtering factor variables */
        double dl_factor( lc_read_double( argc, argv, "--factor", "-f", 2.0 ) );

        /* computation count variables */
        int64_t dl_count( lc_read_signed( argc, argv, "--count", "-c", 64 ) );

        /* filtering threshold variables */
        int64_t dl_thres( lc_read_signed( argc, argv, "--threshold", "-t", 2 ) );

        /* filtering mode variables */
        bool dl_mode( lc_read_flag( argc, argv, "--adaptative", "-a" ) );

        /* temporary path variables */
        char dl_path[256];

        /* minimums mean variables */
        double dl_mean( 0.0 );

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

    /* error managament */
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
        dl_filter_temporary( dl_path, DL_FILTER_CREATE );

        /* compute minimum distance mean value */
        dl_mean = lc_statistic_mdmv( dl_istream, dl_count, LC_UF3_CHUNK );

        /* create hashed storage */
        lc_hash( dl_istream, dl_path, DL_FILTER_HASH, dl_mean, LC_UF3_CHUNK );

        /* filtering process */
        dl_filter( dl_ostream, dl_path, dl_mean, dl_factor, dl_thres, dl_mode );

        /* delete temporary storage */
        dl_filter_temporary( dl_path, DL_FILTER_DELETE );

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

    /* error managament */
    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

