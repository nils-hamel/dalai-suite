/*
 *  dalai-suite - hash
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

    # include "dalai-hash.hpp"

/*
    source - statistical methods
 */

    double dl_hash_stat( std::ifstream & dl_istream, int64_t const dl_size, int64_t const dl_count ) {

        /* distance variables */
        double dl_distance( 0.0 );

        /* buffer variables */
        char   * dl_sample( nullptr );
        char   * dl_chunks( nullptr );
        double * dl_values( nullptr );

        /* array mapping variables */
        double * dl_posei( nullptr );
        double * dl_poses( nullptr );

        /* allocate and check buffer memory */
        if ( ( dl_sample = new char[dl_count * 27] ) == nullptr ) {

            /* send message */
            return( 0.0 );

        } else {

            /* allocate and check buffer memory */
            if ( ( dl_chunks = new char[DL_HASH_CHUNK * 27] ) == nullptr ) {

                /* release buffers memory */
                delete [] dl_sample;

                /* send message */
                return( 0.0 );

            } else {

                /* allocate and check buffer memory */
                if ( ( dl_values = new double[dl_count] ) == nullptr ) {

                    /* release buffers memory */
                    delete [] dl_sample;
                    delete [] dl_chunks;

                    /* send message */
                    return( 0.0 );

                }

            }

        }

        /* initialise values array */
        for ( int64_t dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

            /* assign initial value */
            dl_values[dl_parse] = std::numeric_limits<double>::max();

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* create sample array */
        for ( int64_t dl_parse = 0; dl_parse < dl_count; dl_parse ++ ) {

            /* assign stream position */
            dl_istream.seekg( dl_parse * ( ( ( dl_size / 27 ) / dl_count ) * 27 ) );

            /* read sample values */
            dl_istream.read( dl_sample + ( dl_parse * 27 ), 27 );

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* reading input stream chunks */
        do {

            /* read input stream chunk */
            dl_istream.read( dl_chunks, DL_HASH_CHUNK * 27 );

            /* parsing input stream chunk */
            for ( int64_t dl_parse( 0 ), dl_limit( dl_istream.gcount() ); dl_parse < dl_limit; dl_parse += 27 ) {

                /* compute and assign array mapping */
                dl_posei = ( double * ) ( dl_chunks + dl_parse );

                /* parsing sample array */
                for ( int64_t dl_index( 0 ); dl_index < dl_count; dl_index ++ ) {

                    /* compute and assign array mapping */
                    dl_poses = ( double * ) ( dl_sample + ( dl_index * 27 ) );

                    /* compute and check distance */
                    if ( ( dl_distance =

                        ( dl_posei[0] - dl_poses[0] ) * ( dl_posei[0] - dl_poses[0] ) +
                        ( dl_posei[1] - dl_poses[1] ) * ( dl_posei[1] - dl_poses[1] ) +
                        ( dl_posei[2] - dl_poses[2] ) * ( dl_posei[2] - dl_poses[2] )

                    ) < dl_values[dl_index] ) {

                        /* avoid identical point */
                        if ( dl_distance > 0.0 ) dl_values[dl_index] = dl_distance;

                    }

                }

            }

        } while ( dl_istream.gcount() > 0 );

        /* prepare statistical mean computation */
        dl_distance = 0.0;

        /* compute statistical mean value */
        for ( int64_t dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

            /* accumulate distances */
            dl_distance += sqrt( dl_values[dl_parse] );

        }

        /* release buffer memory */
        delete [] dl_sample;
        delete [] dl_chunks;
        delete [] dl_values;

        /* send message - mean minimum value */
        return( dl_distance / dl_count );

    }

/*
    source - hashing methods
 */

    bool dl_hash( std::ifstream & dl_istream, char const * const dl_opath, double const dl_param, double const dl_mean ) {

        /* stream path variables */
        char dl_fpath[8192];

        /* stream variables */
        std::ofstream dl_ostream;

        /* buffer variables */
        char * dl_chunk( nullptr );

        /* array mapping variables */
        double * dl_pose( nullptr );

        /* hashing parameter variables */
        double dl_hash( dl_mean * dl_param );

        /* returned value variables */
        bool dl_return( true );

        /* check consistency */
        if ( dl_opath == nullptr ) {

            /* send message */
            return( false );

        }

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new char[DL_HASH_CHUNK * 27] ) == nullptr ) {

            /* send message */
            return( false );

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* parsing input stream chunks */
        do {

            /* read input stream chunk */
            dl_istream.read( dl_chunk, DL_HASH_CHUNK * 27 );

            /* parsing input stream chunk */
            for ( int64_t dl_parse( 0 ), dl_limit( dl_istream.gcount() ); dl_parse < dl_limit; dl_parse += 27 ) {

                /* compute and assign array mapping */
                dl_pose = ( double * ) ( dl_chunk + dl_parse );

                /* compute stream path */
                sprintf( dl_fpath, "%s/%+" PRId64 "_%+" PRId64 "_%+" PRId64 ".uf3", dl_opath, ( int64_t ) floor( dl_pose[0] / dl_hash ), ( int64_t ) floor( dl_pose[1] / dl_hash ), ( int64_t ) floor( dl_pose[2] / dl_hash ) );

                /* create output stream */
                dl_ostream.open( dl_fpath, std::ios::app | std::ios::out | std::ios::binary );

                /* check output stream */
                if ( dl_ostream.is_open() == true ) {

                    /* export chunk element */
                    dl_ostream.write( dl_chunk + dl_parse, 27 );

                    /* delete output stream */
                    dl_ostream.close();

                /* push message */
                } else { dl_return = false; }

            }

        } while ( ( dl_istream.gcount() > 0 ) && ( dl_return == true ) );

        /* release buffer memory */
        delete [] dl_chunk;

        /* send message */
        return( dl_return );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream variables */
        std::ifstream dl_istream;

        /* sampling count variables */
        int64_t dl_count( lc_read_signed( argc, argv, "--count", "-c", 64 ) );

        /* hashing parameter variables */
        double dl_param( lc_read_double( argc, argv, "--param", "-p", 250.0 ) );

        /* statistical mean variables */
        double dl_mean( 0.0 );

        /* returned value variables */
        int dl_return( EXIT_SUCCESS );

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--uf3", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == true ) {

            /* compute and check statistical mean */
            if ( ( dl_mean = dl_hash_stat( dl_istream, dl_istream.tellg(), dl_count ) ) > 0.0 ) {

                /* hashing input stream */
                if ( dl_hash( dl_istream, lc_read_string( argc, argv, "--output", "-o" ), dl_param, dl_mean ) == false ) {

                    /* display message */
                    std::cerr << "dalai-suite : error : unable to hash input stream" << std::endl;

                    /* push message */
                    dl_return = EXIT_FAILURE;

                }

            } else {

                /* display message */
                std::cerr << "dalai-suite : error : unable to compute statistical parameter" << std::endl;

                /* push message */
                dl_return = EXIT_FAILURE;

            }

            /* delete output stream */
            dl_istream.close();

        } else {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access input stream" << std::endl;

            /* push message */
            dl_return = EXIT_FAILURE;

        }

        /* send message */
        return( dl_return );

    }

