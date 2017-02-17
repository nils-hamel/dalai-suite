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

        /* array mapping variables */
        lc_uf3p_t * dl_posei( nullptr );
        lc_uf3p_t * dl_poses( nullptr );

        /* distance variables */
        double dl_distance( 0.0 );

        /* buffer variables */
        char   * dl_sample( nullptr );
        char   * dl_chunks( nullptr );
        double * dl_values( nullptr );

        /* allocate and check buffer memory */
        if ( ( dl_sample = new ( std::nothrow ) char[dl_count * LC_UF3_RECLEN] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate and check buffer memory */
        if ( ( dl_chunks = new ( std::nothrow ) char[DL_HASH_CHUNK * LC_UF3_RECLEN] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate and check buffer memory */
        if ( ( dl_values = new ( std::nothrow ) double[dl_count] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

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
        for ( int64_t dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

            /* assign stream position */
            dl_istream.seekg( dl_parse * ( ( ( dl_size / LC_UF3_RECLEN ) / dl_count ) * LC_UF3_RECLEN ) );

            /* read sample values */
            dl_istream.read( dl_sample + ( dl_parse * LC_UF3_RECLEN ), LC_UF3_RECLEN );

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* reading input stream chunks */
        do {

            /* read input stream chunk */
            dl_istream.read( dl_chunks, DL_HASH_CHUNK * LC_UF3_RECLEN );

            /* parsing input stream chunk */
            for ( int64_t dl_parse( 0 ), dl_limit( dl_istream.gcount() ); dl_parse < dl_limit; dl_parse += LC_UF3_RECLEN ) {

                /* compute and assign array mapping */
                dl_posei = ( lc_uf3p_t * ) ( dl_chunks + dl_parse );

                /* parsing sample array */
                for ( int64_t dl_index( 0 ); dl_index < dl_count; dl_index ++ ) {

                    /* compute and assign array mapping */
                    dl_poses = ( lc_uf3p_t * ) ( dl_sample + ( dl_index * LC_UF3_RECLEN ) );

                    /* compute and check distance */
                    if ( ( dl_distance =

                        ( dl_posei[0] - dl_poses[0] ) * ( dl_posei[0] - dl_poses[0] ) +
                        ( dl_posei[1] - dl_poses[1] ) * ( dl_posei[1] - dl_poses[1] ) +
                        ( dl_posei[2] - dl_poses[2] ) * ( dl_posei[2] - dl_poses[2] )

                    ) < dl_values[dl_index] ) {

                        /* avoid identical point */
                        dl_values[dl_index] = dl_distance > 0.0 ? dl_distance : dl_values[dl_index];

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

        /* return minimal distance mean value */
        return( dl_distance / dl_count );

    }

/*
    source - hashing methods
 */

    void dl_hash( std::ifstream & dl_istream, char const * const dl_opath, double const dl_param, double const dl_mean ) {

        /* array mapping variables */
        lc_uf3p_t * dl_pose( nullptr );

        /* stream variables */
        std::ofstream dl_ostream;

        /* stream path variables */
        char dl_fpath[8192];

        /* buffer variables */
        char * dl_chunk( nullptr );

        /* hashing parameter variables */
        double dl_hash( dl_mean * dl_param );

        /* check consistency */
        if ( dl_opath == nullptr ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new ( std::nothrow ) char[DL_HASH_CHUNK * LC_UF3_RECLEN] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* parsing input stream chunks */
        do {

            /* read input stream chunk */
            dl_istream.read( dl_chunk, DL_HASH_CHUNK * LC_UF3_RECLEN );

            /* parsing input stream chunk */
            for ( int64_t dl_parse( 0 ), dl_limit( dl_istream.gcount() ); dl_parse < dl_limit; dl_parse += LC_UF3_RECLEN ) {

                /* compute and assign array mapping */
                dl_pose = ( lc_uf3p_t * ) ( dl_chunk + dl_parse );

                /* compute stream path */
                sprintf( dl_fpath, "%s/%+" PRId64 "_%+" PRId64 "_%+" PRId64 ".uf3", dl_opath, ( int64_t ) floor( dl_pose[0] / dl_hash ), ( int64_t ) floor( dl_pose[1] / dl_hash ), ( int64_t ) floor( dl_pose[2] / dl_hash ) );

                /* create output stream */
                dl_ostream.open( dl_fpath, std::ios::app | std::ios::out | std::ios::binary );

                /* check output stream */
                if ( dl_ostream.is_open() == true ) {

                    /* export chunk element */
                    dl_ostream.write( dl_chunk + dl_parse, LC_UF3_RECLEN );

                    /* delete output stream */
                    dl_ostream.close();

                /* send message */
                } else { throw( LC_ERROR_IO_ACCESS ); }

            }

        } while ( dl_istream.gcount() > 0 );

        /* release buffer memory */
        delete [] dl_chunk;

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* sampling count variables */
        int64_t dl_count( lc_read_signed( argc, argv, "--count", "-c", 64 ) );

        /* hashing parameter variables */
        double dl_param( lc_read_double( argc, argv, "--param", "-p", 250.0 ) );

        /* model mean value variables */
        double dl_mean( 0.0 );

        /* stream variables */
        std::ifstream dl_istream;

        /* error management */
        try {

            /* create input stream */
            dl_istream.open( lc_read_string( argc, argv, "--uf3", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

            /* check input stream */
            if ( dl_istream.is_open() == false ) {

                /* send message */
                throw( LC_ERROR_IO_ACCESS );

            }

            /* compute model mean value */
            dl_mean = dl_hash_stat( dl_istream, dl_istream.tellg(), dl_count );

            /* hash model */
            dl_hash( dl_istream, lc_read_string( argc, argv, "--output", "-o" ), dl_param, dl_mean );

            /* delete input stream */
            dl_istream.close();

        } catch ( int dl_code ) {

            /* switch on error code */
            switch ( dl_code ) {

                case ( LC_ERROR_MEMORY ) : {

                    /* display message */
                    std::cerr << "dalai-suite : error : memory allocation" << std::endl;

                } break;

                case ( LC_ERROR_IO_ACCESS ) : {

                    /* display message */
                    std::cerr << "dalai-suite : error : stream access" << std::endl;

                } break;

                case ( LC_ERROR_IO_READ ) : {

                    /* display message */
                    std::cerr << "dalai-suite : error : stream reading" << std::endl;

                } break;

            };

            /* send message */
            return( EXIT_FAILURE );

        } /* otherwise */ {

            /* send message */
            return( EXIT_SUCCESS );

        }

    }

