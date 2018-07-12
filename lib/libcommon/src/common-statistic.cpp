/*
 *  dalai-suite - common library
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

    # include "common-statistic.hpp"

/*
    source - statistical methods
 */

    double lc_statistic_mdmv( std::ifstream & lc_istream, int64_t const lc_count ) {

        /* array mapping variables */
        lc_uf3p_t * lc_posei( nullptr );
        lc_uf3p_t * lc_poses( nullptr );

        /* stream size variables */
        int64_t lc_size( 0 );

        /* distance variables */
        double lc_distance( 0.0 );

        /* buffer variables */
        char   * lc_sample( nullptr );
        char   * lc_chunks( nullptr );
        double * lc_values( nullptr );

        /* allocate and check buffer memory */
        if ( ( lc_sample = new ( std::nothrow ) char[lc_count * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate and check buffer memory */
        if ( ( lc_chunks = new ( std::nothrow ) char[LE_UV3_CHUNK * LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate and check buffer memory */
        if ( ( lc_values = new ( std::nothrow ) double[lc_count] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* initialise values array */
        for ( int64_t lc_parse( 0 ); lc_parse < lc_count; lc_parse ++ ) {

            /* assign initial value */
            lc_values[lc_parse] = std::numeric_limits<double>::max();

        }

        /* clear input stream */
        lc_istream.clear();

        /* set input stream offset */
        lc_istream.seekg( 0, std::ios::end );

        /* retrieve stream size */
        lc_size = lc_istream.tellg();

        /* set input stream offset */
        lc_istream.seekg( 0, std::ios::beg );

        /* create sample array */
        for ( int64_t lc_parse( 0 ); lc_parse < lc_count; lc_parse ++ ) {

            /* assign stream position */
            lc_istream.seekg( lc_parse * ( ( ( lc_size / LE_UV3_RECORD ) / lc_count ) * LE_UV3_RECORD ) );

            /* read sample values */
            lc_istream.read( lc_sample + ( lc_parse * LE_UV3_RECORD ), LE_UV3_RECORD );

        }

        /* clear input stream */
        lc_istream.clear();

        /* set input stream offset */
        lc_istream.seekg( 0, std::ios::beg );

        /* reading input stream chunks */
        do {

            /* read input stream chunk */
            lc_istream.read( lc_chunks, LE_UV3_CHUNK * LE_UV3_RECORD );

            /* parsing input stream chunk */
            for ( int64_t lc_parse( 0 ), lc_limit( lc_istream.gcount() ); lc_parse < lc_limit; lc_parse += LE_UV3_RECORD ) {

                /* compute and assign array mapping */
                lc_posei = ( le_real_t * ) ( lc_chunks + lc_parse );

                /* parsing sample array */
                for ( int64_t lc_index( 0 ); lc_index < lc_count; lc_index ++ ) {

                    /* compute and assign array mapping */
                    lc_poses = ( le_real_t * ) ( lc_sample + ( lc_index * LE_UV3_RECORD ) );

                    /* compute and check distance */
                    if ( ( lc_distance =

                        ( lc_posei[0] - lc_poses[0] ) * ( lc_posei[0] - lc_poses[0] ) +
                        ( lc_posei[1] - lc_poses[1] ) * ( lc_posei[1] - lc_poses[1] ) +
                        ( lc_posei[2] - lc_poses[2] ) * ( lc_posei[2] - lc_poses[2] )

                    ) < lc_values[lc_index] ) {

                        /* avoid identical point */
                        lc_values[lc_index] = lc_distance > 0.0 ? lc_distance : lc_values[lc_index];

                    }

                }

            }

        /* reading input stream chunks */
        } while ( lc_istream.gcount() > 0 );

        /* prepare statistical mean computation */
        lc_distance = 0.0;

        /* compute statistical mean value */
        for ( int64_t lc_parse( 0 ); lc_parse < lc_count; lc_parse ++ ) {

            /* accumulate distances */
            lc_distance += sqrt( lc_values[lc_parse] );

        }

        /* release buffer memory */
        delete [] lc_sample;
        delete [] lc_chunks;
        delete [] lc_values;

        /* return minimal distance mean value */
        return( lc_distance / double( lc_count ) );

    }

