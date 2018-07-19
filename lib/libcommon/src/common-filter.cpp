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

    # include "common-filter.hpp"

/*
    source - filtering methods
 */

    le_void_t lc_filter_homogeneous( std::ifstream & lc_istream, std::ofstream & lc_ostream, le_real_t const lc_mean, le_real_t const lc_factor, le_size_t const lc_threshold ) {

        /* array mapping variables */
        le_real_t * lc_posea( nullptr );
        le_real_t * lc_poseb( nullptr );

        /* buffer variables */
        le_byte_t * lc_chunk( nullptr );
        le_size_t * lc_count( nullptr );

        /* stream size variables */
        le_size_t lc_size( 0 );

        /* indexation variables */
        le_size_t lc_delay( 0 );

        /* distance variables */
        le_real_t lc_distance( 0.0 );

        /* condition variables */
        le_real_t lc_condition( lc_mean * lc_mean * lc_factor * lc_factor );

        /* clear input stream */
        lc_istream.clear();

        /* set input stream offset */
        lc_istream.seekg( 0, std::ios::end );

        /* retrieve input stream size */
        lc_size = lc_istream.tellg();

        /* allocate and check buffer memory */
        if ( ( lc_chunk = new ( std::nothrow ) le_byte_t[lc_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* set input stream offset */
        lc_istream.seekg( 0, std::ios::beg );

        /* read input stream */
        lc_istream.read( ( char * ) lc_chunk, lc_size );

        /* allocate and check buffer memory */
        if ( ( lc_count = new ( std::nothrow ) le_size_t[lc_size / LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* initialise buffer */
        std::memset( ( char * ) lc_count, 0, ( lc_size / LE_UV3_RECORD ) * sizeof( le_size_t ) );

        /* parsing input stream elements */
        for ( le_size_t lc_parse( 0 ), lc_limit( lc_size - LE_UV3_RECORD ); lc_parse < lc_limit; lc_parse += LE_UV3_RECORD ) {

            /* compute and assign array mapping */
            lc_posea = ( le_real_t * ) ( lc_chunk + lc_parse );

            /* parsing input stream elements */
            for ( le_size_t lc_index( lc_parse + LE_UV3_RECORD ); lc_index < lc_size; lc_index += LE_UV3_RECORD ) {

                /* compute and assign array mapping */
                lc_poseb = ( le_real_t * ) ( lc_chunk + lc_index );

                /* compute element-element distance */
                lc_distance = ( lc_posea[0] - lc_poseb[0] ) * ( lc_posea[0] - lc_poseb[0] ) +
                              ( lc_posea[1] - lc_poseb[1] ) * ( lc_posea[1] - lc_poseb[1] ) +
                              ( lc_posea[2] - lc_poseb[2] ) * ( lc_posea[2] - lc_poseb[2] );

                /* check condition */
                if ( lc_distance < lc_condition ) {

                    /* update element count */
                    lc_count[lc_parse / LE_UV3_RECORD] ++;

                    /* update element count */
                    lc_count[lc_index / LE_UV3_RECORD] ++;

                }

            }

        }

        /* reset delayed indexation */
        lc_delay = 0;

        /* parsing count array */
        for ( le_size_t lc_parse( 0 ), lc_limit( lc_size / LE_UV3_RECORD ); lc_parse < lc_limit; lc_parse ++ ) {

            /* filtering condition */
            if ( lc_count[lc_parse] >= lc_threshold ) {

                /* delayed indexation */
                if ( lc_delay < lc_parse ) {

                    /* index filtered element */
                    std::memcpy( lc_chunk + ( lc_delay * LE_UV3_RECORD ), lc_chunk + ( lc_parse * LE_UV3_RECORD ), LE_UV3_RECORD );

                }

                /* update delay */
                lc_delay ++;

            }

        }

        /* exported filtered elements */
        lc_ostream.write( ( char * ) lc_chunk, lc_delay * LE_UV3_RECORD );

        /* release buffer memory */
        delete [] lc_chunk;
        delete [] lc_count;

    }

    le_void_t lc_filter_adaptative( std::ifstream & lc_istream, std::ofstream & lc_ostream, le_real_t const lc_factor, le_size_t const lc_threshold ) {

        /* buffer variables */
        le_byte_t * lc_chunk( nullptr );
        le_real_t * lc_dists( nullptr );
        le_size_t * lc_count( nullptr );

        /* array mapping variables */
        le_real_t * lc_pose1( nullptr );
        le_real_t * lc_pose2( nullptr );

        /* stream size variables */
        le_size_t lc_size( 0 );

        /* indexation variables */
        le_size_t lc_delay( 0 );

        /* distance variables */
        le_real_t lc_distance( 0.0 );

        /* condition variables */
        le_real_t lc_condition( 0.0 );

        /* clear input stream */
        lc_istream.clear();

        /* set input stream offset */
        lc_istream.seekg( 0, std::ios::end );

        /* retrieve input stream size */
        lc_size = lc_istream.tellg();

        /* allocate and check buffer memory */
        if ( ( lc_chunk = new ( std::nothrow ) le_byte_t[lc_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* set input stream offset */
        lc_istream.seekg( 0, std::ios::beg );

        /* read input stream */
        lc_istream.read( ( char * ) lc_chunk, lc_size );

        /* allocate and check buffer memory */
        if ( ( lc_count = new ( std::nothrow ) le_size_t[lc_size / LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate and check buffer memory */
        if ( ( lc_dists = new ( std::nothrow ) le_real_t[lc_size / LE_UV3_RECORD] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* initialise array values */
        for ( le_size_t lc_parse( 0 ), lc_limit( lc_size / LE_UV3_RECORD ); lc_parse < lc_limit; lc_parse ++ ) {

            /* assign initial value */
            lc_count[lc_parse] = 0;

            /* assign initial value */
            lc_dists[lc_parse] = std::numeric_limits<double>::max();

        }

        /* clear input stream */
        lc_istream.clear();

        /* reset input stream */
        lc_istream.seekg( 0 );

        /* read input stream */
        lc_istream.read( ( char * ) lc_chunk, lc_size );

        /* parsing input stream elements */
        for ( le_size_t lc_parse( 0 ), lc_limit( lc_size - LE_UV3_RECORD ); lc_parse < lc_limit; lc_parse += LE_UV3_RECORD ) {

            /* compute and assign array mapping */
            lc_pose1 = ( le_real_t * ) ( lc_chunk + lc_parse );

            /* parsing input stream elements */
            for ( le_size_t lc_index( lc_parse + LE_UV3_RECORD ); lc_index < lc_size; lc_index += LC_UF3_RECLEN ) {

                /* compute and assign array mapping */
                lc_pose2 = ( le_real_t  * ) ( lc_chunk + lc_index );

                /* compute element-element distance */
                lc_distance = ( lc_pose1[0] - lc_pose2[0] ) * ( lc_pose1[0] - lc_pose2[0] ) +
                              ( lc_pose1[1] - lc_pose2[1] ) * ( lc_pose1[1] - lc_pose2[1] ) +
                              ( lc_pose1[2] - lc_pose2[2] ) * ( lc_pose1[2] - lc_pose2[2] );

                /* search minimal distances */
                if ( lc_distance < lc_dists[lc_parse/LE_UV3_RECORD] ) lc_dists[lc_parse/LE_UV3_RECORD] = lc_distance;
                if ( lc_distance < lc_dists[lc_index/LE_UV3_RECORD] ) lc_dists[lc_index/LE_UV3_RECORD] = lc_distance;

            }

        }

        /* reset condition value */
        lc_condition = 0.0;

        /* prepare local threshold condition */
        for ( le_size_t lc_parse( 0 ), lc_limit( lc_size / LE_UV3_RECORD ); lc_parse < lc_limit; lc_parse ++ ) {

            /* accumulate local condition */
            lc_condition += sqrt( lc_dists[lc_parse] );

        }

        /* compute local threshold condition */
        lc_condition = lc_condition / ( lc_size / LE_UV3_RECORD );

        /* compute local threshold condition */
        lc_condition = lc_condition * lc_condition * lc_factor * lc_factor;

        /* parsing input stream elements */
        for ( le_size_t lc_parse( 0 ), lc_limit( lc_size - LE_UV3_RECORD ); lc_parse < lc_limit; lc_parse += LE_UV3_RECORD ) {

            /* compute and assign array mapping */
            lc_pose1 = ( le_real_t * ) ( lc_chunk + lc_parse );

            /* parsing input stream elements */
            for ( le_size_t lc_index( lc_parse + LE_UV3_RECORD ); lc_index < lc_size; lc_index += LE_UV3_RECORD ) {

                /* compute and assign array mapping */
                lc_pose2 = ( le_real_t * ) ( lc_chunk + lc_index );

                /* compute element-element distance */
                lc_distance = ( lc_pose1[0] - lc_pose2[0] ) * ( lc_pose1[0] - lc_pose2[0] ) +
                              ( lc_pose1[1] - lc_pose2[1] ) * ( lc_pose1[1] - lc_pose2[1] ) +
                              ( lc_pose1[2] - lc_pose2[2] ) * ( lc_pose1[2] - lc_pose2[2] );

                /* check condition */
                if ( lc_distance < lc_condition ) {

                    /* update element count */
                    lc_count[lc_parse/LE_UV3_RECORD] ++;

                    /* update element count */
                    lc_count[lc_index/LE_UV3_RECORD] ++;

                }

            }

        }

        /* reset delayed indexation */
        lc_delay = 0;

        /* parsing count array */
        for ( le_size_t lc_parse( 0 ), lc_limit( lc_size / LE_UV3_RECORD ); lc_parse < lc_limit; lc_parse ++ ) {

            /* filtering condition */
            if ( lc_count[lc_parse] >= lc_threshold ) {

                /* delayed indexation */
                if ( lc_delay < lc_parse ) {

                    /* index filtered element */
                    std::memcpy( lc_chunk + ( lc_delay * LE_UV3_RECORD ), lc_chunk + ( lc_parse * LE_UV3_RECORD ), LE_UV3_RECORD );

                }

                /* update delay */
                lc_delay ++;

            }

        }

        /* exported filtered elements */
        lc_ostream.write( ( char * ) lc_chunk, lc_delay * LE_UV3_RECORD );

        /* release buffer memory */
        delete [] lc_chunk;
        delete [] lc_dists;
        delete [] lc_count;

    }

