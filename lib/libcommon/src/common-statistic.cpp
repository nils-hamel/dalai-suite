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

    le_real_t lc_statistic_mdmv( std::ifstream & lc_istream, le_size_t const lc_count ) {

        /* value variable */
        le_real_t * lc_value( nullptr );

        /* buffer variable */
        le_byte_t * lc_sample( nullptr );

        /* buffer variable */
        le_byte_t * lc_buffer( nullptr );

        /* reading variable */
        le_size_t lc_read( 1 );

        /* size variable */
        le_size_t lc_size( 0 );

        /* distance variable */
        le_real_t lc_distance( 0.0 );

        /* buffer pointer variable */
        le_real_t * lc_uv3p( nullptr );

        /* buffer pointer variable */
        le_real_t * lc_uv3s( nullptr );

        /* reset stream */
        lc_istream.clear();

        /* stream offset to end */
        lc_istream.seekg( 0, std::ios::end );

        /* retrieve stream size */
        lc_size = lc_istream.tellg();

        /* allocate buffer memory */
        if ( ( lc_buffer = new ( std::nothrow ) le_byte_t[LE_ARRAY_DATA * LE_UV3_CHUNK] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( lc_sample = new ( std::nothrow ) le_byte_t[LE_ARRAY_DATA * lc_count] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( lc_value = new ( std::nothrow ) le_real_t[lc_count] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* initialise arrays */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_count; lc_parse ++ ) {

            /* initialise value array */
            lc_value[lc_parse] = std::numeric_limits<le_real_t>::max();

            /* select homogeneous stream record */
            lc_istream.seekg( ( ( lc_size / LE_ARRAY_DATA ) / lc_count ) * LE_ARRAY_DATA * lc_parse );

            /* import selected stream record */
            lc_istream.read( ( char * ) ( lc_sample + lc_parse * LE_ARRAY_DATA ), LE_ARRAY_DATA );

        }

        /* reset stream */
        lc_istream.clear();

        /* stream offset to begining */
        lc_istream.seekg( 0, std::ios::beg );

        /* stream chunk reading */
        while ( lc_read > 0 ) {

            /* read stream chunk */
            lc_istream.read( ( char * ) lc_buffer, LE_ARRAY_DATA * LE_UV3_CHUNK );

            /* read byte count */
            lc_read = lc_istream.gcount();

            /* parsing chunk */
            for ( le_size_t lc_parse( 0 ); lc_parse < lc_read; lc_parse += LE_ARRAY_DATA ) {

                /* create buffer pointer */
                lc_uv3p = ( le_real_t * ) ( lc_buffer + lc_parse );

                /* parsing sample array */
                for ( le_size_t lc_index( 0 ); lc_index < lc_count; lc_index ++ ) {

                    /* create buffer pointer */
                    lc_uv3s = ( le_real_t * ) ( lc_sample + lc_index * LE_ARRAY_DATA );

                    /* compute and check distance */
                    if ( ( lc_distance = lc_geometry_squaredist( lc_uv3p, lc_uv3s ) ) < lc_value[lc_index] ) {

                        /* avoid matching element */
                        if ( lc_distance > 0.0 ) {

                            /* update minimal distance */
                            lc_value[lc_index] = lc_distance;

                        }

                    }

                }

            }

        }

        /* reset mean value */
        lc_distance = 0.0;

        /* parsing value array */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_count; lc_parse ++ ) {

            /* accumulate minimal distance */
            lc_distance += std::sqrt( lc_value[lc_parse] );

        }

        /* release buffer memory */
        delete [] lc_value;

        /* release buffer memory */
        delete [] lc_sample;

        /* release buffer memory */
        delete [] lc_buffer;

        /* compute and return minimal distance mean */
        return( lc_distance / le_real_t( lc_count ) );

    }

