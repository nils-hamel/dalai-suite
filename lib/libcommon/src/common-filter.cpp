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

    le_void_t lc_filter_homogeneous( le_char_t const * const lc_ipath, std::ofstream & lc_ostream, le_real_t const lc_mean, le_real_t const lc_factor, le_size_t const lc_threshold ) {

        /* stream variable */
        std::fstream lc_istream;

        /* condition variable */
        le_real_t lc_select( lc_mean * lc_mean * lc_factor * lc_factor );

        /* size variable */
        le_size_t lc_size( 0 );

        /* index variable */
        le_size_t lc_delay( 0 );

        /* buffer variable */
        le_byte_t * lc_buffer( nullptr );

        /* buffer variable */
        le_size_t * lc_count( nullptr );

        /* buffer pointer variable */
        le_real_t * lc_uv3p( nullptr );

        /* buffer pointer variable */
        le_real_t * lc_uv3s( nullptr );

        /* create input stream */
        lc_istream.open( ( char * ) lc_ipath, std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( lc_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* retrieve input stream size */
        lc_size = lc_istream.tellg();

        /* allocate buffer memory */
        if ( ( lc_count = new ( std::nothrow ) le_size_t[lc_size / LE_ARRAY_DATA] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* initialise buffer */
        std::memset( ( char * ) lc_count, 0, ( lc_size / LE_ARRAY_DATA ) * sizeof( le_size_t ) );

        /* allocate buffer memory */
        if ( ( lc_buffer = new ( std::nothrow ) le_byte_t[lc_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* input stream offset at begining */
        lc_istream.seekg( 0, std::ios::beg );

        /* read stream content */
        lc_istream.read( ( char * ) lc_buffer, lc_size );

        /* check read buffer */
        if ( lc_istream.gcount() != lc_size ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* parsing stream element */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_size; lc_parse += LE_ARRAY_DATA ) {

            /* compute buffer pointer */
            lc_uv3p = ( le_real_t * ) ( lc_buffer + lc_parse );

            /* parsing stream element */
            for ( le_size_t lc_index( lc_parse + LE_ARRAY_DATA ); lc_index < lc_size; lc_index += LE_ARRAY_DATA ) {

                /* compute buffer pointer */
                lc_uv3s = ( le_real_t * ) ( lc_buffer + lc_index );

                /* compute and check condition */
                if ( lc_geometry_squaredist( lc_uv3p, lc_uv3s ) < lc_select ) {

                    /* update count */
                    lc_count[lc_parse/LE_ARRAY_DATA] ++;

                    /* update count */
                    lc_count[lc_index/LE_ARRAY_DATA] ++;

                }

            }

        }

        /* parsing count array */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_size; lc_parse += LE_ARRAY_DATA ) {

            /* check condition */
            if ( lc_count[lc_parse/LE_ARRAY_DATA] >= lc_threshold ) {

                /* check index delay */
                if ( lc_delay < lc_parse ) {

                    /* record selection */
                    std::memcpy( lc_buffer + lc_delay, lc_buffer + lc_parse, LE_ARRAY_DATA );

                }

                /* update index */
                lc_delay += LE_ARRAY_DATA;

            }

        }

        /* export filtered stream */
        lc_ostream.write( ( char * ) lc_buffer, lc_delay );

        /* release buffer memory */
        delete [] lc_buffer;

        /* release buffer memory */
        delete [] lc_count;

        /* delete input stream */
        lc_istream.close();

    }

    le_void_t lc_filter_adaptative( le_char_t const * const lc_ipath, std::ofstream & lc_ostream, le_real_t const lc_factor, le_size_t const lc_threshold ) {

        /* stream variable */
        std::fstream lc_istream;

        /* condition variable */
        le_real_t lc_select( 0.0 );

        /* distance variable */
        le_real_t lc_distance( 0.0 );

        /* size variable */
        le_size_t lc_size( 0 );

        /* size variable */
        le_size_t lc_real( 0 );

        /* index variable */
        le_size_t lc_delay( 0 );

        /* buffer variable */
        le_byte_t * lc_buffer( nullptr );

        /* buffer variable */
        le_size_t * lc_count( nullptr );

        /* buffer variable */
        le_real_t * lc_mean( nullptr );

        /* buffer pointer variable */
        le_real_t * lc_uv3p( nullptr );

        /* buffer pointer variable */
        le_real_t * lc_uv3s( nullptr );

        /* create input stream */
        lc_istream.open( ( char * ) lc_ipath, std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( lc_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* retrieve input stream size */
        lc_real = ( lc_size = lc_istream.tellg() ) / LE_ARRAY_DATA;

        /* allocate buffer memory */
        if ( ( lc_count = new ( std::nothrow ) le_size_t[lc_real] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( lc_mean = new ( std::nothrow ) le_real_t[lc_real] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* parsing array */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_real; lc_parse ++ ) {

            /* reset count value */
            lc_count[lc_parse] = 0;

            /* reset mean value */
            lc_mean[lc_parse] = std::numeric_limits<double>::max();

        }

        /* allocate buffer memory */
        if ( ( lc_buffer = new ( std::nothrow ) le_byte_t[lc_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* input stream offset at begining */
        lc_istream.seekg( 0, std::ios::beg );

        /* read stream content */
        lc_istream.read( ( char * ) lc_buffer, lc_size );

        /* check read buffer */
        if ( lc_istream.gcount() != lc_size ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* parsing stream element */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_size; lc_parse += LE_ARRAY_DATA ) {

            /* compute buffer pointer */
            lc_uv3p = ( le_real_t * ) ( lc_buffer + lc_parse );

            /* parsing stream element */
            for ( le_size_t lc_index( lc_parse + LE_ARRAY_DATA ); lc_index < lc_size; lc_index += LE_ARRAY_DATA ) {

                /* compute buffer pointer */
                lc_uv3s = ( le_real_t * ) ( lc_buffer + lc_index );

                /* compute distance */
                lc_distance = lc_geometry_squaredist( lc_uv3p, lc_uv3s );

                /* check distance */
                if ( lc_distance < lc_mean[lc_parse/LE_ARRAY_DATA] ) {

                    /* update distance */
                    lc_mean[lc_parse/LE_ARRAY_DATA] = lc_distance;

                }

                /* check distance */
                if ( lc_distance < lc_mean[lc_index/LE_ARRAY_DATA] ) {

                    /* update distance */
                    lc_mean[lc_index/LE_ARRAY_DATA] = lc_distance;

                }

            }

        }

        /* parsing array */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_real; lc_parse ++ ) {

            /* mean value accumulation */
            lc_select += std::sqrt( lc_mean[lc_parse] );

        }

        /* compute mean value */
        lc_select = ( lc_select / le_real_t( lc_real ) );

        /* compute selection condition */
        lc_select = lc_select * lc_select * lc_factor * lc_factor;

        /* parsing stream element */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_size; lc_parse += LE_ARRAY_DATA ) {

            /* compute buffer pointer */
            lc_uv3p = ( le_real_t * ) ( lc_buffer + lc_parse );

            /* parsing stream element */
            for ( le_size_t lc_index( lc_parse + LE_ARRAY_DATA ); lc_index < lc_size; lc_index += LE_ARRAY_DATA ) {

                /* compute buffer pointer */
                lc_uv3s = ( le_real_t * ) ( lc_buffer + lc_index );

                /* compute and check condition */
                if ( lc_geometry_squaredist( lc_uv3p, lc_uv3s ) < lc_select ) {

                    /* update count */
                    lc_count[lc_parse/LE_ARRAY_DATA] ++;

                    /* update count */
                    lc_count[lc_index/LE_ARRAY_DATA] ++;

                }

            }

        }

        /* parsing count array */
        for ( le_size_t lc_parse( 0 ); lc_parse < lc_size; lc_parse += LE_ARRAY_DATA ) {

            /* check condition */
            if ( lc_count[lc_parse/LE_ARRAY_DATA] >= lc_threshold ) {

                /* check index delay */
                if ( lc_delay < lc_parse ) {

                    /* record selection */
                    std::memcpy( lc_buffer + lc_delay, lc_buffer + lc_parse, LE_ARRAY_DATA );

                }

                /* update index */
                lc_delay += LE_ARRAY_DATA;

            }

        }

        /* export filtered stream */
        lc_ostream.write( ( char * ) lc_buffer, lc_delay );

        /* release buffer memory */
        delete [] lc_buffer;

        /* release buffer memory */
        delete [] lc_mean;

        /* release buffer memory */
        delete [] lc_count;

        /* delete input stream */
        lc_istream.close();

    }

