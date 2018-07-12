/*
 *  dalai-suite - uf3-sort
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

    # include "dalai-sort.hpp"

/*
    source - common methods
 */

    le_size_t dl_sort_filesize( le_char_t const * const dl_path ) {

        /* statistics structure */
        struct stat dl_stat = { 0 };

        /* retrieve statistics */
        if ( stat( ( char * ) dl_path, & dl_stat ) != 0 ) {

            /* send message */
            return( 0 );

        }

        /* return size */
        return( dl_stat.st_size );

    }

    le_void_t dl_sort_copy( le_char_t const * const dl_ipath, le_char_t const * const dl_opath ) {

        /* buffer variable */
        le_byte_t * dl_buffer( nullptr );

        /* stream variable */
        std::fstream dl_istream;

        /* stream variable */
        std::fstream dl_ostream;

        /* read variable */
        le_size_t dl_read( 1 );

        /* allocate buffer memory */
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[DL_SORT_BUFFER] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* create input stream */
        dl_istream.open( ( char * ) dl_ipath, std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* create output stream */
        dl_ostream.open( ( char * ) dl_opath, std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_WRITE );

        }

        /* copy loop */
        do {

            /* read buffer */
            dl_istream.read( ( char * ) dl_buffer, DL_SORT_BUFFER );

            /* read byte count */
            dl_read = dl_istream.gcount();

            /*  write buffer */
            dl_ostream.write( ( char * ) dl_buffer, dl_read );

        } while ( dl_read > 0 );

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

        /* release buffer memory */
        delete [] dl_buffer;

    }

/*
    source - dispatch methods
 */

    le_size_t dl_sort_dispatch( le_char_t const * const dl_path, le_size_t const dl_size, le_byte_t const dl_depth, le_char_t const * const dl_temp ) {

        /* stream variable */
        std::fstream dl_istream;

        /* stream variable */
        std::fstream dl_ostream;

        /* buffer variable */
        le_byte_t * dl_buffer( nullptr );

        /* head variable */
        le_size_t dl_head( 0 );

        /* reading variable */
        le_size_t dl_read( 0 );

        /* dispatch variable */
        le_size_t dl_segment( 0 );

        /* string variable */
        le_char_t dl_tfile[_LE_USE_PATH] = { 0 };

        /* allocate buffer memory */
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[DL_SORT_CHUNK] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* create input stream */
        dl_istream.open( ( char * ) dl_path, std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* reading input stream */
        while ( dl_head < dl_size ) {

            /* read stream chunk */
            dl_istream.read( ( char * ) dl_buffer, DL_SORT_CHUNK );

            /* update head and chunk size */
            dl_head += ( dl_read = dl_istream.gcount() );

            /* sort chunk buffer */
            dl_buffer = dl_sort_algorithm_memory( dl_buffer, dl_read, dl_depth );

            /* compose chunk path */
            sprintf( ( char * ) dl_tfile, DL_SORT_ORIGIN, dl_temp, dl_segment ++ );

            /* create output stream */
            dl_ostream.open( ( char * ) dl_tfile, std::ios::out | std::ios::binary );

            /* check output stream */
            if ( dl_ostream.is_open() == false ) {

                /* send message */
                throw( LC_ERROR_IO_ACCESS );

            }

            /* export sorted chunk */
            dl_ostream.write( ( char * ) dl_buffer, dl_read );

            /* delete output stream */
            dl_ostream.close();

        }

        /* delete input stream */
        dl_istream.close();

        /* release buffer memory */
        delete [] dl_buffer;

        /* return counter */
        return( dl_segment );

    }

/*
    source - sorting methods
 */

    void dl_sort_memory( le_char_t const * const dl_ipath, le_char_t const * const dl_opath, le_size_t const dl_size, le_byte_t const dl_depth ) {

        /* buffer variable */
        le_byte_t * dl_buffer( nullptr );

        /* stream variable */
        std::fstream dl_stream;

        /* allocate buffer memory */
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[dl_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* create input stream */
        dl_stream.open( ( char * ) dl_ipath, std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_stream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* import stream bytes */
        dl_stream.read( ( char * ) dl_buffer, dl_size );

        /* delete input stream */
        dl_stream.close();

        /* sorting buffer */
        dl_buffer = dl_sort_algorithm_memory( dl_buffer, dl_size, dl_depth );

        /* create ouptut stream */
        dl_stream.open( ( char * ) dl_opath, std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_stream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* export stream bytes */
        dl_stream.write( ( char * ) dl_buffer, dl_size );

        /* delete output stream */
        dl_stream.close();

        /* release buffer memory */
        delete [] dl_buffer;

    }

    void dl_sort_disk( le_char_t const * const dl_ipath, le_char_t const * const dl_opath, le_size_t const dl_size, le_byte_t const dl_depth, le_char_t const * const dl_temp ) {

        /* path variable */
        le_char_t dl_ffile[_LE_USE_PATH] = { 0 };

        /* path variable */
        le_char_t dl_sfile[_LE_USE_PATH] = { 0 };

        /* path variable */
        le_char_t dl_ofile[_LE_USE_PATH] = { 0 };

        /* count variable */
        le_size_t dl_segment( 0 );

        /* parsing variable */
        le_size_t dl_parse( 0 );

        /* size variable */
        le_size_t dl_flength;

        /* size variable */
        le_size_t dl_slength;
        
        /* dispatch stream chunks */
        dl_segment = dl_sort_dispatch( dl_ipath, dl_size, dl_depth, dl_temp );

        /* stream chunks merge */
        while ( dl_segment > 1 ) {

            /* reset parser */
            dl_parse = 0;

            /* merge chunks */
            while ( dl_parse < dl_segment ) {

                /* compose chunk path */
                sprintf( ( char * ) dl_ffile, DL_SORT_ORIGIN, dl_temp, dl_parse );

                /* compose chunk path */
                sprintf( ( char * ) dl_ofile, DL_SORT_TARGET, dl_temp, dl_parse >> 1 );

                /* check remaining odd-chunk */
                if ( ( dl_parse + 1 ) == dl_segment ) {

                    /* chunk direct copy */
                    rename( ( char * ) dl_ffile, ( char * ) dl_ofile );

                } else {

                    /* compose chunk path */
                    sprintf( ( char * ) dl_sfile, DL_SORT_ORIGIN, dl_temp, dl_parse + 1 );

                    /* retrieve chunk size */
                    dl_flength = dl_sort_filesize( dl_ffile );

                    /* retrieve chunk size */
                    dl_slength = dl_sort_filesize( dl_sfile );

                    /* disk-based merge sort */
                    dl_sort_algorithm_disk( dl_ffile, dl_flength, dl_sfile, dl_slength, dl_ofile, dl_depth );

                    /* remove origin chunk */
                    remove( ( char * ) dl_ffile );

                    /* remove origin chunk */
                    remove( ( char * ) dl_sfile );

                }

                /* update parser */
                dl_parse += 2;

            }

            /* update counter */
            dl_segment = ( dl_segment >> 1 ) + ( dl_segment % 2 );

            /* reset parser */
            dl_parse = 0;

            /* parsing files */
            while ( dl_parse < dl_segment ) {

                /* compose chunk path */
                sprintf( ( char * ) dl_ffile, DL_SORT_TARGET, dl_temp, dl_parse );

                /* compose chunk path */
                sprintf( ( char * ) dl_ofile, DL_SORT_ORIGIN, dl_temp, dl_parse );

                /* target chunk to origin chunk */
                rename( ( char * ) dl_ffile, ( char * ) dl_ofile );

                /* update parser */
                dl_parse ++;

            }

        }

        /* check existing output file */
        if ( le_get_exist( dl_opath ) == _LE_TRUE ) {

            /* remove existing output file */
            remove( ( char * ) dl_opath );

        }

        /* compose final chunk path */
        sprintf( ( char * ) dl_ofile, DL_SORT_ORIGIN, dl_temp, _LE_SIZE_L( 0 ) );

        /* copy final chunk to output file */
        if ( rename( ( char * ) dl_ofile, ( char * ) dl_opath ) != 0 ) {

            /* formal copy */
            dl_sort_copy( dl_ofile, dl_opath );

            /* remove source */
            remove( ( char * ) dl_ofile );
    
        }

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* path variable */
        le_char_t * dl_ipath( ( le_char_t * ) lc_read_string( argc, argv, "--input", "-i" ) );

        /* path variable */
        le_char_t * dl_opath( ( le_char_t * ) lc_read_string( argc, argv, "--output", "-o" ) );

        /* path variable */
        le_char_t * dl_tpath( ( le_char_t * ) lc_read_string( argc, argv, "--temporary", "-t" ) );

        /* index length variable */
        le_byte_t dl_depth( lc_read_unsigned( argc, argv, "--depth", "-d", 0 ) );

        /* stream size variable */
        le_size_t dl_size( 0 );

    /* error bloc */
    try {

        /* check consistency */
        if ( ( dl_depth == 0 ) || ( dl_depth >= _LE_USE_DEPTH ) ) {

            /* send message */
            throw( LC_ERROR_DOMAIN );

        }

        /* check consistency */
        if ( ( dl_size = dl_sort_filesize( dl_ipath ) ) == 0 ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* select sorting algorithm */
        if ( dl_size > ( le_size_t ) DL_SORT_CHUNK ) {

            /* check consistency */
            if ( le_get_exist( dl_tpath ) == _LE_FALSE ) {

                /* send message */
                throw( LC_ERROR_IO_ACCESS );

            }

            /* disk-based sorting process */
            dl_sort_disk( dl_ipath, dl_opath, dl_size, dl_depth, dl_tpath );

        } else {

            /* memory-based sorting process */
            dl_sort_memory( dl_ipath, dl_opath, dl_size, dl_depth );   

        }

    /* error bloc */
    } catch ( int dl_code ) {

        /* error management */
        lc_error( dl_code );

        /* send message */
        return( EXIT_FAILURE );

    }

        /* send message */
        return( EXIT_SUCCESS );

    }

