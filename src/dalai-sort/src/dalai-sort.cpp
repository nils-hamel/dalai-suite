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
    source - comparison methods
 */

    bool dl_sort_function( le_real_t const * const dl_pose_a, le_real_t const * const dl_pose_b, le_byte_t const dl_length ) {

        /* position variable */
        le_real_t dl_pa[3] = {

        ( dl_pose_a[0] - LE_ADDRESS_MIN_L ) * LE_ADDRESS_IRN_L,
        ( dl_pose_a[1] - LE_ADDRESS_MIN_A ) * LE_ADDRESS_IRN_A,
        ( dl_pose_a[2] - LE_ADDRESS_MIN_H ) * LE_ADDRESS_IRN_H,

        };

        /* position variable */
        le_real_t dl_pb[3] = {

        ( dl_pose_b[0] - LE_ADDRESS_MIN_L ) * LE_ADDRESS_IRN_L,
        ( dl_pose_b[1] - LE_ADDRESS_MIN_A ) * LE_ADDRESS_IRN_A,
        ( dl_pose_b[2] - LE_ADDRESS_MIN_H ) * LE_ADDRESS_IRN_H,

        };

        /* digit variable */
        le_byte_t dl_da( 0 );
        le_byte_t dl_db( 0 );

        /* comparison function */
        for ( le_size_t dl_parse( 0 ); dl_parse < dl_length; dl_parse ++ ) {

            /* check digit value */
            if ( ( dl_pa[0] *= 2.0 ) >= 1.0 ) {

                /* assign digit component */
                dl_da = 0x01;

                /* update dimension value */
                dl_pa[0] -= 1.0;

            /* initialise digit */
            } else { dl_da = 0x00; }

            /* check digit value */
            if ( ( dl_pb[0] *= 2.0 ) >= 1.0 ) {

                /* assign digit component */
                dl_db = 0x01;

                /* update dimension value */
                dl_pb[0] -= 1.0;

            /* initialise digit */
            } else { dl_db = 0x00; }

            /* asynchronous dimension */
            if ( dl_parse >= LE_ADDRESS_DEPTH_P ) {

                /* check digit value */
                if ( ( dl_pa[1] *= 2.0 ) >= 1.0 ) {

                    /* assign digit component */
                    dl_da |= 0x02;

                    /* update dimension value */
                    dl_pa[1] -= 1.0;

                }

                /* check digit value */
                if ( ( dl_pb[1] *= 2.0 ) >= 1.0 ) {

                    /* assign digit component */
                    dl_db |= 0x02;

                    /* update dimension value */
                    dl_pb[1] -= 1.0;

                }

                /* asynchronous dimension */
                if ( dl_parse >= LE_ADDRESS_DEPTH_A ) {

                    /* check digit value */
                    if ( ( dl_pa[2] *= 2.0 ) >= 1.0 ) {

                        /* assign digit component */
                        dl_da |= 0x04;

                        /* update dimension value */
                        dl_pa[2] -= 1.0;

                    }

                    /* check digit value */
                    if ( ( dl_pb[2] *= 2.0 ) >= 1.0 ) {

                        /* assign digit component */
                        dl_db |= 0x04;

                        /* update dimension value */
                        dl_pb[2] -= 1.0;

                    }

                }

            }

            /* compare digit */
            if ( dl_da > dl_db ) {

                /* send result */
                return( true );

            } else {

                /* compare digit */
                if ( dl_da < dl_db ) {

                    /* send result */
                    return( false );

                }

            }

        }

        /* send result */
        return( false );

    }

/*
    source - sorting methods
 */

    le_byte_t * dl_sort( le_byte_t * const dl_buffer, le_size_t const dl_size, le_byte_t const dl_length ) {

        /* buffer variable */
        le_byte_t * dl_stack[2] = { dl_buffer, nullptr };

        /* buffer switch variable */
        le_size_t dl_head( 0 );
        le_size_t dl_dual( 1 );

        /* merge range variable */
        le_size_t dl_rahead( 0 );
        le_size_t dl_raedge( 0 );
        le_size_t dl_rbhead( 0 );
        le_size_t dl_rbedge( 0 );

        /* merge index variable */
        le_size_t dl_index( 0 );

        /* merge step variable */
        le_size_t dl_step( LC_UF3_RECLEN );

        /* allocate dual buffer memory */
        if ( ( dl_stack[dl_dual] = new ( std::nothrow ) le_byte_t[dl_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* merge sort main loop */
        while ( dl_step < dl_size ) {

            /* reset merge index */
            dl_index = 0;

            /* reset merge range */
            dl_rahead = 0;

            /* parsing merge ranges */
            while ( dl_rahead < dl_size ) {

                /* compute merge range */
                dl_raedge = dl_rahead + dl_step;
                dl_rbhead = dl_raedge;
                dl_rbedge = dl_rbhead + dl_step;

                /* check range edge */
                if ( dl_raedge > dl_size ) {

                    /* edge correction */
                    dl_raedge = dl_size;

                }

                /* check range edge */
                if ( dl_rbedge > dl_size ) {

                    /* edge correction */
                    dl_rbedge = dl_size;

                }
                
                /* ranges merge */
                while ( ( dl_rahead < dl_raedge ) || ( dl_rbhead < dl_rbedge ) ) {

                    /* merge condition */
                    if ( dl_rahead >= dl_raedge ) {

                        /* record selection */
                        std::memcpy( dl_stack[dl_dual] + dl_index, dl_stack[dl_head] + dl_rbhead, LC_UF3_RECLEN );

                        /* update merge head */
                        dl_rbhead += LC_UF3_RECLEN;

                    } else {

                        /* merge condition */
                        if ( dl_rbhead >= dl_rbedge ) {

                            /* record selection */
                            std::memcpy( dl_stack[dl_dual] + dl_index, dl_stack[dl_head] + dl_rahead, LC_UF3_RECLEN );

                            /* update merge head */
                            dl_rahead += LC_UF3_RECLEN;

                        } else {

                            /* merge condition */
                            if ( dl_sort_function( ( le_real_t * ) ( dl_stack[dl_head] + dl_rahead ), ( le_real_t * ) ( dl_stack[dl_head] + dl_rbhead ), dl_length ) == true ) {

                                /* record selection */
                                std::memcpy( dl_stack[dl_dual] + dl_index, dl_stack[dl_head] + dl_rbhead, LC_UF3_RECLEN );

                                /* update merge head */
                                dl_rbhead += LC_UF3_RECLEN;

                            } else {

                                /* record selection */
                                std::memcpy( dl_stack[dl_dual] + dl_index, dl_stack[dl_head] + dl_rahead, LC_UF3_RECLEN );

                                /* update merge head */
                                dl_rahead += LC_UF3_RECLEN;

                            }

                        }

                    }

                    /* update merge index */
                    dl_index += LC_UF3_RECLEN;

                }

                /* push next range */
                dl_rahead = dl_rbedge;

            }

            /* update merge step */
            dl_step <<= 1;

            /* swap buffer switch */
            dl_head = 1 - dl_head;
            dl_dual = 1 - dl_dual;

        }

        /* check buffer switch */
        if ( dl_head == 1 ) {

            /* release dual buffer memory */
            delete [] dl_stack[0];

            /* return sorted buffer */
            return( dl_stack[1] );

        } else {

            /* release dual buffer memory */
            delete [] dl_stack[1];

            /* return sorted buffer */
            return( dl_stack[0] );

        }

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream variable */
        std::fstream dl_stream;

        /* stream size variable */
        le_size_t dl_size( 0 );

        /* stream buffer */
        le_byte_t * dl_buffer( nullptr );

        /* index size variable */
        le_byte_t dl_length( lc_read_unsigned( argc, argv, "--index", "-x", 0 ) );

    /* error management */
    try {

        /* create input stream */
        dl_stream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( dl_stream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* extract file size */
        dl_size = dl_stream.tellg();

        /* allocate memory */
        if ( ( dl_buffer = new ( std::nothrow ) le_byte_t[dl_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* rewind stream */
        dl_stream.seekg( 0, std::ios::beg );

        /* read stream buffer */
        dl_stream.read( ( char * ) dl_buffer, dl_size );

        /* delete stream */
        dl_stream.close();

        /* sorting buffer */
        dl_buffer = dl_sort( dl_buffer, dl_size, dl_length );

        /* create output stream */
        dl_stream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_stream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* write stream buffer */
        dl_stream.write( ( char * ) dl_buffer, dl_size );

        /* close output stream */
        dl_stream.close();

        /* release buffer memory */
        delete [] dl_buffer;

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

