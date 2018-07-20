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

    # include "dalai-sort-algorithm.hpp"

/*
    source - comparison methods
 */

    bool dl_sort_algorithm( le_real_t const * const dl_fpose, le_real_t const * const dl_spose, le_byte_t const dl_depth ) {

        /* coordinates variable */
        le_real_t dl_fnormal[3] = { DL_SORT_X( dl_fpose ), DL_SORT_Y( dl_fpose ), DL_SORT_Z( dl_fpose ) };

        /* coordinates variable */
        le_real_t dl_snormal[3] = { DL_SORT_X( dl_spose ), DL_SORT_Y( dl_spose ), DL_SORT_Z( dl_spose ) };

        /* digit variable */
        le_byte_t dl_fdigit( 0 );

        /* digit variable */
        le_byte_t dl_sdigit( 0 );

        /* comparison process */
        for ( le_size_t dl_parse( 0 ); dl_parse < dl_depth; dl_parse ++ ) {

            /* check digit value */
            if ( ( dl_fnormal[0] *= 2.0 ) >= 1.0 ) {

                /* assign digit component */
                dl_fdigit = 0x01;

                /* update dimension value */
                dl_fnormal[0] -= 1.0;

            /* reset digit */
            } else { dl_fdigit = 0x00; }

            /* check digit value */
            if ( ( dl_snormal[0] *= 2.0 ) >= 1.0 ) {

                /* assign digit component */
                dl_sdigit = 0x01;

                /* update dimension value */
                dl_snormal[0] -= 1.0;

            /* reset digit */
            } else { dl_sdigit = 0x00; }

            /* asynchronous dimension */
            if ( dl_parse >= LE_ADDRESS_DEPTH_P ) {

                /* check digit value */
                if ( ( dl_fnormal[1] *= 2.0 ) >= 1.0 ) {

                    /* assign digit component */
                    dl_fdigit |= 0x02;

                    /* update dimension value */
                    dl_fnormal[1] -= 1.0;

                }

                /* check digit value */
                if ( ( dl_snormal[1] *= 2.0 ) >= 1.0 ) {

                    /* assign digit component */
                    dl_sdigit |= 0x02;

                    /* update dimension value */
                    dl_snormal[1] -= 1.0;

                }

                /* asynchronous dimension */
                if ( dl_parse >= LE_ADDRESS_DEPTH_A ) {

                    /* check digit value */
                    if ( ( dl_fnormal[2] *= 2.0 ) >= 1.0 ) {

                        /* assign digit component */
                        dl_fdigit |= 0x04;

                        /* update dimension value */
                        dl_fnormal[2] -= 1.0;

                    }

                    /* check digit value */
                    if ( ( dl_snormal[2] *= 2.0 ) >= 1.0 ) {

                        /* assign digit component */
                        dl_sdigit |= 0x04;

                        /* update dimension value */
                        dl_snormal[2] -= 1.0;

                    }

                }

            }

            /* compare digit */
            if ( dl_fdigit > dl_sdigit ) {

                /* send result */
                return( true );

            } else if ( dl_fdigit < dl_sdigit ) {

                /* send result */
                return( false );

            }

        }

        /* send result */
        return( false );

    }

/*
    source - sorting methods
 */

    le_byte_t * dl_sort_algorithm_memory( le_byte_t * const dl_buffer, le_size_t const dl_size, le_byte_t const dl_depth ) {

        /* buffer variable */
        le_byte_t * dl_pair[2] = { dl_buffer, nullptr };

        /* switch variable */
        le_size_t dl_origin( 0 );
        le_size_t dl_target( 1 );

        /* range variable */
        le_size_t dl_fhead( 0 );
        le_size_t dl_fedge( 0 );

        /* range variable */
        le_size_t dl_shead( 0 );
        le_size_t dl_sedge( 0 );

        /* index variable */
        le_size_t dl_index( 0 );

        /* step variable */
        le_size_t dl_step( LE_UV3_RECORD );

        /* allocate buffer memory */
        if ( ( dl_pair[dl_target] = new ( std::nothrow ) le_byte_t[dl_size] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* merge sort dichotomous loop */
        while ( dl_step < dl_size ) {

            /* reset index */
            dl_index = 0;

            /* ranges merging */
            while ( dl_index < dl_size ) {

                /* compute range */
                dl_fhead = dl_index;
                dl_fedge = dl_fhead + dl_step;

                /* compute range */
                dl_shead = dl_fedge;
                dl_sedge = dl_shead + dl_step;

                /* check range edge */
                if ( dl_fedge > dl_size ) {

                    /* edge correction */
                    dl_fedge = dl_size;

                }

                /* check range edge */
                if ( dl_sedge > dl_size ) {

                    /* edge correction */
                    dl_sedge = dl_size;

                }
                
                /* range merge */
                while ( dl_index < dl_sedge ) {

                    /* empty stack condition */
                    if ( dl_fhead >= dl_fedge ) {

                        /* copy remaining records */
                        std::memcpy( dl_pair[dl_target] + dl_index, dl_pair[dl_origin] + dl_shead, dl_sedge - dl_shead );

                        /* update index */
                        dl_index = dl_sedge;

                    } else {

                        /* empty stack condition */
                        if ( dl_shead >= dl_sedge ) {

                            /* copy remaining records */
                            std::memcpy( dl_pair[dl_target] + dl_index, dl_pair[dl_origin] + dl_fhead, dl_fedge - dl_fhead );

                            /* update index */
                            dl_index = dl_sedge;

                        } else {

                            /* merge function */
                            if ( dl_sort_algorithm( ( le_real_t * ) ( dl_pair[dl_origin] + dl_fhead ), ( le_real_t * ) ( dl_pair[dl_origin] + dl_shead ), dl_depth ) == true ) {

                                /* copy record */
                                std::memcpy( dl_pair[dl_target] + dl_index, dl_pair[dl_origin] + dl_shead, LE_UV3_RECORD );

                                /* update head */
                                dl_shead += LE_UV3_RECORD;

                            } else {

                                /* copy record */
                                std::memcpy( dl_pair[dl_target] + dl_index, dl_pair[dl_origin] + dl_fhead, LE_UV3_RECORD );

                                /* update head */
                                dl_fhead += LE_UV3_RECORD;

                            }

                            /* update index */
                            dl_index += LE_UV3_RECORD;

                        }

                    }

                }

            }

            /* update step */
            dl_step <<= 1;

            /* swap switch */
            dl_origin = 1 - dl_origin;
            dl_target = 1 - dl_target;

        }

        /* release memory */
        delete [] dl_pair[dl_target];

        /* return sorted buffer */
        return( dl_pair[dl_origin] );

    }

/*
    source - merge methods
 */

    le_void_t dl_sort_algorithm_disk( le_char_t const * const dl_fpath, le_size_t const dl_flength, le_char_t const * const dl_spath, le_size_t const dl_slength, le_char_t const * const dl_opath, le_byte_t const dl_depth ) {

        /* buffer variable */
        le_byte_t * dl_fbuffer( nullptr );

        /* buffer variable */
        le_byte_t * dl_sbuffer( nullptr );

        /* buffer variable */
        le_byte_t * dl_obuffer( nullptr );

        /* stream variable */
        std::fstream dl_fstream;

        /* stream variable */
        std::fstream dl_sstream;

        /* stream variable */
        std::fstream dl_ostream;

        /* range variable */
        le_size_t dl_fhead( 1 );
        le_size_t dl_shead( 1 );
        le_size_t dl_ohead( 0 );

        /* range variable */
        le_size_t dl_fedge( 1 );
        le_size_t dl_sedge( 1 );

        /* allocate buffer memory */
        if ( ( dl_fbuffer = new ( std::nothrow ) le_byte_t[DL_SORT_BUFFER] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( dl_sbuffer = new ( std::nothrow ) le_byte_t[DL_SORT_BUFFER] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* allocate buffer memory */
        if ( ( dl_obuffer = new ( std::nothrow ) le_byte_t[DL_SORT_BUFFER] ) == nullptr ) {

            /* send message */
            throw( LC_ERROR_MEMORY );

        }

        /* create input stream */
        dl_fstream.open( ( char * ) dl_fpath, std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_fstream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* create input stream */
        dl_sstream.open( ( char * ) dl_spath, std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_sstream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* create output stream */
        dl_ostream.open( ( char * ) dl_opath, std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_ACCESS );

        }

        /* merge sort dichotomous loop */
        while ( ( dl_fedge != 0 ) || ( dl_sedge != 0 ) ) {

            /* check buffer state */
            if ( ( dl_fhead == dl_fedge ) && ( dl_fedge != 0 ) ) {

                /* read stream chunk */
                dl_fstream.read( ( char * ) dl_fbuffer, DL_SORT_BUFFER );

                /* retrieve byte count */
                dl_fedge = dl_fstream.gcount();

                /* reset head */
                dl_fhead = 0; 

            }  
 
            /* check buffer state */
            if ( ( dl_shead == dl_sedge ) && ( dl_sedge != 0 ) ) {

                /* read stream chunk */
                dl_sstream.read( ( char * ) dl_sbuffer, DL_SORT_BUFFER );

                /* retrieve byte count */
                dl_sedge = dl_sstream.gcount();

                /* reset head */
                dl_shead = 0;

            }

            /* check buffer state */
            if ( ( dl_fedge == 0 ) && ( dl_sedge == 0 ) ) {

                /* write stream chunk */
                dl_ostream.write( ( char * ) dl_obuffer, dl_ohead );

            } else {

                /* check buffer state */
                if ( dl_fedge == 0 ) {

                    /* copy record */
                    std::memcpy( ( char * ) ( dl_obuffer + dl_ohead ), ( char * ) ( dl_sbuffer + dl_shead ), LE_UV3_RECORD );

                    /* update head */
                    dl_shead += LE_UV3_RECORD;

                } else {

                    /* check buffer state */
                    if ( dl_sedge == 0 ) {

                        /* copy record */
                        std::memcpy( ( char * ) ( dl_obuffer + dl_ohead ), ( char * ) ( dl_fbuffer + dl_fhead ), LE_UV3_RECORD );

                        /* update head */
                        dl_fhead += LE_UV3_RECORD;

                    } else {

                        /* merge function */
                        if ( dl_sort_algorithm( ( le_real_t * ) ( dl_fbuffer + dl_fhead ), ( le_real_t * ) ( dl_sbuffer + dl_shead ), dl_depth ) == true ) {

                            /* copy record */
                            std::memcpy( ( char * ) ( dl_obuffer + dl_ohead ), ( char * ) ( dl_sbuffer + dl_shead ), LE_UV3_RECORD );

                            /* update head */
                            dl_shead += LE_UV3_RECORD;

                        } else {

                            /* copy record */
                            std::memcpy( ( char * ) ( dl_obuffer + dl_ohead ), ( char * ) ( dl_fbuffer + dl_fhead ), LE_UV3_RECORD );

                            /* update head */
                            dl_fhead += LE_UV3_RECORD;

                        }

                    }

                }

                /* check buffer state */
                if ( ( dl_ohead += LE_UV3_RECORD ) == DL_SORT_BUFFER ) {

                    /* write stream chunk */
                    dl_ostream.write( ( char * ) dl_obuffer, dl_ohead );

                    /* reset head */
                    dl_ohead = 0;

                }

            }

        }

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_sstream.close();

        /* delete input stream */
        dl_fstream.close();

        /* release buffer memory */
        delete [] dl_obuffer;

        /* release buffer memory */
        delete [] dl_sbuffer;

        /* release buffer memory */
        delete [] dl_fbuffer;

    }

