/*
 *  dalai-suite - interlis-uv3
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

    # include "dalai-interlis-uv3.hpp"

/*
    source - buffer methods
 */

    void dl_interlis_init( le_byte_t * dl_buffer, le_byte_t const dl_red, le_byte_t const dl_green, le_byte_t const dl_blue ) {

        /* buffer pointer variable */
        le_real_t * dl_pose( NULL );

        /* assign position pointer */
        dl_pose = ( le_real_t * ) dl_buffer;

        /* initialise position */
        dl_pose[0] = 0.0;
        dl_pose[1] = 0.0;
        dl_pose[2] = 0.0;

        /* assign data pointer */
        dl_buffer += LE_ARRAY_DATA_POSE;

        /* initialise type */
        dl_buffer[0] = 2;

        /* initialise color */
        dl_buffer[1] = dl_red;
        dl_buffer[2] = dl_green;
        dl_buffer[3] = dl_blue;

    }

    void dl_interlis_export( le_byte_t * const dl_buffer, std::ofstream & dl_stream, le_real_t const dl_x, le_real_t const dl_y ) {

        /* buffer pointer variable */
        le_real_t * dl_pose( NULL );

        /* assign position pointer */
        dl_pose = ( le_real_t * ) dl_buffer;

        /* assign position values */
        dl_pose[0] = dl_x;
        dl_pose[1] = dl_y;

        /* export buffer */
        dl_stream.write( ( char * ) dl_buffer, LE_ARRAY_DATA );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* buffer variable */
        le_byte_t dl_buffer[LE_ARRAY_DATA] = { 0 };

        /* target variable */
        char * dl_target_topic( lc_read_string( argc, argv, "--topic", "-t" ) );

        /* target variable */
        char * dl_target_table( lc_read_string( argc, argv, "--table", "-a" ) );

        /* color variable */
        le_byte_t dl_red( lc_read_unsigned( argc, argv, "--red", "-r", 0 ) );

        /* color variable */
        le_byte_t dl_green( lc_read_unsigned( argc, argv, "--green", "-g", 0 ) );

        /* color variable */
        le_byte_t dl_blue( lc_read_unsigned( argc, argv, "--blue", "-b", 0 ) );

        /* coordinates variable */
        le_real_t dl_push_x( 0.0 );
        le_real_t dl_push_y( 0.0 );

        /* coordinate variable */
        le_real_t dl_read_x( 0.0 );
        le_real_t dl_read_y( 0.0 );

        /* reading mode variable */
        le_enum_t dl_mode( 0 );

        /* reading token variable */
        std::string dl_token;

        /* input stream variable */
        std::ifstream dl_istream;

        /* output stream variable */
        std::ofstream dl_ostream;

    /* error management */
    try {

        /* initialise buffer */
        dl_interlis_init( dl_buffer, dl_red, dl_green, dl_blue );

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--interlis", "-i" ), std::ios::in );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* send message */
            throw( LC_ERROR_IO_READ );

        }

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--uv3", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* send message */
            throw( LE_ERROR_IO_WRITE );

        }

        /* input stream reading */
        while ( dl_istream >> dl_token ) {

            /* switch on read mode */
            switch ( dl_mode ) {

                /* read mode */
                case ( 0 ) : {

                    /* token detection */
                    if ( dl_token.compare( "TOPI" ) == 0 ) {

                        /* update mode */
                        dl_mode = 1;

                    }

                } break;

                /* read mode */
                case ( 1 ) : {

                    /* token detection */
                    if ( dl_token.compare( dl_target_topic ) == 0 ) {

                        /* update mode */
                        dl_mode = 2;

                    /* update mode */
                    } else { dl_mode = 0; }

                } break;

                /* read mode */
                case ( 2 ) : {

                    /* token detection */
                    if ( dl_token.compare( "ETOP" ) == 0 ) {

                        /* update mode */
                        dl_mode = 0;

                    } else
                    if ( dl_token.compare( "TABL" ) == 0 ) {

                        /* update mode */
                        dl_mode = 3;

                    }

                } break;

                /* read mode */
                case ( 3 ) :  {

                    /* token detection */
                    if ( dl_token.compare( dl_target_table ) == 0 ) {

                        /* update mode */
                        dl_mode = 4;

                    /* update mode */
                    } else { dl_mode = 2; }

                } break;

                /* read mode */
                case ( 4 ) : {

                    /* token detection */
                    if ( dl_token.compare( "ETAB" ) == 0 ) {

                        /* update mode */
                        dl_mode = 2;

                    } else
                    if ( dl_token.compare( "STPT" ) == 0 ) {

                        /* update mode */
                        dl_mode = 5;

                    } else
                    if ( dl_token.compare( "ARCP" ) == 0 ) {

                        /* update mode */
                        dl_mode = 6;

                    } else
                    if ( dl_token.compare( "LIPT" ) == 0 ) {

                        /* update mode */
                        dl_mode = 6;

                    }

                } break;

                /* read mode */
                case ( 5 ) : {

                    /* convert and push coordinate */
                    dl_push_x = std::stod( dl_token );

                    /* read next token */
                    dl_istream >> dl_token;

                    /* convert and push coordinate */
                    dl_push_y = std::stod( dl_token );

                    /* update mode */
                    dl_mode = 4;

                } break;

                /* read mode */
                case ( 6 ) : {

                    /* convert coordinate */
                    dl_read_x = std::stod( dl_token );

                    /* read next token */
                    dl_istream >> dl_token;

                    /* convert coordinate */
                    dl_read_y = std::stod( dl_token  );

                    /* export buffer */
                    dl_interlis_export( dl_buffer, dl_ostream, dl_push_x, dl_push_y );

                    /* export buffer */
                    dl_interlis_export( dl_buffer, dl_ostream, dl_read_x, dl_read_y );

                    /* push coordinates */
                    dl_push_x = dl_read_x;
                    dl_push_y = dl_read_y;

                    /* update mode */
                    dl_mode = 4;

                } break;

            };

        }

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

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

