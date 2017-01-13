/*
 *  dalai-suite - radius
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

    # include "dalai-radius.hpp"

/*
    source - filtering methods
 */

    double dl_radius_mean( std::ifstream & dl_istream, long long int dl_size, long long int dl_count ) {

        /* array variables */
        char   * dl_point( nullptr );
        char   * dl_chunk( nullptr );
        double * dl_close( nullptr );

        /* array mapping variables */
        double * dl_posec( nullptr );
        double * dl_posep( nullptr );

        /* distance variables */
        double dl_radius( 0.0 );

        /* returned value variables */
        double dl_return( 0.0 );

        /* selection step variables */
        long long int dl_step( dl_size / dl_count );

        /* allocate and check buffer memory */
        if ( ( dl_point = new char[dl_count * 27] ) != nullptr ) {

            /* allocate and check buffer memory */
            if ( ( dl_chunk = new char[DL_RADIUS_CHUNK * 27] ) != nullptr ) {

                /* allocate and check buffer memory */
                if ( ( dl_close = new double[dl_count] ) != nullptr ) {

                    /* initialise close radii */
                    for ( long long int dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

                        /* setting initial radius */
                        dl_close[dl_parse] = 1e300;

                    }

                    /* gather selected points */
                    for ( long long int dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

                        /* stream position */
                        dl_istream.seekg( dl_parse * dl_step * 27 );

                        /* read stream record */
                        dl_istream.read( dl_point + ( dl_parse * 27 ), 27 );

                    }

                    /* reset input stream position */
                    dl_istream.seekg( 0 );

                    /* parsing stream chunks */
                    do {

                        /* read stream chunk */
                        dl_istream.read( dl_chunk, DL_RADIUS_CHUNK * 27 );

                        /* parsing chunk records */
                        for ( long long int dl_parse( 0 ); dl_parse < dl_istream.gcount(); dl_parse += 27 ) {

                            /* parsing seleced points */
                            for ( long long int dl_index( 0 ); dl_index < dl_count; dl_index ++ ) {

                                /* create array mapping */
                                dl_posec = ( double * ) ( dl_chunk + dl_parse      );
                                dl_posep = ( double * ) ( dl_point + dl_index * 27 );

                                /* compute and check radius */
                                if ( ( dl_radius =

                                    ( dl_posec[0] - dl_posep[0] ) * ( dl_posec[0] - dl_posep[0] ) +
                                    ( dl_posec[1] - dl_posep[1] ) * ( dl_posec[1] - dl_posep[1] ) +
                                    ( dl_posec[2] - dl_posep[2] ) * ( dl_posec[2] - dl_posep[2] )

                                ) > 0.0 ) {

                                    /* search for minimum radius - assignation */
                                    if ( dl_radius < dl_close[dl_index] ) dl_close[dl_index] = dl_radius;

                                }

                            }

                        }

                    } while ( dl_istream.gcount() > 0 );

                    /* compute sum of close radius */
                    for ( long long int dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

                        /* accumulate radius */
                        dl_return += sqrt( dl_close[dl_parse] );

                    }

                    /* release buffer memory */
                    delete [] dl_close;

                }

                /* release buffer memory */
                delete [] dl_chunk;

            }

            /* release buffer memory */
            delete [] dl_point;

        }

        /* return computed mean radius */
        return( dl_return / dl_count );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* message variables */
        int dl_message( EXIT_SUCCESS );

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* stream record variables */
        long long int dl_size( 0 );

        /* mean count variables */
        long long int dl_count( lc_read_signed( argc, argv, "--count", "-c", 32 ) );

        /* mean distance variables */
        double dl_mean( 0.0 );

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--uf3", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == true ) {

            /* retrieve stream records count */
            dl_size = dl_istream.tellg() / 27;

            /* compute and check mean distance */
            if ( ( dl_mean = dl_radius_mean( dl_istream, dl_size, dl_count ) ) > 0.0 ) {

                std::cerr << dl_mean << std::endl;

            } else {

                /* display message */
                std::cerr << "dalai-suite : error : unable to compute mean radius" << std::endl;

                /* push message */
                dl_message = EXIT_FAILURE;

            }

            /* delete input stream */
            dl_istream.close();

        } else {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

            /* push message */
            dl_message = EXIT_FAILURE;

        }

        /* send message */
        return( dl_message );

    }

