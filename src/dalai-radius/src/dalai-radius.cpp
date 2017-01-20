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
    source - storage methods
 */

    bool dl_radius_temp_create( char * const dl_tpath ) {

        /* compose temporary directory */
        sprintf( dl_tpath, "%s", "/tmp/dalai-radius-XXXXXXXX" );

        /* create temporary directory */
        if ( mkdtemp( dl_tpath ) != nullptr ) {

            /* send message */
            return( true );

        } else {

            /* send message */
            return( false );

        }

    }

    void dl_radius_temp_delete( char const * const dl_tpath ) {

        /* delete temporary directory */
        rmdir( dl_tpath );

    }

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
        long long int dl_step( ( dl_size / 27 ) / dl_count );

        /* allocate and check buffer memory */
        if ( ( dl_point = new char[dl_count * 27] ) != nullptr ) {

            /* allocate and check buffer memory */
            if ( ( dl_chunk = new char[DL_RADIUS_CHUNK * 27] ) != nullptr ) {

                /* allocate and check buffer memory */
                if ( ( dl_close = new double[dl_count] ) != nullptr ) {

                    /* initialise close radius */
                    for ( long long int dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

                        /* setting initial radius */
                        dl_close[dl_parse] = std::numeric_limits<double>::max();

                    }

                    /* gather selected points */
                    for ( long long int dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

                        /* stream position */
                        dl_istream.seekg( dl_parse * dl_step * 27 );

                        /* read stream record */
                        dl_istream.read( dl_point + ( dl_parse * 27 ), 27 );

                    }

                    /* clear stream state */
                    dl_istream.clear();

                    /* reset input stream position */
                    dl_istream.seekg( 0 );

                    /* parsing stream chunks */
                    do {

                        /* read stream chunk */
                        dl_istream.read( dl_chunk, DL_RADIUS_CHUNK * 27 );

                        /* parsing chunk records */
                        for ( long long int dl_parse( 0 ); dl_parse < dl_istream.gcount(); dl_parse += 27 ) {

                            /* create array mapping */
                            dl_posec = ( double * ) ( dl_chunk + dl_parse );

                            /* parsing seleced points */
                            for ( long long int dl_index( 0 ); dl_index < dl_count; dl_index ++ ) {

                                /* create array mapping */
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
    source - hashing methods
 */

    bool dl_radius_hash_create( std::ifstream & dl_istream, char * dl_tpath, long long int dl_size, double dl_radius, double dl_factor ) {

        /* chunk buffer variables */
        char * dl_chunk( nullptr );

        /* array mapping variables */
        double * dl_pose( nullptr );

        /* hash files path variables */
        std::stringstream dl_file;

        long long int dl_read( 0 );

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new char[DL_RADIUS_CHUNK * 27] ) != nullptr ) {

            /* clear input stream */
            dl_istream.clear();

            /* reset input stream */
            dl_istream.seekg( 0 );

            /* hashing input stream */
            do {

                /* read input stream chunk */
                dl_istream.read( dl_chunk, DL_RADIUS_CHUNK * 27 );

                dl_read = dl_istream.gcount();

                /* parsing chunk elements */
                for ( long long int dl_parse( 0 ); dl_parse < dl_read; dl_parse += 27 ) {

                    /* compute array mapping */
                    dl_pose = ( double * ) ( dl_chunk + dl_parse );

                    /* compose path */
                    dl_file.str( "" );
                    dl_file << dl_tpath << "/";
                    dl_file << ( long long int ) ( dl_pose[0] / ( dl_radius * dl_factor * 100 ) ) << "_";
                    dl_file << ( long long int ) ( dl_pose[1] / ( dl_radius * dl_factor * 100 ) ) << "_";
                    dl_file << ( long long int ) ( dl_pose[2] / ( dl_radius * dl_factor * 100 ) ) << ".uf3";

                    std::ofstream dl_ostream;

                    dl_ostream.open( dl_file.str().c_str(), std::ios::binary | std::ios::out | std::ios::app );

                    if ( dl_ostream.is_open() == true ) {

                        dl_ostream.write( dl_chunk + dl_parse, 27 );

                        dl_ostream.close();

                    }

                }

            } while ( dl_read > 0 );

            /* release buffer memory */
            delete [] dl_chunk;

            /* send message */
            return( true );

        } else {

            /* send message */
            return( false );

        }

    }

/*
    source - filtering methods
 */

    bool dl_radius_filter( std::ofstream & dl_ostream, char const * const dl_tpath, double dl_radius, double dl_factor ) {

        /* buffer variables */
        char * dl_buffer( nullptr );

        /* distance buffer */
        double * dl_dists( nullptr );

        /* directory structure variables */
        DIR * dl_dir( nullptr );

        /* entiry structure variables */
        struct dirent * dl_ent( nullptr );

        /* path variables */
        char dl_file[256];

        /* stream variables */
        std::ifstream dl_istream;

        /* open and check directory */
        if ( ( dl_dir = opendir( dl_tpath ) ) != nullptr ) {

            /* entities enumeration */
            while ( ( dl_ent = readdir( dl_dir ) ) != nullptr ) {

                /* check for files */
                if ( dl_ent->d_type == DT_REG ) {

                    sprintf( dl_file, "%s/%s", dl_tpath, dl_ent->d_name );

                    dl_istream.open( dl_file, std::ios::in | std::ios::ate | std::ios::binary );

                    long long int dl_size( 0 );

                    dl_size = dl_istream.tellg();
                    std::cerr << dl_size << std::endl;
                    dl_istream.seekg( 0 );

                    if ( ( dl_buffer = new char[dl_size] ) != nullptr ) {

                        if ( ( dl_dists = new double[dl_size/27] ) != nullptr ) {

                            dl_istream.read( dl_buffer, dl_size );

                            for ( long long int dl_parse( 0 ); dl_parse < dl_size / 27; dl_parse ++ ) {

                                dl_dists[dl_parse] = std::numeric_limits<double>::max();

                            }

                            for ( long long int dl_parse( 0 ); dl_parse < ( dl_size - 27 ); dl_parse += 27 ) {

                                for ( long long int dl_index( dl_parse + 27 ); dl_index < dl_size; dl_index += 27 ) {

                                    double * dl_pose1 = ( double * ) ( dl_buffer + dl_parse );
                                    double * dl_pose2 = ( double * ) ( dl_buffer + dl_index );

                                    double dl_radius( sqrt(

                                        ( dl_pose1[0] - dl_pose2[0] ) * ( dl_pose1[0] - dl_pose2[0] ) +
                                        ( dl_pose1[1] - dl_pose2[1] ) * ( dl_pose1[1] - dl_pose2[1] ) +
                                        ( dl_pose1[2] - dl_pose2[2] ) * ( dl_pose1[2] - dl_pose2[2] )

                                    ) );

                                    if ( dl_radius < dl_dists[dl_parse/27] ) dl_dists[dl_parse/27] = dl_radius;
                                    if ( dl_radius < dl_dists[dl_index/27] ) dl_dists[dl_index/27] = dl_radius;

                                }

                            }

                            for ( long long int dl_parse( 0 ); dl_parse < dl_size / 27; dl_parse ++ ) {

                                if ( dl_dists[dl_parse] < dl_factor * dl_radius ) {

                                    dl_ostream.write( dl_buffer + dl_parse * 27, 27 );

                                }

                            }

                            delete [] dl_dists;

                        }

                        delete [] dl_buffer;

                    }

                    dl_istream.close();

                    std::remove( dl_file );

                }

            }

            /* close directory */
            closedir( dl_dir );

        }

        return( true );

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

        /* filtering factor variables */
        double dl_factor( lc_read_double( argc, argv, "--factor", "-f", 2.0 ) );

        /* mean distance variables */
        double dl_mean( 0.0 );

        /* temporary path variables */
        char dl_tpath[256];

        /* create and check temporary file */
        if ( dl_radius_temp_create( dl_tpath ) != true ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to create temporary directory" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == true ) {

            /* retrieve stream records count */
            dl_size = dl_istream.tellg();

            /* compute and check mean distance */
            if ( ( dl_mean = dl_radius_mean( dl_istream, dl_size, dl_count ) ) > 0.0 ) {

                /* display information */
                std::cout << dl_mean << std::endl;

                /* create hashing storage */
                if ( dl_radius_hash_create( dl_istream, dl_tpath, dl_size, dl_mean, dl_factor ) == true ) {

                    /* create output stream */
                    dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

                    /* check output stream */
                    if ( dl_ostream.is_open() == true ) {

                        /* filtering method */
                        dl_radius_filter( dl_ostream, dl_tpath, dl_mean, dl_factor );

                        /* delete output stream */
                        dl_ostream.close();

                    } else {

                        /* display message */
                        std::cerr << "dalai-suite : error : unable to access output file" << std::endl;

                        /* push message */
                        dl_message = EXIT_FAILURE;

                    }

                } else {

                        /* display message */
                        std::cerr << "dalai-suite : error : unable to create hashing storage" << std::endl;

                }

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

        /* delete temporary directory */
        dl_radius_temp_delete( dl_tpath );

        /* send message */
        return( dl_message );

    }

