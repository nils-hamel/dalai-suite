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
        sprintf( dl_tpath, "%s", "/tmp/dalai-radius-XXXXXX" );

        /* create temporary directory */
        if ( mkdtemp( dl_tpath ) != nullptr ) {

            /* send message */
            return( true );

        /* send message */
        } else { return( false ); }

    }

    void dl_radius_temp_delete( char const * const dl_tpath ) {

        /* delete temporary directory */
        rmdir( dl_tpath );

    }

/*
    source - hashing methods
 */

    bool dl_radius_hash( std::ifstream & dl_istream, char const * const dl_tpath, int64_t const dl_size, double const dl_radius ) {

        /* output stream variables */
        std::ofstream dl_ostream;

        /* hash files path variables */
        char dl_path[256];

        /* chunk buffer variables */
        char * dl_chunk( nullptr );

        /* array mapping variables */
        double * dl_pose( nullptr );

        /* hashing variables */
        double dl_hash( dl_radius * 100.0 );

        /* returned value variables */
        bool dl_message( true );

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new char[DL_RADIUS_CHUNK * 27] ) == nullptr ) {

            /* send message */
            return( false );

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* input stream hashing */
        do {

            /* read input stream chunk */
            dl_istream.read( dl_chunk, DL_RADIUS_CHUNK * 27 );

            /* parsing chunk elements */
            for ( int64_t dl_parse( 0 ), dl_limit( dl_istream.gcount() ); dl_parse < dl_limit; dl_parse += 27 ) {

                /* compute array mapping */
                dl_pose = ( double * ) ( dl_chunk + dl_parse );

                /* compose path string */
                sprintf( dl_path, "%s/%" PRId64 "_%" PRId64 "_%" PRId64 ".uf3", dl_tpath, ( int64_t ) ( dl_pose[0] / dl_hash ), ( int64_t ) ( dl_pose[1] / dl_hash ), ( int64_t ) ( dl_pose[2] / dl_hash ) );

                /* create output stream */
                dl_ostream.open( dl_path, std::ios::binary | std::ios::out | std::ios::app );

                /* check output stream */
                if ( dl_ostream.is_open() == true ) {

                    /* export chunk element */
                    dl_ostream.write( dl_chunk + dl_parse, 27 );

                    /* close output stream */
                    dl_ostream.close();

                /* push message */
                } else { dl_message = false; }

            }

        } while ( ( dl_istream.gcount() > 0 ) && ( dl_message == true ) );

        /* release buffer memory */
        delete [] dl_chunk;

        /* send message */
        return( dl_message );

    }

/*
    source - filtering methods
 */

    double dl_radius_mean( std::ifstream & dl_istream, int64_t const dl_size, int64_t const dl_count ) {

        /* array variables */
        char   * dl_point( nullptr );
        char   * dl_chunk( nullptr );
        double * dl_dists( nullptr );

        /* array mapping variables */
        double * dl_posec( nullptr );
        double * dl_posep( nullptr );

        /* selection step variables */
        int64_t dl_step( ( dl_size / 27 ) / dl_count );

        /* distance variables */
        double dl_radius( 0.0 );

        /* returned value variables */
        double dl_return( 0.0 );

        /* allocate and check buffer memory */
        if ( ( dl_point = new char[dl_count * 27] ) != nullptr ) {

            /* allocate and check buffer memory */
            if ( ( dl_dists = new double[dl_count] ) != nullptr ) {

                /* allocate and check buffer memory */
                if ( ( dl_chunk = new char[DL_RADIUS_CHUNK * 27] ) != nullptr ) {

                    /* initialise distances array */
                    for ( int64_t dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

                        /* setting initial distance */
                        dl_dists[dl_parse] = std::numeric_limits<double>::max();

                    }

                    /* clear stream state */
                    dl_istream.clear();

                    /* reset input stream position */
                    dl_istream.seekg( 0 );

                    /* import selected points */
                    for ( int64_t dl_parse( 0 ), dl_limit( dl_count * 27 ); dl_parse < dl_limit; dl_parse += 27 ) {

                        /* stream position */
                        dl_istream.seekg( dl_parse * dl_step );

                        /* read stream record */
                        dl_istream.read( dl_point + dl_parse, 27 );

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
                        for ( int64_t dl_parse( 0 ), dl_limit( dl_istream.gcount() ) ; dl_parse < dl_limit ; dl_parse += 27 ) {

                            /* create array mapping */
                            dl_posec = ( double * ) ( dl_chunk + dl_parse );

                            /* parsing seleced points */
                            for ( int64_t dl_index( 0 ); dl_index < dl_count; dl_index ++ ) {

                                /* create array mapping */
                                dl_posep = ( double * ) ( dl_point + dl_index * 27 );

                                /* compute and check distance */
                                if ( ( dl_radius =

                                    ( dl_posec[0] - dl_posep[0] ) * ( dl_posec[0] - dl_posep[0] ) +
                                    ( dl_posec[1] - dl_posep[1] ) * ( dl_posec[1] - dl_posep[1] ) +
                                    ( dl_posec[2] - dl_posep[2] ) * ( dl_posec[2] - dl_posep[2] )

                                ) > 0.0 ) {

                                    /* search for minimum distance - assignation of minimum */
                                    if ( dl_radius < dl_dists[dl_index] ) dl_dists[dl_index] = dl_radius;

                                }

                            }

                        }

                    } while ( dl_istream.gcount() > 0 );

                    /* compute mean-minimum distance */
                    for ( int64_t dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

                        /* accumulate distances */
                        dl_return += sqrt( dl_dists[dl_parse] );

                    }

                    /* release buffer memory */
                    delete [] dl_chunk;

                /* send message */
                } else { return( 0.0 ); }

                /* release buffer memory */
                delete [] dl_dists;

            /* send message */
            } else { return( 0.0 ); }

            /* release buffer memory */
            delete [] dl_point;

        /* send message */
        } else { return( 0.0 ); }

        /* return computed mean radius */
        return( dl_return / dl_count );

    }

    bool dl_radius_filter( std::ofstream & dl_ostream, char const * const dl_tpath, double const dl_radius, double const dl_factor, int const dl_mode ) {

        /* stream variables */
        std::ifstream dl_istream;

        /* hashed chunks path variables */
        char dl_path[256];

        /* directory structure variables */
        DIR * dl_directory( nullptr );

        /* entity structure variables */
        struct dirent * dl_entity( nullptr );

        /* returned value variables */
        bool dl_message( true );

        /* open and check directory */
        if ( ( dl_directory = opendir( dl_tpath ) ) == nullptr ) {

            /* send message */
            return( false );

        }

        /* directory entities enumeration */
        while ( ( ( dl_entity = readdir( dl_directory ) ) != nullptr ) && ( dl_message == true ) ) {

            /* directory entity filtering */
            if ( dl_entity->d_type == DT_REG ) {

                /* compose input stream path */
                sprintf( dl_path, "%s/%s", dl_tpath, dl_entity->d_name );

                /* create input stream */
                dl_istream.open( dl_path, std::ios::in | std::ios::ate | std::ios::binary );

                /* check input stream */
                if ( dl_istream.is_open() == true ) {

                    /* uniform filtering method */
                    dl_radius_filter_segment( dl_istream, dl_ostream, dl_istream.tellg(), dl_radius, dl_factor, dl_mode );

                    /* delete input stream */
                    dl_istream.close();

                /* push message */
                } else { dl_message = false; }

                /* remove processed file */
                std::remove( dl_path );

            }

        }

        /* close directory */
        closedir( dl_directory );

        /* send message */
        return( dl_message );

    }

    bool dl_radius_filter_segment( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_radius, double const dl_factor, int const dl_mode ) {

        /* distance variables */
        double dl_distance( 0.0 );

        /* condition variables */
        double dl_condition( dl_radius * dl_radius * dl_factor * dl_factor );

        /* indexation variables */
        int64_t dl_filter( 0 );

        /* buffer variables */
        char   * dl_chunk( nullptr );
        double * dl_dists( nullptr );

        /* array mapping variables */
        double * dl_pose1( nullptr );
        double * dl_pose2( nullptr );

        /* check size consistency */
        if ( dl_size <= 27 ) {

            /* send message */
            return( true );

        }

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new char[dl_size] ) != nullptr ) {

            /* allocate and check buffer memory */
            if ( ( dl_dists = new double[dl_size/27] ) != nullptr ) {

                /* clear input stream */
                dl_istream.clear();

                /* reset input stream */
                dl_istream.seekg( 0 );

                /* read hashed chunk content */
                dl_istream.read( dl_chunk, dl_size );

                /* reset minimum distance array */
                for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

                    /* initialise distance */
                    dl_dists[dl_parse] = std::numeric_limits<double>::max();

                }

                /* search minimum distances */
                for ( int64_t dl_parse( 0 ), dl_limit( dl_size - 27 ); dl_parse < dl_limit; dl_parse += 27 ) {

                    /* compute array mapping */
                    dl_pose1 = ( double * ) ( dl_chunk + dl_parse );

                    /* search minimum distances */
                    for ( long long int dl_index( dl_parse + 27 ); dl_index < dl_size; dl_index += 27 ) {

                        /* compose array mapping */
                        dl_pose2 = ( double * ) ( dl_chunk + dl_index );

                        /* compute distance */
                        dl_distance = ( dl_pose1[0] - dl_pose2[0] ) * ( dl_pose1[0] - dl_pose2[0] ) +
                                      ( dl_pose1[1] - dl_pose2[1] ) * ( dl_pose1[1] - dl_pose2[1] ) +
                                      ( dl_pose1[2] - dl_pose2[2] ) * ( dl_pose1[2] - dl_pose2[2] );

                        /* check and assign distance */
                        if ( dl_distance < dl_dists[dl_parse/27] ) dl_dists[dl_parse/27] = dl_distance;
                        if ( dl_distance < dl_dists[dl_index/27] ) dl_dists[dl_index/27] = dl_distance;

                    }

                }

                /* check adaptative mode */
                if ( dl_mode == DL_RADIUS_ADAPTATIVE ) {

                    /* reset condition */
                    dl_condition = 0.0;

                    /* compute local mean distance */
                    for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

                        /* accumulate distances */
                        dl_condition += dl_dists[dl_parse];

                    }

                    /* compute mean distance and adaptative condition */
                    dl_condition = ( dl_condition / ( dl_size / 27 ) ) * dl_factor * dl_factor;

                }

                /* reset delayed indexation */
                dl_filter = 0;

                /* filtering loop */
                for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

                    /* check filtering condition */
                    if ( dl_dists[dl_parse] < dl_condition ) {

                        /* check delayed indexation */
                        if ( dl_filter < dl_parse ) {

                            /* index filtered element */
                            std::memcpy( dl_chunk + ( dl_filter * 27 ), dl_chunk + ( dl_parse * 27 ), 27 );

                        }

                        /* update delayed indexation */
                        dl_filter ++;

                    }

                }

                /* write filtered elements */
                dl_ostream.write( dl_chunk, dl_filter * 27 );

                /* release buffer memory */
                delete [] dl_dists;

            /* send message */
            } else { return( false ); }

            /* release buffer memory */
            delete [] dl_chunk;

        /* send message */
        } else { return( false ); }

        /* send message */
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

        /* temporary path variables */
        char dl_tpath[256];

        /* mode variables */
        int dl_mode( DL_RADIUS_UNIFORM );

        /* mean distance variables */
        double dl_mean( 0.0 );

        /* stream record variables */
        int64_t dl_size( 0 );

        /* mean count variables */
        int64_t dl_count( lc_read_signed( argc, argv, "--count", "-c", 64 ) );

        /* filtering factor variables */
        double dl_factor( lc_read_double( argc, argv, "--factor", "-f", 2.0 ) );

        /* create and check temporary file */
        if ( dl_radius_temp_create( dl_tpath ) != true ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to create temporary directory" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* check filtering mode */
        if ( lc_read_flag( argc, argv, "--uniform", "-u" ) == LC_TRUE ) {

            /* assign mode */
            dl_mode = DL_RADIUS_UNIFORM;

        } else if ( lc_read_flag( argc, argv, "--adaptative", "-a" ) == LC_TRUE ) {

            /* assign mode */
            dl_mode = DL_RADIUS_ADAPTATIVE;

        }

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == true ) {

            /* retrieve stream records count */
            dl_size = dl_istream.tellg();

            /* compute and check mean distance */
            if ( ( dl_mean = dl_radius_mean( dl_istream, dl_size, dl_count ) ) > 0.0 ) {

                /* create hashing storage */
                if ( dl_radius_hash( dl_istream, dl_tpath, dl_size, dl_mean ) == true ) {

                    /* create output stream */
                    dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

                    /* check output stream */
                    if ( dl_ostream.is_open() == true ) {

                        /* filtering method */
                        if ( dl_radius_filter( dl_ostream, dl_tpath, dl_mean, dl_factor, dl_mode ) != true ) {

                            /* display message */
                            std::cerr << "dalai-suite : error : unable to apply filter" << std::endl;

                            /* push message */
                            dl_message = EXIT_FAILURE;

                        }

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

                    /* push message */
                    dl_message = EXIT_FAILURE;

                }

            } else {

                /* display message */
                std::cerr << "dalai-suite : error : unable to compute mean distance" << std::endl;

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

