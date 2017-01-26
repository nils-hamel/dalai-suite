/*
 *  dalai-suite - filter
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

    # include "dalai-filter.hpp"

/*
    source - storage methods
 */

    bool dl_filter_temporary( char * const dl_path, int dl_mode ) {

        /* check mode value */
        if ( dl_mode == DL_FILTER_CREATE ) {

            /* compose temporary storage path */
            sprintf( dl_path, "%s", "/tmp/dalai-suite-XXXXXX" );

            /* create temporary storage */
            if ( mkdtemp( dl_path ) != nullptr ) {

                /* send message */
                return( true );

            /* send message */
            } else { return( false ); }

        } else {

            /* delete temporary storage */
            if ( rmdir( dl_path ) == 0 ) {

                /* send message */
                return( true );

            /* send message */
            } else { return( false ); }

        }

    }

/*
    source - hashing methods
 */

    bool dl_filter_hash( std::ifstream & dl_istream, char const * const dl_opath, double const dl_mean ) {

        /* stream path variables */
        char dl_fpath[8192];

        /* stream variables */
        std::ofstream dl_ostream;

        /* buffer variables */
        char * dl_chunk( nullptr );

        /* array mapping variables */
        double * dl_pose( nullptr );

        /* hashing parameter variables */
        double dl_hash( dl_mean * 100 );

        /* returned value variables */
        bool dl_return( true );

        /* check consistency */
        if ( dl_opath == nullptr ) {

            /* send message */
            return( false );

        }

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new char[DL_FILTER_CHUNK * 27] ) == nullptr ) {

            /* send message */
            return( false );

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* parsing input stream chunks */
        do {

            /* read input stream chunk */
            dl_istream.read( dl_chunk, DL_FILTER_CHUNK * 27 );

            /* parsing input stream chunk */
            for ( int64_t dl_parse( 0 ), dl_limit( dl_istream.gcount() ); dl_parse < dl_limit; dl_parse += 27 ) {

                /* compute and assign array mapping */
                dl_pose = ( double * ) ( dl_chunk + dl_parse );

                /* compute stream path */
                sprintf( dl_fpath, "%s/%+" PRId64 "_%+" PRId64 "_%+" PRId64 ".uf3", dl_opath, ( int64_t ) floor( dl_pose[0] / dl_hash ), ( int64_t ) floor( dl_pose[1] / dl_hash ), ( int64_t ) floor( dl_pose[2] / dl_hash ) );

                /* create output stream */
                dl_ostream.open( dl_fpath, std::ios::app | std::ios::out | std::ios::binary );

                /* check output stream */
                if ( dl_ostream.is_open() == true ) {

                    /* export chunk element */
                    dl_ostream.write( dl_chunk + dl_parse, 27 );

                    /* delete output stream */
                    dl_ostream.close();

                /* push message */
                } else { dl_return = false; }

            }

        } while ( ( dl_istream.gcount() > 0 ) && ( dl_return == true ) );

        /* release buffer memory */
        delete [] dl_chunk;

        /* send message */
        return( dl_return );

    }

/*
    source - statistical methods
 */

    double dl_filter_stat( std::ifstream & dl_istream, int64_t const dl_size, int64_t const dl_count ) {

        /* distance variables */
        double dl_distance( 0.0 );

        /* buffer variables */
        char   * dl_sample( nullptr );
        char   * dl_chunks( nullptr );
        double * dl_values( nullptr );

        /* array mapping variables */
        double * dl_posei( nullptr );
        double * dl_poses( nullptr );

        /* allocate and check buffer memory */
        if ( ( dl_sample = new char[dl_count * 27] ) == nullptr ) {

            /* send message */
            return( 0.0 );

        } else {

            /* allocate and check buffer memory */
            if ( ( dl_chunks = new char[DL_FILTER_CHUNK * 27] ) == nullptr ) {

                /* release buffers memory */
                delete [] dl_sample;

                /* send message */
                return( 0.0 );

            } else {

                /* allocate and check buffer memory */
                if ( ( dl_values = new double[dl_count] ) == nullptr ) {

                    /* release buffers memory */
                    delete [] dl_sample;
                    delete [] dl_chunks;

                    /* send message */
                    return( 0.0 );

                }

            }

        }

        /* initialise values array */
        for ( int64_t dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

            /* assign initial value */
            dl_values[dl_parse] = std::numeric_limits<double>::max();

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* create sample array */
        for ( int64_t dl_parse = 0; dl_parse < dl_count; dl_parse ++ ) {

            /* assign stream position */
            dl_istream.seekg( dl_parse * ( ( ( dl_size / 27 ) / dl_count ) * 27 ) );

            /* read sample values */
            dl_istream.read( dl_sample + ( dl_parse * 27 ), 27 );

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* reading input stream chunks */
        do {

            /* read input stream chunk */
            dl_istream.read( dl_chunks, DL_FILTER_CHUNK * 27 );

            /* parsing input stream chunk */
            for ( int64_t dl_parse( 0 ), dl_limit( dl_istream.gcount() ); dl_parse < dl_limit; dl_parse += 27 ) {

                /* compute and assign array mapping */
                dl_posei = ( double * ) ( dl_chunks + dl_parse );

                /* parsing sample array */
                for ( int64_t dl_index( 0 ); dl_index < dl_count; dl_index ++ ) {

                    /* compute and assign array mapping */
                    dl_poses = ( double * ) ( dl_sample + ( dl_index * 27 ) );

                    /* compute and check distance */
                    if ( ( dl_distance =

                        ( dl_posei[0] - dl_poses[0] ) * ( dl_posei[0] - dl_poses[0] ) +
                        ( dl_posei[1] - dl_poses[1] ) * ( dl_posei[1] - dl_poses[1] ) +
                        ( dl_posei[2] - dl_poses[2] ) * ( dl_posei[2] - dl_poses[2] )

                    ) < dl_values[dl_index] ) {

                        /* avoid identical point */
                        if ( dl_distance > 0.0 ) dl_values[dl_index] = dl_distance;

                    }

                }

            }

        } while ( dl_istream.gcount() > 0 );

        /* prepare statistical mean computation */
        dl_distance = 0.0;

        /* compute statistical mean value */
        for ( int64_t dl_parse( 0 ); dl_parse < dl_count; dl_parse ++ ) {

            /* accumulate distances */
            dl_distance += sqrt( dl_values[dl_parse] );

        }

        /* release buffer memory */
        delete [] dl_sample;
        delete [] dl_chunks;
        delete [] dl_values;

        /* send message - mean minimum value */
        return( dl_distance / dl_count );

    }

/*
    source - filtering methods
 */

    bool dl_filter( std::ofstream & dl_ostream, char const * const dl_opath, double const dl_mean, double const dl_factor, int64_t const dl_threshold, int const dl_mode ) {

        /* path variables */
        char dl_fpath[256];

        /* stream variables */
        std::ifstream dl_istream;

        /* directory structure variables */
        DIR * dl_directory( nullptr );

        /* entity structure variables */
        struct dirent * dl_entity( nullptr );

        /* returned value variables */
        bool dl_return( true );

        /* open and check directory */
        if ( ( dl_directory = opendir( dl_opath ) ) == nullptr ) {

            /* send message */
            return( false );

        }

        /* directory entities enumeration */
        while ( ( ( dl_entity = readdir( dl_directory ) ) != nullptr ) && ( dl_return == true ) ) {

            /* directory entity filtering */
            if ( dl_entity->d_type != DT_REG ) continue;

            /* compose stream path */
            sprintf( dl_fpath, "%s/%s", dl_opath, dl_entity->d_name );

            /* create stream */
            dl_istream.open( dl_fpath, std::ios::in | std::ios::ate | std::ios::binary );

            /* check stream */
            if ( dl_istream.is_open() == true ) {

                /* switch on mode */
                if ( ( dl_mode == DL_FILTER_UNITY_UNIF ) || ( dl_mode == DL_FILTER_UNITY_ADAP ) ) {

                    /* apply filtering method */
                    dl_filter_unity( dl_istream, dl_ostream, dl_istream.tellg(), dl_mean, dl_factor, dl_mode );

                } else
                if ( dl_mode == DL_FILTER_COUNT_UNIF ) {

                    /* apply filtering method */
                    dl_filter_count_unif( dl_istream, dl_ostream, dl_istream.tellg(), dl_mean, dl_factor, dl_threshold );

                } else
                if ( dl_mode == DL_FILTER_COUNT_ADAP ) {

                    /* apply filtering method */
                    dl_filter_count_adap( dl_istream, dl_ostream, dl_istream.tellg(), dl_mean, dl_factor, dl_threshold );

                }

                /* delete stream */
                dl_istream.close();

                /* delete stream file */
                std::remove( dl_fpath );

            /* push message */
            } else { dl_return = false; }

        }

        /* close directory */
        closedir( dl_directory );

        /* send message */
        return( dl_return );

    }

    bool dl_filter_unity( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_mean, double const dl_factor, int const dl_mode ) {

        /* buffer variables */
        char   * dl_chunk( nullptr );
        double * dl_dists( nullptr );

        /* array mapping variables */
        double * dl_pose1( nullptr );
        double * dl_pose2( nullptr );

        /* indexation variables */
        int64_t dl_delay( 0 );

        /* distance variables */
        double dl_distance( 0.0 );

        /* condition variables */
        double dl_condition( dl_mean * dl_mean * dl_factor * dl_factor );

        /* check size consistency */
        if ( dl_size <= 27 ) {

            /* send message */
            return( true );

        }

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new char[dl_size] ) == nullptr ) {

            /* send message */
            return( false );

        } else {

            /* allocate and check buffer memory */
            if ( ( dl_dists = new double[dl_size/27] ) == nullptr ) {

                /* release buffer memory */
                delete [] dl_chunk;

                /* send message */
                return( false );

            }

        }

        /* initialise array values */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

            /* assign initial value */
            dl_dists[dl_parse] = std::numeric_limits<double>::max();

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* read input stream */
        dl_istream.read( dl_chunk, dl_size );

        /* parsing input stream elements */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size - 27 ); dl_parse < dl_limit; dl_parse += 27 ) {

            /* compute and assign array mapping */
            dl_pose1 = ( double * ) ( dl_chunk + dl_parse );

            /* parsing input stream elements */
            for ( int64_t dl_index( dl_parse + 27 ); dl_index < dl_size; dl_index += 27 ) {

                /* compute and assign array mapping */
                dl_pose2 = ( double * ) ( dl_chunk + dl_index );

                /* compute element-element distance */
                dl_distance = ( dl_pose1[0] - dl_pose2[0] ) * ( dl_pose1[0] - dl_pose2[0] ) +
                              ( dl_pose1[1] - dl_pose2[1] ) * ( dl_pose1[1] - dl_pose2[1] ) +
                              ( dl_pose1[2] - dl_pose2[2] ) * ( dl_pose1[2] - dl_pose2[2] );

                /* search minimal distances */
                if ( dl_distance < dl_dists[dl_parse/27] ) dl_dists[dl_parse/27] = dl_distance;
                if ( dl_distance < dl_dists[dl_index/27] ) dl_dists[dl_index/27] = dl_distance;

            }

        }

        /* analyse mode value */
        if ( dl_mode == DL_FILTER_UNITY_ADAP ) {

            /* reset condition value */
            dl_condition = 0.0;

            /* prepare local threshold condition */
            for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

                /* accumulate local condition */
                dl_condition += sqrt( dl_dists[dl_parse] );

            }

            /* compute local threshold condition */
            dl_condition = dl_condition / ( dl_size / 27 );

            /* compute local threshold condition */
            dl_condition = dl_condition * dl_condition * dl_factor * dl_factor;

        }

        /* reset delayed indexation */
        dl_delay = 0;

        /* apply filtering condition */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

            /* filtering condition */
            if ( dl_dists[dl_parse] < dl_condition ) {

                /* delayed indexation */
                if ( dl_delay < dl_parse ) {

                    /* index filtered element */
                    std::memcpy( dl_chunk + ( dl_delay * 27 ), dl_chunk + ( dl_parse * 27 ), 27 );

                }

                /* update delay */
                dl_delay ++;

            }

        }

        /* exported filtered elements */
        dl_ostream.write( dl_chunk, dl_delay * 27 );

        /* release buffer memory */
        delete [] dl_chunk;
        delete [] dl_dists;

        /* send message */
        return( true );

    }

    bool dl_filter_count_unif( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_mean, double const dl_factor, int64_t const dl_threshold ) {

        /* buffer variables */
        char    * dl_chunk( nullptr );
        int64_t * dl_count( nullptr );

        /* array mapping variables */
        double * dl_pose1( nullptr );
        double * dl_pose2( nullptr );

        /* indexation variables */
        int64_t dl_delay( 0 );

        /* distance variables */
        double dl_distance( 0.0 );

        /* condition variables */
        double dl_condition( dl_mean * dl_mean * dl_factor * dl_factor );

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new char[dl_size] ) == nullptr ) {

            /* send message */
            return( false );

        } else {

            /* allocate and check buffer memory */
            if ( ( dl_count = new int64_t[dl_size / 27] ) == nullptr ) {

                /* release buffer memory */
                delete [] dl_chunk;

                /* send message */
                return( false );

            }

        }

        /* initialise array values */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

            /* assign initial value */
            dl_count[dl_parse] = 0;

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* read input stream */
        dl_istream.read( dl_chunk, dl_size );

        /* parsing input stream elements */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size - 27 ); dl_parse < dl_limit; dl_parse += 27 ) {

            /* compute and assign array mapping */
            dl_pose1 = ( double * ) ( dl_chunk + dl_parse );

            /* parsing input stream elements */
            for ( int64_t dl_index( dl_parse + 27 ); dl_index < dl_size; dl_index += 27 ) {

                /* compute and assign array mapping */
                dl_pose2 = ( double * ) ( dl_chunk + dl_index );

                /* compute element-element distance */
                dl_distance = ( dl_pose1[0] - dl_pose2[0] ) * ( dl_pose1[0] - dl_pose2[0] ) +
                              ( dl_pose1[1] - dl_pose2[1] ) * ( dl_pose1[1] - dl_pose2[1] ) +
                              ( dl_pose1[2] - dl_pose2[2] ) * ( dl_pose1[2] - dl_pose2[2] );

                /* check condition */
                if ( dl_distance < dl_condition ) {

                    /* update element count */
                    dl_count[dl_parse/27] ++;

                    /* update element count */
                    dl_count[dl_index/27] ++;

                }

            }

        }

        /* reset delayed indexation */
        dl_delay = 0;

        /* parsing count array */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

            /* filtering condition */
            if ( dl_count[dl_parse] >= dl_threshold ) {

                /* delayed indexation */
                if ( dl_delay < dl_parse ) {

                    /* index filtered element */
                    std::memcpy( dl_chunk + ( dl_delay * 27 ), dl_chunk + ( dl_parse * 27 ), 27 );

                }

                /* update delay */
                dl_delay ++;

            }

        }

        /* exported filtered elements */
        dl_ostream.write( dl_chunk, dl_delay * 27 );

        /* release buffer memory */
        delete [] dl_chunk;
        delete [] dl_count;

        /* send message */
        return( true );

    }

    bool dl_filter_count_adap( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_mean, double const dl_factor, int64_t const dl_threshold ) {

        /* buffer variables */
        char    * dl_chunk( nullptr );
        double  * dl_dists( nullptr );
        int64_t * dl_count( nullptr );

        /* array mapping variables */
        double * dl_pose1( nullptr );
        double * dl_pose2( nullptr );

        /* indexation variables */
        int64_t dl_delay( 0 );

        /* distance variables */
        double dl_distance( 0.0 );

        /* condition variables */
        double dl_condition( dl_mean * dl_mean * dl_factor * dl_factor );

        /* allocate and check buffer memory */
        if ( ( dl_chunk = new char[dl_size] ) == nullptr ) {

            /* send message */
            return( false );

        } else {

            /* allocate and check buffer memory */
            if ( ( dl_count = new int64_t[dl_size / 27] ) == nullptr ) {

                /* release buffer memory */
                delete [] dl_chunk;

                /* send message */
                return( false );

            } else {

                /* allocate and check buffer memory */
                if ( ( dl_dists = new double[dl_size / 27] ) == nullptr ) {

                    /* release buffer memory */
                    delete [] dl_chunk;
                    delete [] dl_count;

                    /* send message */
                    return( false );

                }

            }

        }

        /* initialise array values */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

            /* assign initial value */
            dl_count[dl_parse] = 0;

            /* assign initial value */
            dl_dists[dl_parse] = std::numeric_limits<double>::max();

        }

        /* clear input stream */
        dl_istream.clear();

        /* reset input stream */
        dl_istream.seekg( 0 );

        /* read input stream */
        dl_istream.read( dl_chunk, dl_size );

        /* parsing input stream elements */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size - 27 ); dl_parse < dl_limit; dl_parse += 27 ) {

            /* compute and assign array mapping */
            dl_pose1 = ( double * ) ( dl_chunk + dl_parse );

            /* parsing input stream elements */
            for ( int64_t dl_index( dl_parse + 27 ); dl_index < dl_size; dl_index += 27 ) {

                /* compute and assign array mapping */
                dl_pose2 = ( double * ) ( dl_chunk + dl_index );

                /* compute element-element distance */
                dl_distance = ( dl_pose1[0] - dl_pose2[0] ) * ( dl_pose1[0] - dl_pose2[0] ) +
                              ( dl_pose1[1] - dl_pose2[1] ) * ( dl_pose1[1] - dl_pose2[1] ) +
                              ( dl_pose1[2] - dl_pose2[2] ) * ( dl_pose1[2] - dl_pose2[2] );

                /* search minimal distances */
                if ( dl_distance < dl_dists[dl_parse/27] ) dl_dists[dl_parse/27] = dl_distance;
                if ( dl_distance < dl_dists[dl_index/27] ) dl_dists[dl_index/27] = dl_distance;

            }

        }

        /* reset condition value */
        dl_condition = 0.0;

        /* prepare local threshold condition */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

            /* accumulate local condition */
            dl_condition += sqrt( dl_dists[dl_parse] );

        }

        /* compute local threshold condition */
        dl_condition = dl_condition / ( dl_size / 27 );

        /* compute local threshold condition */
        dl_condition = dl_condition * dl_condition * dl_factor * dl_factor;

        /* parsing input stream elements */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size - 27 ); dl_parse < dl_limit; dl_parse += 27 ) {

            /* compute and assign array mapping */
            dl_pose1 = ( double * ) ( dl_chunk + dl_parse );

            /* parsing input stream elements */
            for ( int64_t dl_index( dl_parse + 27 ); dl_index < dl_size; dl_index += 27 ) {

                /* compute and assign array mapping */
                dl_pose2 = ( double * ) ( dl_chunk + dl_index );

                /* compute element-element distance */
                dl_distance = ( dl_pose1[0] - dl_pose2[0] ) * ( dl_pose1[0] - dl_pose2[0] ) +
                              ( dl_pose1[1] - dl_pose2[1] ) * ( dl_pose1[1] - dl_pose2[1] ) +
                              ( dl_pose1[2] - dl_pose2[2] ) * ( dl_pose1[2] - dl_pose2[2] );

                /* check condition */
                if ( dl_distance < dl_condition ) {

                    /* update element count */
                    dl_count[dl_parse/27] ++;

                    /* update element count */
                    dl_count[dl_index/27] ++;

                }

            }

        }

        /* reset delayed indexation */
        dl_delay = 0;

        /* parsing count array */
        for ( int64_t dl_parse( 0 ), dl_limit( dl_size / 27 ); dl_parse < dl_limit; dl_parse ++ ) {

            /* filtering condition */
            if ( dl_count[dl_parse] >= dl_threshold ) {

                /* delayed indexation */
                if ( dl_delay < dl_parse ) {

                    /* index filtered element */
                    std::memcpy( dl_chunk + ( dl_delay * 27 ), dl_chunk + ( dl_parse * 27 ), 27 );

                }

                /* update delay */
                dl_delay ++;

            }

        }

        /* exported filtered elements */
        dl_ostream.write( dl_chunk, dl_delay * 27 );

        /* release buffer memory */
        delete [] dl_chunk;
        delete [] dl_dists;
        delete [] dl_count;

        /* send message */
        return( true );

    }

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* temporary path variables */
        char dl_tpath[256];

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* minimums mean variables */
        double dl_mean( 0.0 );

        /* filtering factor variables */
        double dl_factor( lc_read_double( argc, argv, "--factor", "-f", 2.0 ) );

        /* minimums mean count variables */
        int64_t dl_count( lc_read_signed( argc, argv, "--count", "-c", 64 ) );

        /* filtering threshold variables */
        int64_t dl_thres( lc_read_signed( argc, argv, "--threshold", "-t", 4 ) );

        /* returned value variables */
        int dl_return( EXIT_SUCCESS );

        /* create and check temporary storage */
        if ( dl_filter_temporary( dl_tpath, DL_FILTER_CREATE ) != true ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to create temporary storage" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--input", "-i" ), std::ios::in | std::ios::ate | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == true ) {

            /* compute and check minimums mean */
            if ( ( dl_mean = dl_filter_stat( dl_istream, dl_istream.tellg(), dl_count ) ) > 0.0 ) {

                /* create hashed storage */
                if ( dl_filter_hash( dl_istream, dl_tpath, dl_mean ) == true ) {

                    /* create output stream */
                    dl_ostream.open( lc_read_string( argc, argv, "--output", "-o" ), std::ios::out | std::ios::binary );

                    /* check output stream */
                    if ( dl_ostream.is_open() == true ) {

                        /* switch on filtering method */
                        if ( lc_read_flag( argc, argv, "--unity-unif", "-u" ) == LC_TRUE ) {

                            /* filtering method */
                            if ( dl_filter( dl_ostream, dl_tpath, dl_mean, dl_factor, dl_thres, DL_FILTER_UNITY_UNIF ) != true ) {

                                /* push message */
                                dl_return = EXIT_FAILURE;

                            }

                        } else
                        if ( lc_read_flag( argc, argv, "--unity-adap", "-a" ) == LC_TRUE ) {

                            /* filtering method */
                            if ( dl_filter( dl_ostream, dl_tpath, dl_mean, dl_factor, dl_thres, DL_FILTER_UNITY_ADAP ) != true ) {

                                /* push message */
                                dl_return = EXIT_FAILURE;

                            }

                        } else
                        if ( lc_read_flag( argc, argv, "--count-unif", "-n" ) == LC_TRUE ) {

                            /* filtering method */
                            if ( dl_filter( dl_ostream, dl_tpath, dl_mean, dl_factor, dl_thres, DL_FILTER_COUNT_UNIF ) != true ) {

                                /* push message */
                                dl_return = EXIT_FAILURE;

                            }

                        } else
                        if ( lc_read_flag( argc, argv, "--count-adap", "-d" ) == LC_TRUE ) {

                            /* filtering method */
                            if ( dl_filter( dl_ostream, dl_tpath, dl_mean, dl_factor, dl_thres, DL_FILTER_COUNT_ADAP ) != true ) {

                                /* push message */
                                dl_return = EXIT_FAILURE;

                            }

                        } else {

                            /* display message */
                            std::cerr << "dalai-suite : error : unknown filtering mode" << std::endl;

                            /* push message */
                            dl_return = EXIT_FAILURE;

                        }

                        /* check filtering results */
                        if ( dl_return == EXIT_FAILURE ) {

                            /* display message */
                            std::cerr << "dalai-suite : error : unable to apply filter" << std::endl;

                        }

                        /* delete output stream */
                        dl_ostream.close();

                    } else {

                        /* display message */
                        std::cerr << "dalai-suite : error : unable to access output file" << std::endl;

                        /* push message */
                        dl_return = EXIT_FAILURE;

                    }

                } else {

                    /* display message */
                    std::cerr << "dalai-suite : error : unable to create storage" << std::endl;

                    /* push message */
                    dl_return = EXIT_FAILURE;

                }

            } else {

                /* display message */
                std::cerr << "dalai-suite : error : unable to compute statistics" << std::endl;

                /* push message */
                dl_return = EXIT_FAILURE;

            }

            /* delete input stream */
            dl_istream.close();

        } else {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access input file" << std::endl;

            /* push message */
            dl_return = EXIT_FAILURE;

        }

        /* delete temporary storage */
        dl_filter_temporary( dl_tpath, DL_FILTER_DELETE );

        /* send message */
        return( dl_return );

    }

