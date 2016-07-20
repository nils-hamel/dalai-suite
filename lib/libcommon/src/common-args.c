/*
 *  dalai-suite - geodetic system
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016 EPFL CDH DHLAB
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

    # include "common-args.h"

/*
    source - arguments and parameters parsers
 */

    char * lc_read_string( int const argc, char ** argv, char const * const er_long, char const * const er_short ) {

        /* Parses arguments and parameters */
        for ( int er_parse = 0; er_parse < argc; er_parse ++ ) {

            /* Check argument */
            if ( ( strcmp( argv[er_parse], er_long ) == 0 ) || ( strcmp( argv[er_parse], er_short ) == 0 ) ) {

                /* Check consistency */
                if ( ( ++ er_parse ) < argc ) {

                    /* Return pointer */
                    return( argv[er_parse] );

                /* Return pointer */
                } else { return( NULL ); }

            }

        }

        /* Return pointer */
        return( NULL );

    }

    unsigned long long int lc_read_uint( int const argc, char ** argv, char const * const er_long, char const * const er_short, unsigned long long int er_default ) {
        
        /* Parses arguments and parameters */
        for( int er_parse = 0; er_parse < argc; er_parse ++ ) {

            /* Check argument */
            if ( ( strcmp( argv[er_parse], er_long ) == 0 ) || ( strcmp( argv[er_parse], er_short ) == 0 ) ) {

                /* Check consistency */
                if ( ( ++ er_parse ) < argc ) {

                    /* Return read value */
                    return( strtoull( argv[er_parse], NULL, 10 ) );

                /* Return default value */
                } else { return( er_default ); }

            }

        }

        /* Return default value */
        return( er_default );

    }

    signed long long int lc_read_int( int const argc, char ** argv, char const * const er_long, char const * const er_short, signed long long int er_default ) {
        
        /* Parses arguments and parameters */
        for( int er_parse = 0; er_parse < argc; er_parse ++ ) {

            /* Check argument */
            if ( ( strcmp( argv[er_parse], er_long ) == 0 ) || ( strcmp( argv[er_parse], er_short ) == 0 ) ) {

                /* Check consistency */
                if ( ( ++ er_parse ) < argc ) {

                    /* Return read value */
                    return( strtoll( argv[er_parse], NULL, 10 ) );

                /* Return default value */
                } else { return( er_default ); }

            }

        }

        /* Return default value */
        return( er_default );

    }

