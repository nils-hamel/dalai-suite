/*
 *  dalai-suite - common library
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
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

    # include "common-temp.hpp"

/*
    source - temporary methods
 */

    void lc_temp_directory( char const * const lc_root, char * const lc_path, int const lc_mode ) {

        /* check mode */
        if ( lc_mode == LC_TEMP_CREATE ) {

            if ( lc_root == nullptr ) {

                /* compose temporary path */
                sprintf( ( char * ) lc_path, "/tmp/temp-XXXXXX" );

            } else {

                /* compose temporary path */
                sprintf( ( char * ) lc_path, "%s/temp-XXXXXX", lc_root );

            }

            /* create temporary directory */
            if ( mkdtemp( ( char * ) lc_path ) == nullptr ) {

                /* send message */
                throw( LC_ERROR_IO_ACCESS );

            }

        } else {

            /* delete temporary directory */
            if ( rmdir( ( char * ) lc_path ) != 0 ) {

                /* send message */
                throw( LC_ERROR_IO_REMOVE );

            }

        }

    }
