/*
 *  dalai-suite - common library
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

    # include "common-temp.hpp"

/*
    source - temporary methods
 */

    void lc_temp_directory( char const * const dl_root, char * const dl_path, int const dl_mode ) {

        /* check mode */
        if ( dl_mode == LC_TEMP_CREATE ) {

            if ( dl_root == nullptr ) {

                /* compose temporary path */
                sprintf( dl_path, "/tmp/temp-XXXXXX" );

            } else {

                /* compose temporary path */
                sprintf( dl_path, "%s/temp-XXXXXX", dl_root );

            }

            /* create temporary directory */
            if ( mkdtemp( dl_path ) == nullptr ) {

                /* send message */
                throw( LC_ERROR_IO_ACCESS );

            }

        } else {

            /* delete temporary directory */
            if ( rmdir( dl_path ) != 0 ) {

                /* send message */
                throw( LC_ERROR_IO_REMOVE );

            }

        }

    }
