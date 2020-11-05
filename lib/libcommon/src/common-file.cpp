/*
 *  dalai-suite - common library
 *
 *      Nils Hamel - nils.hamel@alumni.epfl.ch
 *      Copyright (c) 2016-2020 DHLAB, EPFL
 *      Copyright (c) 2020 Republic and Canton of Geneva
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

    # include "common-file.hpp"

/*
    source - filesystem methods
 */

    int lc_file_detect( char const * const lc_path ) {

        /* entity structure variable */
        struct stat lc_entity;

        /* read entity attributes */
        stat( lc_path, & lc_entity );

        /* entity detection */
        if ( lc_entity.st_mode & S_IFDIR ) {

            /* send message */
            return( LC_DIRECTORY );

        }

        /* entity detection */
        if ( lc_entity.st_mode & S_IFREG ) {

            /* send message */
            return( LC_FILE );

        }

        /* send message */
        return( LC_OTHER );

    }
