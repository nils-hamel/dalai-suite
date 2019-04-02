/*
 *  dalai-suite - common library
 *
 *      Nils Hamel - nils.hamel@bluewin.ch
 *      Copyright (c) 2016-2019 DHLAB, EPFL
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

    # include "common-error.hpp"

/*
    source - error methods
 */

    void lc_error( int lc_code ) {

        /* display format */
        std::cerr << "dalai-suite : error : ";

        /* switch on error code */
        switch ( lc_code ) {

            /* error message */
            case ( LC_ERROR_MEMORY ) : {

                /* display message */
                std::cerr << "unable to allocate memory";

            } break;

            /* error message */
            case ( LC_ERROR_IO_ACCESS ) : {

                /* display message */
                std::cerr << "unable to access element";

            } break;

            /* error message */
            case ( LC_ERROR_IO_READ ) : {

                /* display message */
                std::cerr << "unable to read stream";

            } break;

            /* error message */
            case ( LC_ERROR_IO_WRITE ) : {

                /* display message */
                std::cerr << "unable to write stream";

            } break;

            /* error message */
            case ( LC_ERROR_IO_REMOVE ) : {

                /* display message */
                std::cerr << "unable to remove file system entity";

            } break;

            /* error message */
            case ( LC_ERROR_FORMAT ) : {

                /* display message */
                std::cerr << "unable to handle format";

            } break;

            /* error message */
            case ( LC_ERROR_CONTEXT ) : {

                /* display message */
                std::cerr << "unable to create rendering context";

            } break;

            /* error message */
            case ( LC_ERROR_DOMAIN ) : {

                /* display message */
                std::cerr << "parameter out of domain";

            } break;

            /* error message */
            case ( LC_ERROR_RANDOM ) : {

                /* display message */
                std::cerr << "standard random range";

            } break;

            /* error message */
            case ( LC_ERROR_CONVERT ) : {

                /* display message */
                std::cerr << "conversion parameters";

            } break;

        };

        /* display format */
        std::cerr << std::endl;

    }

