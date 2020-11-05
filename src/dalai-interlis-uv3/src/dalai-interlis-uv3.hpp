/*
 *  dalai-suite - interlis-uv3
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

    /*! \file   dalai-interlis-uv3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - interlis-uv3
     */

    /*! \mainpage dalai-suite
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2020 DHLAB, EPFL
     * 
     *  This program is licensed under the terms of the GNU GPLv3. Documentation
     *  and illustrations are licensed under the terms of the CC BY 4.0.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_INTERLIS_UV3__
    # define __DL_INTERLIS_UV3__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <string>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

/*
    header - preprocessor macros
 */

/*
    header - type definition
 */

/*
    header - structures
 */

/*
    header - function prototypes
 */

    /*! \brief buffer methods
     *
     *  This function is used to initialise the content of the provided uv3
     *  record buffer. It simply sets the position to zeros and assign the
     *  provided color. In addition, it also sets the primitive type to two
     *  expecting only lines from INTERLIS format.
     *
     *  \param dl_buffer uv3 record buffer
     *  \param dl_red    uv3 record red color
     *  \param dl_green  uv3 record green color
     *  \param dl_blue   uv3 record blue color
     */

    void dl_interlis_init( le_byte_t * dl_buffer, le_byte_t const dl_red, le_byte_t const dl_green, le_byte_t const dl_blue );

    /*! \brief buffer methods
     *
     *  This function is used to assign a planimetric position to the provided
     *  uv3 record buffer before to export it in the provided stream. The
     *  provided stream has to be an already open stream in binary mode.
     *
     *  \param dl_buffer uv3 record buffer
     *  \param dl_stream Exportation stream
     *  \param dl_x      Planimetric x position
     *  \param dl_y      Planimetric y position
     */

    void dl_interlis_export( le_byte_t * const dl_buffer, std::ofstream & dl_stream, le_real_t const dl_x, le_real_t const dl_y );

    /*! \brief main methods
     *
     *  The main function opens and reads the provided INTERLIS file and extract
     *  the targeted data before to convert and export them in uv3 format :
     *
     *      ./dalai-interlis-uv3 --input/-i [input file]
     *                           --output/-o [output file]
     *                           --red/-r [uv3 record color]
     *                           --green/-g [uv3 record color]
     *                           --blue/-b [uv3 record color]
     *                           --topic/-t [interlis target topic]
     *                           --table/-a [interlis target table]
     *
     *  The main function starts by locating the target topic in the provided
     *  interlis data file. In the found topic, the target table is searched.
     *  The content of the table is then read object by object to extract the
     *  geometry.
     *
     *  The geometry is converted into uv3 lines primitive before to be exported
     *  in the output uv3 file. The provided color is used to assign the uv3
     *  primitives color.
     *
     *  \param  argc Standard parameter
     *  \param  argv Standard parameter
     *
     *  \return Return standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

