/*
 *  dalai-suite - hash
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

    /*! \file   dalai-hash.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - hash
     */

    /*! \mainpage dalai-suite
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2020 DHLAB, EPFL
     *  Copyright (c) 2020 Republic and Canton of Geneva
     * 
     *  This program is licensed under the terms of the GNU GPLv3. Documentation
     *  and illustrations are licensed under the terms of the CC BY 4.0.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_HASH__
    # define __DL_HASH__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
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

    /*! \brief main function
     *
     *  The main function hashes the provided uv3 files into a set of smaller
     *  uv3 sub-files stored in the output directory :
     *
     *      ./dalai-hash --input/-i [input uv3 file path]
     *                   --output/-o [output path directory]
     *                   --count/-c [sampled elements count]
     *                   --parameter/-p [hashing parameter]
     *
     *  The functions starts by gathering the parameters and opens the provided
     *  input file. It computes the file model minimum distances mean value and
     *  provides it to the \b lc_hash() function used to hash the model. The
     *  resulting hashed sub-files are all written in the provided output
     *  directory considering the uv3 format.
     *
     *  The hashing process is based on a floating point congruence applied on
     *  the three coordinates of the input file vertex. The congruence is also
     *  used to name the different pieces output files resulting of the hash
     *  process. See \b libcommon documentation for more information.
     *
     *  The count value gives the amount of input file uv3 records to consider
     *  to compute the minimum distances mean value. The hashing parameter is
     *  used with the mean value to determine the size of the sub-files.
     *
     *  \param  argc Standard parameter
     *  \param  argv Standard parameter
     *
     *  \return Returns standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

