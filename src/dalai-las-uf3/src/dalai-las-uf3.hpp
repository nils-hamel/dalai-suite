/*
 *  dalai-suite - las-uf3
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

    /*! \file   dalai-las-uf3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - las-uf3
     */

    /*! \mainpage dalai-suite
     *
     *  \section dalai-suite
     *
     *  The _dalai-suite_ is designed to be a gathering and standardising
     *  interface to the geodetic system developed at the DHLAB of EPFL. It
     *  consists in a set of softwares having to convert any kind of geodetic
     *  data, mainly point clouds, into the defined universal format considered
     *  in the geodetic system. In other words, it is well described by its
     *  tibetan name.
     *
     *  \section Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2017 EPFL CDH DHLAB
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_LAS_UF3__
    # define __DL_LAS_UF3__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <cstdint>
    # include <common-args.h>
    # include <liblas/liblas.hpp>

/*
    header - preprocessor definitions
 */

    /* define classification colormap */
    # define DL_COLORMAP { { 64, 64, 64 }, { 128, 128, 128 }, { 51, 64, 55 }, { 37, 64, 37 }, { 74, 127, 75 }, { 111, 191, 112 }, { 255, 138, 40 }, { 191, 52, 49 }, { 64, 17, 16 }, { 60, 98, 191 }, { 229, 188, 61 }, { 255, 209, 68 }, { 64, 17, 16 }, { 255, 194, 54 }, { 232, 158, 50 }, { 255, 153, 67 }, { 232, 106, 50 }, { 255, 88, 54 }, { 255, 62, 52 } }

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
     *  The main function converts the provided las (asprs) file and converts
     *  it into a universal format 3 file :
     *
     *      ./dalai-las-uf3 --las/-i [las_file]
     *                      --uf3/-o [uf3_file]
     *                      --classification/-c
     *
     *  The function starts by reading the input file header in order to detect
     *  if colours are provided. If colours are not provided by the input file,
     *  the main function reads each point classification value and considers a
     *  colormap to assign colours to points exported in the output file.
     *
     *  The function allows to force usage of classification values and colormap
     *  even if colours are available by using the last argument (as a switch).
     *
     *  \param argc Main function parameters
     *  \param argv Main function parameters
     *
     *  \return Standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

