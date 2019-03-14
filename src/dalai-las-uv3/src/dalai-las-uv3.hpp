/*
 *  dalai-suite - las-uv3
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

    /*! \file   dalai-las-uv3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - las-uv3
     */

    /*! \mainpage dalai-suite
     *
     *  \section overview Overview
     *
     *  The _dalai-suite_ is part of the Eratosthene Project and is dedicated to
     *  3D models gathering, processing and format conversion. It allows to
     *  consider the most common formats and to convert them into the project
     *  common format. The _dalai-suite_ is mainly used for data gathering and
     *  processing, allowing to make them compatible with remote Eratosthene
     *  servers injection format.
     *
     *  In addition, the project common format allows to apply the _dalai-suite_
     *  solutions for 3D models processing. The suite offers solution for
     *  massive 3D models cleaning, hashing and assisted geographical
     *  registration. The suite also provides solution for 3D models display and
     *  analysis.
     *
     *  \section copyright Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2019 DHLAB, EPFL
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
    # include <liblas/liblas.hpp>
    # include <common-include.hpp>
    # include <eratosthene-include.h>

/*
    header - preprocessor definitions
 */

    /* define extraction mode */
    # define DL_EXTRACT_CLASS 0
    # define DL_EXTRACT_COLOR 1
    # define DL_EXTRACT_INTEN 2

    /* define classification colormap */
    # define DL_COLORMAP { \
    {  64,  64,  64 }, \
    { 128, 128, 128 }, \
    {  51,  64,  55 }, \
    {  58, 111,  59 }, \
    {  74, 127,  75 }, \
    {  90, 143,  91 }, \
    { 255, 138,  40 }, \
    { 191,  52,  49 }, \
    {  64,  64,  64 }, \
    {  60,  98, 191 }, \
    { 229, 188,  61 }, \
    { 255, 209,  68 }, \
    {  64,  64,  64 }, \
    { 255, 194,  54 }, \
    { 232, 158,  50 }, \
    { 255, 153,  67 }, \
    { 232, 106,  50 }, \
    { 255,  88,  54 }, \
    { 191,  52,  49 }  \
    }

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
     *  it into uv3 stream :
     *
     *      ./dalai-las-uv3 --las/-i [las input file]
     *                      --uv3/-o [uv3 output file]
     *                      --classification/-c [extraction switch]
     *                      --color/-r [extraction switch]
     *                      --intensity/-e [extraction switch]
     *
     *  The main function starts by reading the input file header. Depending on
     *  the provided extraction switch, the availability of the desired data
     *  components is checked. The file content is read and converted into the
     *  uv3 format before to be written in the output stream.
     *
     *  If no extraction switch is provided, the classification is assumed for
     *  extraction. If multiple extraction switch are provided, only the first
     *  one is considered.
     *
     *  \param  argc Main function parameters
     *  \param  argv Main function parameters
     *
     *  \return Returns standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

