/*
 *  dalai-suite - las-uv3
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

    /*! \file   dalai-las-uv3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - las-uv3
     */

    /*! \mainpage dalai-suite
     *
     *  \section _1 dalai-suite
     *
     *  The _dalai-suite_ is dedicated to the gathering and processing of
     *  geographical 3-dimensional information. It allows to considers the most
     *  common file formats and to convert them in a standardised and simple
     *  format.
     *
     *  This standardised format allows to use the suite tools for color
     *  edition, model cleaning and model hashing. In addition, the standard
     *  format is also expected by the _eratosthene-suite_ implementing the EPFL
     *  CDH DHLAB indexation server and its geographical 3-dimensional data
     *  injection tools.
     *
     *  \section _2 Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2018 DHLAB, EPFL
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

    /* define classification colormap */
    # define DL_COLORMAP { \
        {  64,  64,  64 }, \
        { 128, 128, 128 }, \
        {  51,  64,  55 }, \
        {  37,  64,  37 }, \
        {  74, 127,  75 }, \
        { 111, 191, 112 }, \
        { 255, 138,  40 }, \
        { 191,  52,  49 }, \
        {  64,  17,  16 }, \
        {  60,  98, 191 }, \
        { 229, 188,  61 }, \
        { 255, 209,  68 }, \
        {  64,  17,  16 }, \
        { 255, 194,  54 }, \
        { 232, 158,  50 }, \
        { 255, 153,  67 }, \
        { 232, 106,  50 }, \
        { 255,  88,  54 }, \
        { 255,  62,  52 }  \
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
     *                      --classification/-c [forced classification]
     *
     *  The function starts by reading the input file header in order to detect
     *  if colors are provided. If colors are not provided by the input file,
     *  the main function reads each point classification value and considers a
     *  colormap to assign colors to points exported in the output stream.
     *
     *  The function allows to force usage of classification values and colormap
     *  even if colors are available by using the last argument (as a switch).
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

