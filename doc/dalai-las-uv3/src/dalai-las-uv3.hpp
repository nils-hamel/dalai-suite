/*
 *  dalai-suite - las-uv3
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

    /*! \file   dalai-las-uv3.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - las-uv3
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
     *      ./dalai-las-uv3 --input/-i [las input file]
     *                      --output/-o [uv3 output file]
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

