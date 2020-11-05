/*
 *  dalai-suite - color
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

    /*! \file   dalai-color.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - color
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

    # ifndef __DL_COLOR__
    # define __DL_COLOR__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <cmath>
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

    /*! \brief color mapping methods
     *
     *  This function assigns a color value to the provided color vector based
     *  on the provided height value. It starts by clamping the height value
     *  using the provided boundaries. The heights is then normalised on this
     *  interval, using a cyclic condition when the height is outside of the
     *  range, and the color is computed following the implemented colormap.
     *
     *  \param dl_height Element height
     *  \param dl_data   Element color array
     *  \param dl_ledge  Height clamping range lower boundary
     *  \param dl_hedge  Height clamping range upper boundary
     */

    le_void_t dl_color( le_real_t dl_height, le_data_t * const dl_data, le_real_t const dl_ledge, le_real_t const dl_hedge );

    /*! \brief main methods
     *
     *  The main function reads the vertex of the primitives of the provided uv3
     *  file and overrides their color using an height-based colormap :
     *
     *      ./dalai-color --input/-i [input file]
     *                    --output/-o [output file]
     *                    --minimum/-m [height low boundary]
     *                    --maximum/-x [height high boundary]
     *
     *  The resulting colored uv3 primitives are exported in the provided output
     *  stream.
     *
     *  The provided maximum and minimum heights are used to set the cyclic
     *  boundaries of the applied colormap. The colormaps starts with its first
     *  color on the minimum heights and ends with its last color on the given
     *  maximum height.
     *
     *  The vertex height is understood as the value stored by their third
     *  coordinate. If the third component is outside of the provided range, a
     *  cyclic condition is considered.
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

