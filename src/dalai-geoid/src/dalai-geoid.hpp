/*
 *  dalai-suite - geoid
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

    /*! \file   dalai-geoid.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - geoid
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

    # ifndef __DL_GEOID__
    # define __DL_GEOID__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <sys/types.h>
    # include <dirent.h>
    # include <common-include.hpp>
    # include <eratosthene-include.h>
    # include <GeographicLib/Geoid.hpp>

/*
    header - preprocessor definitions
 */

/*
    header - preprocessor macros
 */

    /* define data chunk */
    # define DL_CHUNK ( le_size_l( 37449 ) * LE_ARRAY_DATA )

/*
    header - type definition
 */

/*
    header - structures
 */

/*
    header - function prototypes
 */

    /*! \brief conversion methods
     *
     *  This function converts the height of the uv3 file provided through the
     *  input path from MSL to ellipsoidal height and inversely.
     *
     *  The input path points at the source file while the output path points
     *  at the output file receiving the converted data.
     *
     *  The provided geoid structure hold the definition of the geoid to use
     *  and ensure interpolation process (see GeographicLib).
     *
     *  The conversion flag has to be +1.0 for MSL to ellipsoidal height
     *  conversion and -1.0 for the invert conversion.
     *
     *  \param dl_input      Input stream path (file)
     *  \param dl_output     Output stream path (file)
     *  \param dl_geoid      Geoid structure (GeographicLib)
     *  \param dl_conversion Conversion mode value (+1.0 or -1.0)
     */

    le_void_t dl_geoid_height( le_char_t const * const dl_input, le_char_t const * const dl_output, GeographicLib::Geoid & dl_geoid, le_real_t const dl_conversion );

    /*! \brief batch methods
     *
     *  This function allows to apply a conversion batch processing on a folder
     *  containing multiple uv3 files.
     *
     *  The function list and filter the uv3 files from the input directory and
     *  export the conversion result in the output directory using the same file
     *  name. Then, both input and output provided path have to point to valid
     *  directories.
     *
     *  The conversion flag has to be +1.0 for MSL to ellipsoidal height
     *  conversion and -1.0 for the invert conversion.
     *
     *  \param dl_input      Input stream path (directory)
     *  \param dl_output     Output stream path (directory)
     *  \param dl_geoid      Geoid structure (GeographicLib)
     *  \param dl_conversion Conversion mode value (+1.0 or -1.0)
     */

    le_void_t dl_geoid_batch( le_char_t const * const dl_input, le_char_t const * const dl_output, GeographicLib::Geoid & dl_geoid, le_real_t const dl_conversion );

    /*! \brief main methods
     *
     *  The main function reads the specified input uv3 stream and apply a geoid
     *  correction on the heights of the vertices :
     *
     *      ./dalai-geoid --name/-n   [geoid model name]
     *                    --path/-p   [geoid model storage directory]
     *                    --input/-i  [input directive]
     *                    --output/-o [output directive]
     *                    --to-msl/-m [correction directive]
     *                    --to-ell/-e [correction directive]
     *
     *  The geoid model used to apply the height correction on the vertex is
     *  expected to be in PGM format (see GeographicLib for more information).
     *  The '.pgm' extension is added to the name of the geoid and the path is
     *  used to find the model storage file.
     *
     *  If the input directive points to a regular file, it is processed by the
     *  main function and exported in the file pointed by the output directive.
     *  In such a case, the output directive has to point to a file.
     *
     *  If the input directive points to a directory, the main function list the
     *  uv3 files hold in the directory and process each of them. The output
     *  directive has to point to another directory in which the converted uv3
     *  files are exported. The name of the input files is kept to name the
     *  output files in such case.
     *
     *  If the '--to-msl' conversion directive is provided, the vertex heights
     *  are converted from ellipsoidal height to MSL height. If the '--to-ell'
     *  directive is provided, the opposite appends.
     *
     *  Most of the time, geoid model are provided in WGS84 frame. The uv3
     *  files vertices have then to be expressed in this reference frame. If
     *  vertices are expressed in another datum, the geoid model has to be
     *  adapted to this datum.
     *
     *  Finally, the longitudes and latitudes of the uv3 files are expected to
     *  be angles expressed in radian. 
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

