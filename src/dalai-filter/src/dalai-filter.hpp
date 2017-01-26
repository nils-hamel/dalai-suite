/*
 *  dalai-suite - filter
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

    /*! \file   dalai-filter.hpp
     *  \author Nils Hamel <nils.hamel@bluewin.ch>
     *
     *  dalai-suite - filter
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
     *  This standardised format allows to use the suite tools for colour
     *  edition, model cleaning and model hashing. In addition, the standard
     *  format is also expected by the _eratosthene-suite_ implementing the EPFL
     *  CDH DHLAB indexation server and its geographical 3-dimensional data
     *  injection tools.
     *
     *  \section _2 Copyright and License
     *
     *  **dalai-suite** - Nils Hamel <br >
     *  Copyright (c) 2016-2017 EPFL CDH DHLAB
     *
     *  This program is licensed under the terms of the GNU GPLv3.
     */

/*
    header - inclusion guard
 */

    # ifndef __DL_FILTER__
    # define __DL_FILTER__

/*
    header - internal includes
 */

/*
    header - external includes
 */

    # include <iostream>
    # include <fstream>
    # include <sstream>
    # include <cmath>
    # include <limits>
    # include <cstring>
    # include <cstdlib>
    # include <cstdint>
    # include <unistd.h>
    # include <dirent.h>
    # include <common-include.h>

/*
    header - preprocessor definitions
 */

    /* define chunk size */
    # define DL_FILTER_CHUNK      ( 2097152ll )

    /* define storage modes */
    # define DL_FILTER_CREATE     ( 0 )
    # define DL_FILTER_DELETE     ( 1 )

    /* define filtering modes */
    # define DL_FILTER_UNITY_UNIF ( 0x00 )
    # define DL_FILTER_UNITY_ADAP ( 0x01 )
    # define DL_FILTER_COUNT_UNIF ( 0x02 )
    # define DL_FILTER_COUNT_ADAP ( 0x03 )

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

    /*! \brief storage methods
     *
     *  As \b DL_FILTER_CREATE is provided as mode, the function asks the system
     *  for an unused temporary directory in the system temporary volume. On
     *  success, the function creates the temporary directory and updates the
     *  content of the provided path with the created directory path.
     *
     *  If the provided mode is \b DL_FILTER_DELETE, the function simply removes
     *  the temporary directory provided by the \b dl_path parameter. It uses
     *  the \b rmdir function assuming the temporary directory is already empty.
     *  An error is reported otherwise.
     *
     *  \param  dl_path Temporary storage directory path
     *  \param  dl_mode Temporary storage mode
     *
     *  \return Returns true in success, false otherwise
     */

    bool dl_filter_temporary( char * const dl_path, int dl_mode );

    /*! \brief hashing methods
     *
     *  This function imports the point cloud provided by the input stream and
     *  hashes it in the output directory. The hashing consists in cutting the
     *  provided point cloud into many smaller point clouds.
     *
     *  The function reads the input stream elements one by one and uses the
     *  provided minimums mean value to determine in which sub point cloud each
     *  element has to be written. The hash function is driven by the following
     *  values computation :
     *
     *      h_i = ( int ) floor( p_i / ( 100 * dl_mean ) )
     *
     *  with i = x,y,z. The three computed h_i values are then used to composed
     *  the sub point clouds file name. All elements sharing the same h_i values
     *  are then written in the same sub-point-cloud.
     *
     *  \param  dl_istream Input stream descriptor
     *  \param  dl_opath   Output path
     *  \param  dl_mean    Minimums mean value
     *
     *  \return Returns \b true on success, \b false otherwise
     */

    bool dl_filter_hash( std::ifstream & dl_istream, char const * const dl_opath, double const dl_mean );

    /*! \brief statistical methods
     *
     *  This function computes and returns the mean value of the point cloud
     *  elements distance to their closest element. To achieve this computation
     *  in a reasonable amount of time, the following strategy is considered.
     *
     *  The function starts by sampling \b dl_count elements in the point cloud
     *  provided through the stream descriptor. For each sampled element, it
     *  searches the distance to its closest element. It finally computes the
     *  mean value of the found minimal distances on the sampled set.
     *
     *  The approximation of the minimums mean value gets better as \b dl_count
     *  increases. Nevertheless, a value of 32 already allows to compute a very
     *  good approximation of the minimums mean value.
     *
     *  \param  dl_istream Input stream descriptor
     *  \param  dl_size    Size, in bytes, of the input stream
     *  \param  dl_count   Number of sampled elements
     *
     *  \return Returns mean minimum distance on success, 0.0 otherwise
     */

    double dl_filter_stat( std::ifstream & dl_istream, int64_t const dl_size, int64_t const dl_count );

    /*! \brief filtering methods
     *
     *  This function applies the filtering process pointed by the mode value
     *  on each file contained in the provided \b dl_opath directory. It simply
     *  enumerates the files of the directory and applies the filtering process
     *  on each one. The resulting filtered point clouds are all exported in the
     *  output stream.
     *
     *  Usually, this function is used on the directory in which hashed files
     *  produced by the \b dl_filter_hash() are stored. Indeed, the filtering
     *  process expects all the input files to be part of the same initial point
     *  cloud, so sharing the same minimums mean value.
     *
     *  Depending on the provided mode value, a specific filtering process is
     *  applied on all the files contained in the \b dl_opath directory. The
     *  filtering parameters are then provided to the different filtering
     *  processes according to their necessities :
     *
     *  \b dl_filter_unity()
     *  \b dl_filter_count_unif()
     *  \b dl_filter_count_adap()
     *
     *  All the filtering processes share a similar strategy, each implementing
     *  a variation of it.
     *
     *  \param  dl_ostream   Output stream descriptor
     *  \param  dl_opath     Input streams directory path
     *  \param  dl_mean      Minimums mean value
     *  \param  dl_factor    Minimums mean multiplier
     *  \param  dl_threshold Neighbour count threshold
     *  \param  dl_mode      Filtering method mode
     *
     *  \return Returns true on success, false otherwise
     */

    bool dl_filter( std::ofstream & dl_ostream, char const * const dl_opath, double const dl_mean, double const dl_factor, int64_t const dl_threshold, int const dl_mode );

    /*! \brief filtering methods
     *
     *  This filtering function reads the point cloud provided through the input
     *  stream and exports its filtered version in the output stream.
     *
     *  The filtering process consists in computing for each element of the
     *  point cloud the distance to its closest neighbour. Elements that have
     *  a minimal distance verifying :
     *
     *      element_minimal_distance < ( dl_mean * dl_factor )
     *
     *  are kept and written in the output stream. The other elements are simply
     *  discarded. The minimums mean \b dl_mean is expected to be computed
     *  considering the initial point cloud, the input stream being expected to
     *  be a piece of the hashed initial point cloud. The function provides the
     *  previous filtering as the mode parameter is \b DL_FILTER_UNITY_UNIF.
     *
     *  If the \b DL_FILTER_UNITY_ADAP is provided, the filtering method is
     *  similar except that the minimums mean value is recomputed considering
     *  only the elements contained in the input stream point cloud. This allows
     *  to adapt the criterion to the local specificities of the initial point
     *  cloud.
     *
     *  \param  dl_istream Input stream descriptor
     *  \param  dl_ostream Output stream descriptor
     *  \param  dl_size    Size, in bytes, of the input stream
     *  \param  dl_mean    Minimums mean value
     *  \param  dl_factor  Minimums mean multiplier
     *  \param  dl_mode    Filtering method mode
     *
     *  \return Returns true on success, false otherwise
     */

    bool dl_filter_unity( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_mean, double const dl_factor, int const dl_mode );

    /*! \brief filtering methods
     *
     *  This filtering function reads the point cloud provided through the input
     *  stream and exports its filtered version in the output stream.
     *
     *  This function implements a variation of filtering methods implemented
     *  in \b dl_filter_unity() using the \b DL_FILTER_UNITY_UNIF mode value. In
     *  addition to the distance criterion, this function only keeps elements
     *  for which the criterion is true for at least \b dl_threshold of their
     *  neighbour elements.
     *
     *  \param  dl_istream   Input stream descriptor
     *  \param  dl_ostream   Output stream descriptor
     *  \param  dl_size      Size, in bytes, of the input stream
     *  \param  dl_mean      Minimums mean value
     *  \param  dl_factor    Minimums mean multiplier
     *  \param  dl_threshold Neighbour count threshold
     *
     *  \return Returns true on success, false otherwise
     */

    bool dl_filter_count_unif( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_mean, double const dl_factor, int64_t const dl_threshold );

    /*! \brief filtering methods
     *
     *  This filtering function reads the point cloud provided through the input
     *  stream and exports its filtered version in the output stream.
     *
     *  This function implements a variation of filtering methods implemented
     *  in \b dl_filter_unity() using the \b DL_FILTER_UNITY_ADAP mode value. In
     *  addition to the distance criterion, this function only keeps elements
     *  for which the criterion is true for at least \b dl_threshold of their
     *  neighbour elements.
     *
     *  \param  dl_istream   Input stream descriptor
     *  \param  dl_ostream   Output stream descriptor
     *  \param  dl_size      Size, in bytes, of the input stream
     *  \param  dl_mean      Minimums mean value
     *  \param  dl_factor    Minimums mean multiplier
     *  \param  dl_threshold Neighbour count threshold
     *
     *  \return Returns true on success, false otherwise
     */

    bool dl_filter_count_adap( std::ifstream & dl_istream, std::ofstream & dl_ostream, int64_t const dl_size, double const dl_mean, double const dl_factor, int64_t const dl_threshold );

    /*! \brief main function
     *
     *  The main function reads the point cloud provided through the input file
     *  and exports its filtered version in the provided output stream :
     *
     *      ./dalai-filter --input/-i [input file path]
     *                     --output/-o [output file path]
     *                     --unity-unif/--unity-adap
     *                     --count-unif/--count-adap [filtering mode switches]
     *                     --factor/-f [minimums mean multiplier]
     *                     --count/-c [sampled elements for mean computation]
     *                     --threshold/-t [neighbour elements count threshold]
     *
     *  The main function starts by reading the parameters and opens the input
     *  file. It computes the point cloud elements minimum distances mean value
     *  using the count parameter.
     *
     *  In order to performs the filtering process, the input point cloud is
     *  cut in smallest pieces using the hashing function. The hashing function
     *  uses the minimums mean to adapt the hash to the input file point cloud.
     *
     *  The filtering process is then applied on each piece of the hashed point
     *  cloud and the results of the filtering of all the pieces is exported in
     *  the output file.
     *
     *  \param argc Standard parameter
     *  \param argv Standard parameter
     *
     *  \return Returns standard exit code
     */

    int main( int argc, char ** argv );

/*
    header - inclusion guard
 */

    # endif

