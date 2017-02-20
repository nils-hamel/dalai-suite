/*
 *  dalai-suite - uf3-ply
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

    # include "dalai-uf3-ply.hpp"

/*
    source - main methods
 */

    int main( int argc, char ** argv ) {

        /* stream size variables */
        long long dl_size( 0 );

        /* stream variables */
        std::ifstream dl_istream;
        std::ofstream dl_ostream;

        /* i/o buffer variables */
        char * dl_ibuffer( nullptr );
        char * dl_obuffer( nullptr );

        /* i/o buffer mapping variables */
        lc_uf3p_t * dl_ipose( nullptr );
        dl_plyp_t * dl_opose( nullptr );
        lc_uf3d_t * dl_idata( nullptr );
        dl_plyd_t * dl_odata( nullptr );

        /* allocate and check i/o buffer memory */
        if ( ( dl_ibuffer = new ( std::nothrow ) char[DL_UF3_PLY_CHUNK * LC_UF3_RECLEN] ) == nullptr ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to allocate memory" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* allocate and check i/o buffer memory */
        if ( ( dl_obuffer = new ( std::nothrow ) char[DL_UF3_PLY_CHUNK * DL_UF3_PLY_RECLEN] ) == nullptr ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to allocate memory" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* create input stream */
        dl_istream.open( lc_read_string( argc, argv, "--uf3", "-i" ), std::ios::ate | std::ios::in | std::ios::binary );

        /* check input stream */
        if ( dl_istream.is_open() == false ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access input stream" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* retrieve input stream size */
        dl_size = dl_istream.tellg() / LC_UF3_RECLEN;

        /* reset input stream position */
        dl_istream.seekg( 0, std::ios::beg );

        /* create output stream */
        dl_ostream.open( lc_read_string( argc, argv, "--ply", "-o" ), std::ios::out | std::ios::binary );

        /* check output stream */
        if ( dl_ostream.is_open() == false ) {

            /* display message */
            std::cerr << "dalai-suite : error : unable to access output stream" << std::endl;

            /* send message */
            return( EXIT_FAILURE );

        }

        /* export ply header */
        dl_ostream << "ply"                             << std::endl;
        dl_ostream << "format binary_little_endian 1.0" << std::endl;
        dl_ostream << "element vertex " << dl_size      << std::endl;
        dl_ostream << "property float x"                << std::endl;
        dl_ostream << "property float y"                << std::endl;
        dl_ostream << "property float z"                << std::endl;
        dl_ostream << "property uchar red"              << std::endl;
        dl_ostream << "property uchar green"            << std::endl;
        dl_ostream << "property uchar blue"             << std::endl;
        dl_ostream << "end_header"                      << std::endl;

        /* conversion processing loop */
        do {

            /* read input stream chunk */
            dl_istream.read( dl_ibuffer, DL_UF3_PLY_CHUNK * LC_UF3_RECLEN );

            /* parsing input stream chunk */
            for ( long long int dl_iparse( 0 ), dl_oparse( 0 ); dl_iparse < dl_istream.gcount(); dl_iparse += LC_UF3_RECLEN, dl_oparse += DL_UF3_PLY_RECLEN ) {

                /* create i/o buffer mapping */
                dl_ipose = ( lc_uf3p_t * ) ( dl_ibuffer + dl_iparse );
                dl_opose = ( dl_plyp_t * ) ( dl_obuffer + dl_oparse );
                dl_idata = ( lc_uf3d_t * ) ( dl_ipose + 3 );
                dl_odata = ( dl_plyd_t * ) ( dl_opose + 3 );

                /* input stream vertex filtering */
                if ( dl_ipose[0] != dl_ipose[0] ) continue;
                if ( dl_ipose[1] != dl_ipose[1] ) continue;
                if ( dl_ipose[2] != dl_ipose[2] ) continue;

                /* compose output stream chunk */
                dl_opose[0] = dl_ipose[0];
                dl_opose[1] = dl_ipose[1];
                dl_opose[2] = dl_ipose[2];
                dl_odata[0] = dl_idata[0];
                dl_odata[1] = dl_idata[1];
                dl_odata[2] = dl_idata[2];

            }

            /* write output stream chunk */
            dl_ostream.write( dl_obuffer, ( dl_istream.gcount() / LC_UF3_RECLEN ) * DL_UF3_PLY_RECLEN );

        /* conversion end condition */
        } while ( dl_istream.gcount() > 0 );

        /* delete output stream */
        dl_ostream.close();

        /* delete input stream */
        dl_istream.close();

        /* release memory allocation */
        delete [] dl_obuffer;

        /* release memory allocation */
        delete [] dl_ibuffer;

        /* send message */
        return( EXIT_SUCCESS );

    }

