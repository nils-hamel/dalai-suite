## Overview

This tool allows to convert [_INTERLIS_](https://www.interlis.ch/dokumentation/interlis-2) files into _uv3_ files. It allows to modulate the color of the extracted geometries and their source in the database.

## Usage

Considering an _INTERLIS_ file (_ITF_) containing Swiss land register geometries, the following command allows to extract a specific part of the database :

    ./dalai-interlis -i /path/to/file.itf -o /path/to/export.uv3
                     -r 255 -g 192 -b 0 
                     -t Couverture_du_sol
                     -a SurfaceCS_Geometrie

The _-r_, _-g_ and _-b_ parameters allows to specify the color of the extracted data. The desired data are pointed through the _-t (--topic)_ and the _-a (--table)_ that have to correspond to valid and available _INTERLIS_ topic and table in the topic. In addition, the table has to contain proper geometries (_STPT_, _LIPT_, _ELIN_). If these constraints are not fulfilled, the created _uv3_ file is left empty.

The following image on the left gives an illustration of the obtained _uv3_ model using the previous command on the Swiss land register data :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-interlis-uv3/doc/example-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-interlis-uv3/doc/example-2.jpg?raw=true" width="384">
<br />
<i>Example of INTERLIS data converted in uv3 format on a Swiss land register model - Data : SITN</i>
</p>
<br />

The right image on the previous illustration shows the result of the following command made on the same data, pointing at a different topic and table :

    ./dalai-interlis -i /path/to/file.itf -o /path/to/export.uv3
                     -r 255 -g 255 -b 0 
                     -t Objets_divers 
                     -a Element_surfacique_Geometrie

The content of the _INTERLIS_ data have then to be known in order to be able to extract the desired geometries.