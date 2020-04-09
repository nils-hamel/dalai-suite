## Overview

This tool was developed to be able to filter (to clean) arbitrary large 3D point-based models. In the field of _structure from motion_, it is nowadays possible to compute very large models that can weight up to tens of gigabytes and more. Such computed dense photogrammetric models comes with numerous outliers that passed the different filtering process built-in the photogrammetric solutions. It is then most of the time required to perform additional filtering in order to suppress the remaining outliers of the models.

This solution is then made to allow filtering of models that weight much more than the volatile memory of modern computers can store. This tool uses an hash-based methodology to break the models into pieces that are stored on non-volatile memory and filter one by one before to be put together again.

The implemented filtering process is based on a statistical analysis of each point of the model surroundings. A point passes the filtering process if it is determined that it is sufficiently close to enough other points of the models.

This tools expects models in _uv3_ format that contains only points. If models containing lines and triangle are provided, unexpected results can occur (inconsistency in primitives definition).

## Usage

Considering point-based model contain in an _uv3_ file, the following command :

    ./dalai-filter -i /path/to/file.uv3 -o /path/to/filtered.uv3 -c 64 -y /tmp/tempdir -f 2.0 -t 2

allows to filter its content as illustrated by the following images :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-filter/doc/filter-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-filter/doc/filter-2.jpg?raw=true" width="384">
<br />
<i>Original model (left) and its filtered (cleaned) counterpart (right)</i>
</p>
<br />

One can see that numerous points are removed from the original model leading to a cleaner version of it.

The _--temporary/-y_ parameter is mandatory and has to point to a valid non-existing directory (it is created and then removed by the process) that can offer enough of free space to store a full copy of the original model to filter.

The filtering process operates in three phases : the first phase is to compute the _minimum distance mean value_ of the original model. To do so, a sample of points is chosen in the model and the distance to their closest neighbour is computed. A mean value of these minimum distances is computed and called _minimum distance mean value_. The size of the sample can be specified through the _--count/-c_ parameter. The default value is _64_. Increasing this value allows to get a more accurate estimation of the _mean value_ but also increase the computational cost (non-linear increase of processing time).

The second phase of the filtering process is the model hashing : in order to deal with arbitrary large models, they are cut into smaller pieces in order to apply the filtering process on each piece separately, keeping the used volatile memory low. The hashing size of the pieces is determined by a factor of the _minimum distance mean value_ of _75_. The temporary directory is used to store these pieces, explaining why it need a storage capacity of the size of the original model.

The last phase is the filtering itself, applied on each hashed pieces of the model, and comes with two variations : for both variations, the filtering condition is the same and driven by the parameters :

* Minimum distance mean value factor : _--factor/-f_
* Neighbours count value threshold : _--threshold/-t_

The filtering condition is applied as follows : for each point of the model piece, the amount of neighbour points withing a range of :

    range = (minimum distance mean value) * (--factor/-f parameter value)

to the considered point position is determined. If this count is above the threshold parameter (_--threshold/-t_), the point passes the filter and is kept. It is otherwise discarded. As all the point of the pieces are filtered, the resulting point are exported in the output file.

The two available variations of the filtering process are based on the considered _minimum distance mean value_ : for the _homogeneous_ filtering, the overall model value is considered for the process of each hashed piece. If the _adaptative_ variation is considered, the _minimum distance mean value_ is recomputed for each piece separately. This allows the filtering process to remain adapted to the local density of the model. In other words, the _adaptative_ method takes into account the specific density of each hashed pieces during application of the filtering condition. This can be useful when cleaning models showing disparity in their local density of points.

To activate the _adaptative_ filtering, the user just need to add the _--adaptive/-a_ argument to the command without parameter.

In both cases, the modulation of the filtering process is then achieved by modulating both values of _--factor/-f_ and _--threshold/-t_ values until the desired results are met.
