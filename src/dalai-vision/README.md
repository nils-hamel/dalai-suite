## Overview

This tool is dedicate to _uv3_ files visualization through a minimalist 3D interface. The interface allows to display the model contained in the provided _uv3_ and to modify the point of view using the keyboard and mouse.

In addition, the tool also provides a specific graphical-based methodology that allows to extract optimal intersection of model surfaces. It allows the user to extrapolate optimal surfaces from the model structures (like walls and ground) and to determine their optimal intersection point. This methodology is often used during photogrammetric models geographical registration.

## Usage

The tool is started using the following command :

    ./dalai-vision -i /path/to/file.uv3

Before to show the model on the interface, the tool computes an estimation of the model _minimum distance mean value_ which is the mean value of the model vertex closest neighbour. On large models, this operation can take some time and delay the display of the model. The model is also centered on its computed centroid.

As the interface is started, the following displays can be obtained :

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/vision-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/vision-2.jpg?raw=true" width="384">
<br />
<i>Vision interface showing point-based models of the City of Geneva - Data : SITG and DHLAB</i>
<br />
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/vision-3.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/vision-4.jpg?raw=true" width="384">
<br />
<i>Vision interface showing models of the City of Geneva in 1850 and 2018 - Data : Ville de Genève and SITG</i>
</p>
<br />

A minimal frame is also displayed with the model. The frame is in red for _x_ direction, green for _y_ and blue for _z_ ones. The axes of the frame are interrupted to show the estimated _minimum distance mean value_ size and one hundred time its value.

The model can be manipulated through the mouse. To rotate the model, maintain the left-click and simply move the mouse. A _tarball_ approach is implemented for the rotation of the model. The mouse wheel is used to modify the distance to the model. If the _CONTROL_ or the _SHIFT_ key is maintained during model distance modification, it is speed-up or slowed, respectively. The model center of rotation can be set on any part of the model by using the left-double-click on the desired element. The keyboard actions are summarized as follows :

* Key 1, 2, 3 and 4 : modify the model points rendering size
* Key P : display model with points only
* Key O : display model with point and lines only
* Key I : display model with points, lines and polygons

The _ESCAPE_ key is used to end the visualization.

## Surfaces Extrapolation and Optimal Intersection

In the context of model geographical registration, a sequence of homologous points have to be defined between the model and the registration reference. As the selection of such homologous points can be difficult, the interface propose a method to simplify it. The assumption is that references only give access to specific points that are most of the time road or building corners. As such elements can be very complicated to determine in the model to register, this tool give access to surfaces extrapolation and optimal intersection computation. This methodology is mainly used for registration of point-based models, but can also be used on polygonal models.

As the model is loaded and visible in the interface, the user can choose a surface of the model, for example a wall, and start to select a few points that belong to the surface. As at least four points are selected, the interface starts displaying the extrapolated surface (by default, the red surface is used). To add a point to the surface, the user has to choose the desired point and set it as the center of rotation (double-left-click). As the desired point is the center of rotation, the wheel-click is used to add it to the surface. Applying this procedure on a point of the surface removes it. At least four points have to be set, but as many as desired can be set.

<br />
<p align="center">
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/register-1.jpg?raw=true" width="384">
&nbsp;
<img src="https://github.com/nils-hamel/dalai-suite/blob/master/src/dalai-vision/doc/register-2.jpg?raw=true" width="384">
<br />
<i>Single surface extrapolation (left) and optimal intersection of three surfaces (right) - Data : SITG</i>
</p>
<br />

As the surface display itself can make further points selection tricky, it can be hidden using the _TABULATION_ key to switch the surfaces display mode.

As the surface is defined, the keys _A_, _S_ and _D_ can be used to automatically add points to the surface. To do so, points sufficiently close to the surface are automatically added (the _minimum distance mean value_ is used in the proximity criterion). The three keys trigger the same operation but _A_ tends to decrease the amount of points, _S_ maintains it and _D_ tends to increase it. These keys presses can be repeated to collect all the desired points. If the surface is badly defined, this process can lead to undesired results, forcing the user to restart the operation.

The key _BACKSPACE_ can be used to clear all the point of a surface, making it disappears. All the presented operations apply on the active surface (the red one is the default one). You can set the red, green and blue surfaces as the active one by using the _Q_, _W_ and _E_ keys, respectively.

As three surfaces are defined and extrapolated with sufficient precision for the user, the optimal intersection computation can be triggered by hitting the _ENTER_ key. The result of the intersection, expressed in the model coordinates frame, is dump in the standard output (in which the tool is started).