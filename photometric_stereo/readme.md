

![Photometric_stereo](https://user-images.githubusercontent.com/74460048/179398519-adc3e854-3e5d-41be-ac72-828a2f36ba87.png)

Photometric stereo is a technique used to infer the 3D shape of an object based on its 2D image. This reconstruction is done by analyzing the albedo of object. Albedo is the measure of diffuse, or the ratio of radiosity of an object. It is valuated between 0 meaning dark shadow or total absorbance of light known as umbra, and 1 meaning total reflectance of illumination. between these two levels of brightness is the penumbra which is partial shadow.  

To perform photometric stereo, a set of images of a stationary object taken by the same camera from the same frame is needed. The difference between these images must be the illumination of the object. Analyzing the albedo of the pixels of these images leads to the normal vectors at these pixels. From the normal vectors, a depth map is created to give the estimated shape of the patch using orthographic projection.
