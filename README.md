# AI_Course_Image_Reconstruction
## Artificial Intelligence Course Project

## Required Libraries:

#### For installing required libraries, please run pip install requirements.txt in Your terminal


## Usage:

### The main logic of merging the fragments is implemented in merger.py module

### To run the algorithm on Your data please run main.py
As shown in the code, please pass the path to Your data to the reader, and path where You want the resulted object to be stored to the writer


### To Evaluate the algorithm, please run metrics.py
As shown in the code, please pass the path to the object, created by the algorithm as first argument of calculate_transformation function and path to the original object as the second

### File for generating syntetics --- synthetic_raytrace.py
Please, pass the path as a first argument of RayTrace class (as shown in example). By default, the fragments will be written to input_data_folder (can be changed in create_and_write_pcd_file method)

## Link to the Stanford 3D Scanning Repository:
- [Repository](http://graphics.stanford.edu/data/3Dscanrep/)

