
# FeatureExtraction_LaneBoundary_ExtractCL

<!--
The following template is based on:
Best-README-Template
Search for this, and you will find!
>
<!-- PROJECT LOGO -->
<br />
<p align="center">
  <!-- <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a> -->

  <h2 align="center"> FeatureExtraction_LaneBoundary_ExtractCL
  </h2>

  <pre align="center">
    <img src=".\Images\RoadCenterLine.jpg" alt="main centerline picture" width="960" height="540">
    <!--figcaption>Fig.1 - The typical progression of map generation.</figcaption -->
    <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

  <p align="center">
    This repository contains a set of MATLAB functions for extracting lane-marker-based road centerlines using LiDAR data. The extraction process leverages intensity filtering, extrema detection, and pattern matching strategies to identify and clean lane marker features. The output is a refined centerline representation for each traversal or "lap".
    <br />
    <!-- a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/tree/main/Documents">View Demo</a>
    <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/issues">Report Bug</a>
    <a href="https://github.com/ivsg-psu/FeatureExtraction_Association_PointToPointAssociation/issues">Request Feature</a -->
  </p>
</p>

***

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About the Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="structure">Repo Structure</a>
      <ul>
        <li><a href="#directories">Top-Level Directories</li>
        <li><a href="#dependencies">Dependencies</li>
      </ul>
    </li>
    <li><a href="#functions">Functions</li>
      <ul>
        <li><a href="#basic-support-functions">Basic Support Functions</li>
        <ul>
          <li><a href="#fcn_ExtractCL_plotCenterLineXY">fcn_ExtractCL_plotCenterLineXY - Plotting utility for center line outputs in ENU</li>
          <li><a href="#fcn_ExtractCL_plotCenterLineLL">fcn_ExtractCL_plotCenterLineLL - Plotting utility for center line outputs in LLA</li>
        </ul>
        <li><a href="#core-functions">Core Functions</li>
        <ul>
          <li><a href="#fcn_ExtractCL_extractCL_WhiteStrip">fcn_ExtractCL_extractCL_WhiteStrip - Core function of the repo, extract center line from solid white lane marker</li>
          <li><a href="#fcn_ExtractCL_matchCLByPattern_WhiteStrip">fcn_ExtractCL_matchCLByPattern_WhiteStrip - Detects matched lane patterns using filtering and template matching.</li>
          <li><a href="#fcn_ExtractCL_interpolateSTBin">fcn_ExtractCL_interpolateSTBin - Interpolates LiDAR data into regular ST grid for processing.</li>
          <li><a href="#fcn_ExtractCL_matchPattern">fcn_ExtractCL_matchPattern - Implements 1D pattern matching across intensity profiles, find pattern start index.</li>
          <li><a href="#fcn_ExtractCL_projectPC_ENUToST">fcn_ExtractCL_projectPC_ENUToST - Projects ENU coordinates of point clouds to the ST coordinates using reference poses.</li>
          <li><a href="#fcn_ExtractCL_projectPC_STToENU">fcn_ExtractCL_projectPC_STToENU - Projects ST coordinates of points back to the global ENU frame using reference poses.</li>
          <li><a href="#fcn_ExtractCL_cleanCLPoints">fcn_ExtractCL_cleanCLPoints - Sorts and filters the center line points based on lateral trends and outlier removal.</li>
        </ul>
      </ul>
    <li><a href="#usage">Usage</a></li>
     <ul>
     <li><a href="#general-usage">General Usage</li>
     <li><a href="#examples">Examples</li>
     <li><a href="#definition-of-endpoints">Definition of Endpoints</li>
     </ul>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

***

<!-- ABOUT THE PROJECT -->
## About The Project

<!--[![Product Name Screen Shot][product-screenshot]](https://example.com)-->

This repository provides MATLAB functions for extracting lane-marker-based road center lines from 3D LiDAR point clouds in the ENU coordinate system.

The current implementation focuses specifically on detecting **solid white lane markers**, using LiDAR intensity data combined with extrema filtering and pattern matching techniques. These features are used to identify and clean the lane marker, and then project the result into a continuous center line.

This tool is suitable for use in structured road environments such as test tracks or highways, where lane markings are well-defined. Future versions may support additional lane types or custom user-defined features.

* Inputs:
  * a "pointcloud array" type, as explained in the Path library, or a path of XY points in N x 2 format
  * s_width: Width of each s-bin (e.g., 5)
  * s_res: Resolution in s-direction.
  * t_res: Resolution in t-direction.
  * min_pts: Minimum number of points to process a strip.
  * Ref_Pose: Reference trajectory with [X, Y, Z, Roll, Pitch, Yaw, Station].
* Outputs
  * Array of center line points with [X, Y, Z, S, T, E], where E is the MSE of the patter matching
  * Struct containing the corresponding pattern template and extrema filter

<a href="#featureextraction_dataclean_breakdataintolaps">Back to top</a>

***

<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Installation

1. Make sure to run MATLAB 2023b or higher.

2. Clone the repo

   ```sh
   git clone https://github.com/ivsg-psu/FeatureExtraction_LaneBoundary_ExtractCL
   ```

3. Run the main code in the root of the folder (script_demo_ExtractCL.m), this will download the required utilities for this code, unzip the zip files into a Utilities folder (.\Utilities), and update the MATLAB path to include the Utility locations. This install process will only occur the first time. Note: to force the install to occur again, delete the Utilities directory and clear all global variables in MATLAB (type: "clear global *").
4. Confirm it works! Run script_demo_Laps. If the code works, the script should run without errors. This script produces numerous example images such as those in this README file.

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

<!-- STRUCTURE OF THE REPO -->
### Directories

The following are the top level directories within the repository:
<ul>
 <li>/Documents folder: Descriptions of the functionality and usage of the various MATLAB functions and scripts in the repository.</li>
 <li>/Functions folder: The majority of the code for the point and patch association functionalities are implemented in this directory. All functions as well as test scripts are provided.</li>
 <li>/Utilities folder: Dependencies that are utilized but not implemented in this repository are placed in the Utilities directory. These can be single files but are most often folders containing other cloned repositories.</li>
</ul>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

### Dependencies

* [Errata_Tutorials_DebugTools](https://github.com/ivsg-psu/Errata_Tutorials_DebugTools) - The DebugTools repo is used for the initial automated folder setup, and for input checking and general debugging calls within subfunctions. The repo can be found at: <https://github.com/ivsg-psu/Errata_Tutorials_DebugTools>

* [PathPlanning_PathTools_PathClassLibrary](https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary) - the PathClassLibrary contains tools used to find intersections of the data with particular line segments, which is used to find start/end/excursion locations in the functions. The repo can be found at: <https://github.com/ivsg-psu/PathPlanning_PathTools_PathClassLibrary>

* [FieldDataCollection_VisualizingFieldData_PlotRoad](https://github.com/ivsg-psu/FieldDataCollection_VisualizingFieldData_PlotRoad?tab=readme-ov-file#fielddatacollection_visualizingfielddata_plotroad) - the PlotRoad repo is used for plotting data in either ENU or LLA coordinates.

    Each should be installed in a folder called "Utilities" under the root folder, namely ./Utilities/DebugTools/ , ./Utilities/PathClassLibrary/ . If you wish to put these codes in different directories, the main call stack in script_demo_Laps can be easily modified with strings specifying the different location, but the user will have to make these edits directly.

    For ease of getting started, the zip files of the directories used - without the .git repo information, to keep them small - are included in this repo.

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

<!-- FUNCTION DEFINITIONS -->
## Functions

### Basic Support Functions

#### fcn_ExtractCL_plotCenterLineXY

The function fcn_ExtractCL_plotCenterLineXY plots the center line in ENU coordinates. For example, the function was used to make the plot below with sample data.
<pre align="center">
  <img src=".\Images\fcn_ExtractCL_plotCenterLineXY.png" alt="fcn_ExtractCL_plotCenterLineXY picture" width="400" height="300">
  <figcaption>Fig.1 - The function fcn_ExtractCL_plotCenterLineXY plots the center line outputs.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_plotCenterLineLL

The function fcn_ExtractCL_plotCenterLineLL plots the center line in LLA coordinates. For example, the function was used to make the plot below with sample data.

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_plotCenterLineLL.png" alt="fcn_ExtractCL_plotCenterLineLL picture" width="400" height="300">
  <figcaption>Fig.2 - The function fcn_ExtractCL_plotCenterLineXY plots the center line outputs.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_plotCenterLineLL_ZoomIn.png" alt="fcn_ExtractCL_plotCenterLineLL picture" width="400" height="300">
  <figcaption>Fig.3 - Zoom-in view of the center line geoplot.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

### Core Functions

#### fcn_ExtractCL_extractCL_WhiteStrip

The function fcn_ExtractCL_extractCL_WhiteStrip is the core function for this repo that extracts center line from solid white lane marker.The algorithm is designed to work with 3D point clouds projected onto the (S, T) frame, where S is the longitudinal coordinate aligned with the vehicle's path and T is the lateral coordinate perpendicular to it.

##### Key Features

- Detects **solid white lane markers** using:
  - Intensity-based **extrema filtering**
  - **Pattern matching** with a predefined lane template
- Works with **raw LiDAR point clouds** projected to the `(S, T)` frame using vehicle pose
- Returns  **center line trajectory** in ENU coordinates
- Includes outlier removal and optional plotting/debug tools

##### Inputs

- `pointcloud_array`: Nx4 or Nx6 LiDAR array with [X, Y, Z, Intensity, ...]
- `s_width`: Strip width in longitudinal `S` direction
- `s_res`, `t_res`: Resolution of the ST grid
- `min_pts`: Minimum number of points in a strip for processing
- `Ref_Pose`: Mx6 or Mx7 vehicle pose array with [X, Y, Z, ..., Yaw, Station]
- Optional debug flag / figure handle

##### Outputs

- `XYZSTE_Center_Line_Array`: Extracted center line points in ENU [X Y Z S T E], where E is the mean squared error of pattern matching
- `HistoryData`: Struct containing reference data (e.g., extrema locations, matched patterns, extrema filter) for map change detection and update

##### Use Case

This function is best suited for environments where **solid white strip lane markers** are consistently visible—such as test tracks or structured roads. Support for dashed lines or multiple marker types may be added in future versions.

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_plotCenterLineXY_ZoomIn.png" alt="fcn_ExtractCL_extractCL_WhiteStrip picture" width="400" height="300">
  <figcaption>Fig.4 - The function fcn_ExtractCL_extractCL_WhiteStrip is the core function in the repo, and extract center line points from solid white lane marker.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_dataclean_breakdataintolaps">Back to top</a>

#### fcn_ExtractCL_interpolateSTBin

The `fcn_ExtractCL_interpolateSTBin` function converts raw `(s, t, intensity, z)` point cloud values into a gridded ST format using interpolation. This step is essential before lane pattern matching.

##### Key Features

- Interpolates LiDAR data into uniform ST grid
- Supports intensity and Z channels
- Automatically adjusts S-bin range when data is sparse

##### Inputs

- `s_bin`, `t_bin`: Coordinates in bin
- `intensity_bin`, `z_bin`: Data values
- `s_low`: Start of bin
- `s_res`, `s_width`, `t_res`: Grid resolutions

##### Outputs

- `S_interp`, `T_interp`: Grid coordinate mesh
- `I_interp`, `Z_interp`: Interpolated values


<pre align="center">
  <img src=".\Images\fcn_ExtractCL_interpolateSTBin.png" alt="fcn_ExtractCL_interpolateSTBin picture" width="400" height="300">
  <figcaption>Fig.5 - The function fcn_ExtractCL_interpolateSTBin is essential for downstream tasks.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>


***
#### fcn_ExtractCL_matchCLByPattern_WhiteStrip

The `fcn_ExtractCL_matchCLByPattern_WhiteStrip` function supports `fcn_ExtractCL_extractCL_WhiteStrip` by performing **lane centerline detection** in the `(S, T)` intensity grid using a combination of **extrema filtering** and **pattern matching**. It identifies the best-matching pattern in each S-strip and outputs a centerline mask, associated templates, and matching errors. The function also supports optional integration of previously extracted lane patterns (history) for improved robustness.

##### Key Features

- Applies **optimized extrema filters** per strip
- Performs **pattern matching** with both full and single-strip templates
- Uses **SSE minimization** to select best-fit index
- Optionally integrates **historical patterns** for continuity
- Outputs **centerline mask**, pattern templates, filters, and fit errors

##### Inputs

- `intensity_data`: `NxM` intensity grid in ST frame
- `pattern_template`: Reference full pattern
- `T_resolution`: Grid spacing in T
- `single_strip_template`: Template for isolated stripe
- *(optional)* `fig_num`: Enable debug plot
- *(optional)* `HistoryData`: Past pattern/line info
- *(optional)* `pointcloud_in_bin`: Raw bin data
- *(optional)* `s_strip_edges`, `t_strip_edges`: Bin edges

##### Outputs

- `center_line_mask`: Logical matrix of matched points
- `pattern_template_cell`: Pattern per strip
- `extrema_filter_cell`: Filter applied per strip
- `best_pattern_template`: Best selected template
- `best_fit_errors`: Fit errors per strip


<pre align="center">
  <img src=".\Images\fcn_ExtractCL_matchCLByPattern_WhiteStrip.png" alt="fcn_ExtractCL_matchCLByPattern_WhiteStrip picture" width="400" height="300">
  <figcaption>Fig.6 - The function fcn_ExtractCL_matchCLByPattern_WhiteStrip find the best matched pattern for the current strip.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***
#### fcn_LaneDetection_matchPattern

The `fcn_LaneDetection_matchPattern` function performs **1D sliding-window pattern matching** between a known pattern and a LiDAR intensity profile. It outputs the matched pattern, the best alignment index, and the matching error.

##### Key Features

- Performs **template-based pattern matching**
- Uses **sum of squared error (SSE)** to find best match
- Returns aligned binary mask for matched region

##### Inputs

- `intensity_data`: 1D intensity array
- `pattern_template`: 1D template (typically binary)
- *(optional)* `shift_range`: Not yet implemented

##### Outputs

- `best_fit_pattern`: Matched binary mask
- `best_fit_idx`: Start index of match
- `best_fit_error`: Mean squared error of best match

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***


#### fcn_ExtractCL_projectPC_ENUToST

The `fcn_ExtractCL_projectPC_ENUToST` function transforms point cloud data from the global **ENU coordinate frame** into a local **(S, T)** frame aligned with a reference vehicle trajectory. This transformation enables structured lane feature extraction along the path of travel.

##### Key Features

- Projects 3D points into a curvilinear frame aligned with vehicle motion
- Computes `S` (longitudinal station) and `T` (lateral offset) relative to the trajectory
- Enables subsequent intensity mapping, filtering, and pattern matching in the (S, T) space

##### Inputs

- `pointcloud_array`: Nx3 or Nx6 matrix containing `[X, Y, Z, ...]` in ENU coordinates
- `Ref_Pose`: Mx7 matrix of reference vehicle poses `[X, Y, Z, Roll, Pitch, Yaw, Station]`

##### Outputs

- `pointcloud_with_ST`: The input pointcloud, appended with two extra columns: `S` and `T`



<pre align="center">
  <img src=".\Images\fcn_ExtractCL_projectPC_ENUToST.png" alt="fcn_ExtractCL_projectPC_ENUToST picture" width="600" height="300">
  <figcaption>Fig.7 - This function is typically used before remapping or gridding point cloud data into a structured ST intensity image.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***
#### fcn_ExtractCL_projectPC_STToENU

The `fcn_ExtractCL_projectPC_STToENU` function maps local `(S, T)` coordinates of matched points back to global ENU `(X, Y)` using reference pose and heading.

##### Inputs

- `S_extrema`: Longitudinal values
- `T_extrema`: Lateral values
- `Ref_Pose`: Vehicle poses

##### Output

- `XY_extrema`: Global ENU coordinates

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***
#### fcn_ExtractCL_cleanCLPoints

The `fcn_ExtractCL_cleanCLPoints` function sorts center line by stations `S` and filters outliers in the extracted center line by comparing lateral `T` values to local medians.

##### Inputs

- `XYZSTE_Center_Line_Array`: Extracted center line
- `T_threshold`: Lateral tolerance (e.g., 0.3 m)

##### Output

- `XYZSTE_Center_Line_Array_Cleaned`: Filtered and sorted result

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

<!-- USAGE EXAMPLES -->
## Usage
<!-- Use this space to show useful examples of how a project can be used.
Additional screenshots, code examples and demos work well in this space. You may
also link to more resources. -->

### General Usage

Each of the functions has an associated test script, using the convention

```sh
script_test_fcn_fcnname
```

where fcnname is the function name as listed above.

As well, each of the functions includes a well-documented header that explains inputs and outputs. These are supported by MATLAB's help style so that one can type:

```sh
help fcn_fcnname
```

for any function to view function details.

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

### Examples

1. Run the main script to set up the workspace and demonstrate main outputs, including the figures included here:

   ```sh
   script_demo_ExtractCL
   ```

    This exercises the main function of this code: fcn_ExtractCL_extractCL_WhiteStrip

2. After running the main script to define the included directories for utility functions, one can then navigate to the Functions directory and run any of the functions or scripts there as well. All functions for this library are found in the Functions sub-folder, and each has an associated test script. Run any of the various test scripts, such as:

   ```sh
   script_test_fcn_ExtractCL_createLanePattern
   ```

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

## Major release versions

This code is still in development (alpha testing)

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

<!-- CONTACT -->
## Contact

Sean Brennan - sbrennan@psu.edu

Project Link: [hhttps://github.com/ivsg-psu/FeatureExtraction_LaneBoundary_ExtractCL](https://github.com/ivsg-psu/FeatureExtraction_LaneBoundary_ExtractCL)

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
