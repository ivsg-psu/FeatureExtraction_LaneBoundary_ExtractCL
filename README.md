
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
    <li><a href="#structure">Repo Structure</a>
      <ul>
        <li><a href="#directories">Top-Level Directories</a></li>
        <li><a href="#dependencies">Dependencies</a></li>
      </ul>
    </li>
    <li><a href="#functions">Functions</a>
      <ul>
        <li><a href="#core-functions">Core Functions</a>
          <ul>
            <li><a href="#fcn_ExtractCL_extractCenterLine">fcn_ExtractCL_extractCenterLine</a> – Core lane centerline extraction pipeline</li>
            <li><a href="#fcn_ExtractCL_projectPC_ENUToST">fcn_ExtractCL_projectPC_ENUToST</a> – Project LiDAR points to ST coordinates</li>
            <li><a href="#fcn_ExtractCL_convertRefTrajToST">fcn_ExtractCL_convertRefTrajToST</a> – Convert reference trajectory to ST frame</li>
            <li><a href="#fcn_ExtractCL_filterPCinT">fcn_ExtractCL_filterPCinT</a> – Lateral filtering of ST point clouds</li>
            <li><a href="#fcn_ExtractCL_organizePointCloudST">fcn_ExtractCL_organizePointCloudST</a> – Organize point clouds into S–T grids</li>
            <li><a href="#fcn_ExtractCL_extractLaneMarkers">fcn_ExtractCL_extractLaneMarkers</a> – Extract lane markers via extrema filtering and pattern matching</li>
            <li><a href="#fcn_ExtractCL_createLanePattern">fcn_ExtractCL_createLanePattern</a> – Generate binary lane-marker pattern templates</li>
            <li><a href="#fcn_ExtractCL_findLaneMarkersByPattern">fcn_ExtractCL_findLaneMarkersByPattern</a> – 1D pattern matching across intensity strips</li>
            <li><a href="#fcn_ExtractCL_matchPattern">fcn_ExtractCL_matchPattern</a> – Sliding-window template matching</li>
            <li><a href="#fcn_ExtractCL_separateLaneMarkers">fcn_ExtractCL_separateLaneMarkers</a> – Separate left/right lane markers</li>
            <li><a href="#fcn_ExtractCL_computeCLwithLaneMarkers">fcn_ExtractCL_computeCLwithLaneMarkers</a> – Compute road centerline from lane markers</li>
            <li><a href="#fcn_ExtractCL_projectPC_STToENU">fcn_ExtractCL_projectPC_STToENU</a> – Back-project ST results into ENU frame</li>
          </ul>
        </li>
        <li><a href="#basic-support-functions">Basic Support Functions</a>
          <ul>
            <li><a href="#fcn_ExtractCL_loadMatData">fcn_ExtractCL_loadMatData</a> – Load pointcloud data and reference trajectory</li>
            <li><a href="#fcn_ExtractCL_plotCenterLineXY">fcn_ExtractCL_plotCenterLineXY</a> – Plot centerline in ENU</li>
            <li><a href="#fcn_ExtractCL_plotCenterLineLL">fcn_ExtractCL_plotCenterLineLL</a> – Plot centerline in LLA</li>
            <li><a href="#fcn_ExtractCL_comparePCinENUandST">fcn_ExtractCL_comparePCinENUandST</a> – Compare ENU vs ST coordinates</li>

          </ul>
        </li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>


***

<!-- ABOUT THE PROJECT -->
## About The Project

<!--[![Product Name Screen Shot][product-screenshot]](https://example.com)-->

This repository provides MATLAB functions for extracting lane-marker-based road center lines from 3D LiDAR point clouds in the ENU coordinate system.

The current implementation focuses specifically on detecting **lane markers**, using LiDAR intensity data combined with extrema filtering and pattern matching techniques. These features are used to identify and separate the lane markers, and then compute the center line based on the lane markers coordinates.

This tool is suitable for use in structured road environments such as test tracks or highways, where lane markings are well-defined. Future versions may support additional lane types or custom user-defined features.

* Inputs:
  * a "pointcloud array" type, as explained in the Path library, or a path of XY points in N x 2 format
  * s_width: Width of each s-bin (e.g., 5)
  * s_res: Resolution in s-direction.
  * t_res: Resolution in t-direction.
  * min_pts: Minimum number of points to process a strip.
  * Ref_Pose: Reference trajectory with [X, Y, Z, Roll, Pitch, Yaw, Station].
* Outputs
  * Array of center line points with [X, Y, Z, S, T]
  * Struct containing the corresponding pattern template and extrema filter

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

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



### Core Functions

#### fcn_ExtractCL_extractCenterLine

The function `fcn_ExtractCL_extractCenterLine` is the core function for this repo that extracts center line from lane markers. It performs the full lane center line extraction pipeline from LiDAR point cloud data in ENU coordinates.
It integrates ENU→ST projection, lateral filtering, extrema-based lane marker extraction, and lane-pair separation to generate the final road center line.

##### Key Features

- Implements the complete end-to-end lane extraction pipeline in one function.
- Converts ENU point clouds to the station–lateral (S,T) frame using vehicle poses.
- Applies lateral filtering to limit processing to relevant roadway regions.
- Detects **lane markers** using:
  - Intensity-based **extrema filtering**
  - **Pattern matching** with a predefined lane template
- Separates left and right lane markers, and computes the center line from the selected side.
- Returns  **road center line** in ENU and ST coordinates
- Modular design — calls other subfunctions

##### Inputs

- `PointCloud_ENU_Example`: LiDAR point cloud in ENU frame [X, Y, Z, Intensity, ...].

- `VehiclePose_ENU_Example`: Vehicle poses [X, Y, Z, Roll, Pitch, Yaw].

- `T_range`: Lateral filtering range [Tmin Tmax].

- (Optional) `s_length`, `N_s`, `t_res`, `min_pts`, `mode`, `fig_num` — control longitudinal binning, ST resolution, point thresholds, and plotting options.

##### Outputs

- `RoadCenterLine`: Extracted road center line points in ENU [X Y Z S T].
- `LaneMarkerCenterLine`: Extracted road center line points in ENU [X Y Z S T].

##### Use Case

This is the core extraction function of the repository, suitable for structured environments such as highways, test tracks, or road sections with clearly visible **lane markers**. It is typically called within the top-level scripts (e.g.,`script_demo_ExtractCL.m`) to generate road center lines for HD-map construction and visualization. Support for dashed lines or multiple marker types may be added in future versions.

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_extractCenterLine.png" alt="fcn_ExtractCL_extractCenterLine picture"  width="600">
  <figcaption>Fig.1 - The function fcn_ExtractCL_extractCenterLine is the core function in the repo, and extract center line points lane markers.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_extractCenterLine_geoplot.png" alt="fcn_ExtractCL_extractCenterLine picture" width="600">
  <figcaption>Fig.2 - The function fcn_ExtractCL_extractCenterLine is the core function in the repo, and extract center line points lane markers.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>


<a href="#featureextraction_laneboundary_extractcl">Back to top</a>


#### fcn_ExtractCL_projectPC_ENUToST  

The function `fcn_ExtractCL_projectPC_ENUToST` projects LiDAR point clouds from the ENU frame into the **curvilinear (S, T)** coordinate system defined by a reference trajectory. This function internally calls `fcn_ExtractCL_buildSegmentsFromRefPose` to generate the segment cache (Seg), which includes precomputed tangents, normals, and station indexing along the reference trajectory. Each LiDAR point is associated with its nearest trajectory segment to compute its **station (S)** and **lateral offset (T)** for downstream lane-marker extraction.  

##### Key Features  
- Converts raw ENU point clouds to **station–lateral (S,T)** coordinates.  
- Supports multi-frame LiDAR data through cell input format.  
- Uses a **KD-tree nearest-segment search** for fast projection.  
- Returns per-frame results by **appending two columns (S, T)** to the original point cloud input.  
- Generates segment cache (`Seg`) containing tangents, normals, and station indexing for reuse.  
- Optional debug/plot mode for visualization and validation.  

##### Inputs  
- `PointCloud_ENU_cell`: `{N×1}` cell array, each cell `[X Y Z Intensity (...)]`.  
- `Ref_Pose`: `[M×K]` numeric, reference trajectory `[X Y Z Roll Pitch Yaw]`.  
- *(Optional)* `fign_num`: figure handle for debug plotting (`-1` = no plot).  

##### Outputs  
- `pointCloud_ST_cell`: `{N×1}` cell array, each frame appended with `(S, T)`.  
- `ref_station`: cumulative arc-length along trajectory.  
- `Seg`: structure with segment geometry (tangents, normals, KD-tree, etc.).  

##### Use Case  
This function is typically the **first step** in the lane-centerline extraction pipeline. It converts the LiDAR point cloud into a trajectory-aligned coordinate frame for all subsequent filtering and lane-marker detection stages. Please run `script_test_fcn_ExtractCL_projectPC_ENUToST.m` for details.  

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_projectPC_ENUToST.png" alt="fcn_ExtractCL_projectPC_ENUToST picture">
  <figcaption>Fig.3 – LiDAR point clouds projected into the (S, T) coordinate frame relative to the reference trajectory.</figcaption>
</pre>


<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_convertRefTrajToST  

The function `fcn_ExtractCL_convertRefTrajToST` converts a given reference trajectory from ENU coordinates into its **station–lateral (S, T)** representation.  
It computes cumulative station, tangents, and normals between consecutive trajectory points, and builds a KD-tree for efficient nearest-segment queries during ENU→ST LiDAR projection.

##### Key Features  
- Converts a reference trajectory into its (S, T) geometric frame.  
- Computes **station**, **segment vectors**, **unit tangents**, and **left normals**.  
- Handles both predefined and automatically computed station values.  
- Builds a **KD-tree** of segment midpoints for nearest-segment lookup.  
- Used internally by `fcn_ExtractCL_projectPC_ENUToST` for LiDAR-to-ST projection.  

##### Inputs  
- `Ref_Pose`: `[M×K]` numeric array `[X Y Z (Roll Pitch Yaw Station)]`.  
  - If station values are not provided, cumulative arc length is computed automatically.  

##### Outputs  
- `Seg`: structure containing  
  - `ref_station` – cumulative arc length along the trajectory  
  - `seg_tangent`, `seg_normal` – per-segment unit direction vectors  
  - `traj_start`, `traj_end`, `seg_mid` – ENU coordinates of each segment  
  - `seg_tree` – KD-tree of segment midpoints for fast search  

##### Use Case  
This function is typically called inside `fcn_ExtractCL_projectPC_ENUToST` to establish the geometric basis of the reference path before projecting LiDAR point clouds into the (S, T) domain. Please see `script_test_fcn_ExtractCL_convertRefTrajToST.m` for details.  


<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_filterPCinT  

The function `fcn_ExtractCL_filterPCinT` filters LiDAR point clouds (already projected into the **(S, T)** coordinate system) by a given **lateral range**.  
It removes points whose lateral offset *T* falls outside the specified bounds, keeping only the region of interest for lane-marker extraction.

##### Key Features  
- Performs **lateral filtering** of point clouds in the (S, T) frame.  
- Operates on **per-frame cell arrays**, maintaining frame structure.  
- Automatically sorts input range `[T_min, T_max]` if reversed.  
- Efficiently removes invalid or out-of-bounds points.  
- Typical layout of each frame: `[X Y Z Intensity ... s t]`, where `t` is column 10.  

##### Inputs  
- `pointCloud_ST_cell`: `{N×1}` cell array containing per-frame LiDAR data `[X Y Z I ... S T]`.  
- `T_range`: `[1×2]` numeric vector `[T_min, T_max]` specifying inclusive filter bounds in meters.  

##### Outputs  
- `pointCloud_ST_filtered_cell`: `{N×1}` cell array with the same structure as input, but only points whose `t` values lie within the given range are retained.  

##### Use Case  
This function is used immediately after ENU→ST projection to restrict the processing region to the expected lane area (e.g., ±5 m from vehicle centerline).  
It improves efficiency for downstream lane-marker extraction and reduces noise from irrelevant roadway regions.

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_extractLaneMarkers  

The function `fcn_ExtractCL_extractLaneMarkers` is a **core stage** of the lane-marker extraction pipeline.  
It detects lane-marker ridges from LiDAR intensity data organized in the **(S, T)** coordinate system through **extrema filtering** and **adaptive pattern matching**.  
The resulting lane-marker points are projected back into ENU coordinates for further processing and visualization.

##### Key Features  
- Performs lane-marker detection from LiDAR intensity in the (S, T) frame.  
- Applies **extrema filtering** to identify high-intensity peaks across the lateral (T) axis.  
- Uses **adaptive template matching** (e.g., double-yellow, dashed, solid-white) for robust identification.  
- Supports **adaptive history** (`HistoryData`) for continuous template refinement across frames.  
- Projects detected marker points back to ENU using precomputed segment geometry (`Seg`).  
- Returns detected marker points `[X Y Z S T]` and diagnostic data for reproducibility.  

##### Inputs  
- `pointcloud_array`: `[N×10]` numeric array `[X Y Z Intensity ... S T]`.  
- `s_width`: longitudinal bin width (m).  
- `s_res`: longitudinal sampling step (m).  
- `t_res`: lateral sampling step (m).  
- `min_pts`: minimum number of points per bin to perform filtering.  
- `Seg`: segment geometry struct from `fcn_ExtractCL_convertRefPoseToST`.  
- *(Optional)* `fig_num`: figure number for per-bin visualization.  
- *(Optional)* `HistoryData`: struct for adaptive template updates across bins.  

##### Outputs  
- `XYZST_LaneMarkers_Array`: `[K×5]` or `[K×6]` numeric array `[X Y Z S T (error)]`, containing detected lane-marker points in ENU coordinates.  
- `HistoryData`: updated structure storing template history, extrema diagnostics, and lateral grids for adaptive refinement.  

##### Use Case  
This function represents the **main detection step** in the extraction pipeline, identifying and tracking lane-marker patterns from LiDAR intensity profiles.  
It follows `fcn_ExtractCL_filterPCinT` and precedes marker separation (`fcn_ExtractCL_separateLaneMarkers`) and centerline computation.  
It is best suited for structured roads or test tracks where paint-marked lane boundaries are visible.  

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_plotCenterLineXY_ZoomIn.png" alt="fcn_ExtractCL_extractLaneMarkers picture" width="600" height="400">
  <figcaption>Fig.4 – Lane-marker extraction using extrema filtering and adaptive pattern matching in the (S, T) domain.</figcaption>
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_createLanePattern  

The function `fcn_ExtractCL_createLanePattern` generates **1D binary lane-marker templates** used for pattern matching in LiDAR intensity strips.  
These templates encode typical U.S. roadway configurations such as **double-yellow** and **solid-white** markings, allowing consistent lateral structure matching across S-strips.

##### Key Features  
- Creates binary templates representing **lane-marker layouts** (e.g., double-yellow + solid-white).  
- Defines marker placement based on **lane width**, **marker width**, and **inter-marker gap**.  
- Parameterized by lateral resolution (`t_res`) for direct alignment with ST-grid sampling.  
- Supports optional visualization for debugging or documentation.  
- Fully deterministic—no filtering or fitting is performed here.  

##### Inputs  
- `lane_type`: string, pattern type.  
  Supported:  
  - `'left_double_yellow_right_white'`  
  - `'single_strip'`  
- `t_res`: scalar, lateral resolution (m).  
- *(Optional)* `lane_width`: lane width (default `3.6 m`).  
- *(Optional)* `marker_width`: marker width (default `0.10 m`).  
- *(Optional)* `double_marker_gap`: gap between two yellow markers (default `0.12 m`).  
- *(Optional)* `fig_num`: figure number for plotting (default `-1`, off).  

##### Outputs  
- `pattern_template`: binary column vector (`0/1`) representing the specified lane layout along the lateral axis.  

##### Use Case  
This function is typically called once at the start of the pipeline (e.g., by `fcn_ExtractCL_extractLaneMarkers`) to initialize the **reference lane-marker pattern**. During adaptive detection, the template may be refined or replaced based on observed layout transitions. Please check `script_test_fcn_ExtractCL_extractLaneMarkers` for details.

<pre align="center">
  <img src=".\Images\binary_template_double_yellow_solid_white.png" alt="fcn_ExtractCL_createLanePattern picture" width="600">
  <figcaption>Fig.5 – Example generated binary pattern for a double-yellow + solid-white lane configuration.</figcaption>
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_organizePointCloudST  

The function `fcn_ExtractCL_organizePointCloudST` converts **unstructured LiDAR points** in the (S, T) frame into a **regularized grid** of longitudinal “strips,” effectively forming **artificial LiDAR rings**.  
This grid-based representation enables efficient extrema filtering and template matching during lane-marker detection.

##### Key Features  
- Converts irregular (S, T) LiDAR samples into evenly spaced **S–T grid strips**.  
- Uses **boxcar aggregation** (sliding-window averaging) along S — no interpolation or convolution.  
- Computes mean **intensity** and **elevation** for each (S, T) cell.  
- Automatically filters strips with low point density or poor lateral coverage.  
- Fully self-contained with optional parameters for window size, margins, and thresholds.  

##### Inputs  
- `s_bin`, `t_bin`, `intensity_bin`, `z_bin`: `[N×1]` vectors of LiDAR data for one longitudinal bin.  
- `s_low`: starting S value of the current section [m].  
- `s_res`: sampling step (strip spacing) [m].  
- `s_width`: total longitudinal window width [m].  
- `t_res`: lateral histogram bin width [m].  
- `t_edges`: vector of lateral bin edges [m].  
- *(Optional Name–Value pairs)*  
  - `'TMargin'` – lateral margin factor (default 0.20)  
  - `'MinTRangeWidth'` – minimum lateral range (default 1.50 m)  
  - `'WindowWidthS'` – aggregation width in S (default `s_res`)  
  - `'MinPointsPerStrip'` – minimum number of points per S-strip (default 50)  
  - `'MinCoverageRatio'` – minimum non-empty T-bin fraction (default 0.15)  

##### Outputs  
- `S_interp`, `T_interp`: `[M×K]` coordinate grids of S and T values.  
- `I_interp`: `[M×K]` mean intensity map.  
- `Z_interp`: `[M×K]` mean elevation map.  
- `s_strip_centers`: `[1×K]` centers of longitudinal strips [m].  

##### Use Case  
This function is called by `fcn_ExtractCL_extractLaneMarkers` to organize raw (S, T) data into intensity images suitable for **extrema filtering** and **pattern matching**.  
Each S-strip acts as a pseudo-scanline, analogous to a LiDAR ring, providing locally structured data while preserving geometric fidelity. Please check `script_test_fcn_ExtractCL_organizePointCloudST.m` for details


<pre align="center">
  <img src=".\Images\organize_pointcloud_sub3.png" alt="fcn_ExtractCL_organizePointCloudST picture" width="500" height="400">
  <figcaption>Fig.6 - The function fcn_ExtractCL_organizePointCloudST is essential for downstream tasks.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>


***
#### fcn_ExtractCL_findLaneMarkersByPattern  

The function `fcn_ExtractCL_findLaneMarkersByPattern` performs **lane-marker detection** on a per-strip basis using **extrema filtering** and **pattern matching** in the (S, T) intensity grid.  
It refines candidate peaks along each lateral profile (`T` direction) to identify lane-marker locations that best match a predefined or adaptive binary lane template.

##### Key Features  
- Applies **Savitzky–Golay smoothing** to suppress noise before extrema filtering.  
- Builds an **extrema correlation profile** to emphasize paint-marked lane edges.  
- Performs **template-based matching** against the expected lane-marker pattern.  
- Adapts the reference template when persistent mismatches are detected.  
- Supports **history-guided matching** when previous `HistoryData` is available.  
- Returns a binary mask of detected markers along with diagnostic filters and fit errors.  

##### Inputs  
- `intensity_data`: `[N×M]` matrix of organized LiDAR intensity in the (S, T) grid.  
- `pattern_template`: column vector, baseline binary lane-marker pattern.  
- `T_resolution`: scalar, lateral resolution [m].  
- `templateUpdateCount`: scalar counter tracking prior adaptive updates.  
- `t_profile`: `[N×M]` matrix of lateral coordinate values corresponding to `intensity_data`.  
- *(Optional)* `fig_num`: scalar, figure number for per-strip visualization.  
- *(Optional)* `HistoryData`: struct containing prior `CenterLine`, `LanePattern`, and `T_Ref` for adaptive matching.  
- *(Optional)* `pointcloud_in_bin`: raw points in the current S-bin, `[N×≥10]`.  
- *(Optional)* `s_strip_edges`, `t_strip_edges`: ST grid boundaries used to locate matching windows.  

##### Outputs  
- `lane_marker_mask`: `[N×M]` logical array, true where lane markers are detected.  
- `pattern_cell`: `{M×1}` cell, detected binary pattern per strip.  
- `extrema_filter_cell`: `{M×1}` cell, extrema filter coefficients for each strip.  
- `best_pattern_template`: updated binary template, adapted if pattern drift detected.  
- `best_fit_errors`: vector of per-strip template matching errors.  

##### Use Case  
This function is the **core pattern-matching step** inside `fcn_ExtractCL_extractLaneMarkers`.  
Each longitudinal strip of the (S, T) grid is analyzed laterally to locate lane markers, compare them to an expected configuration, and update the pattern adaptively. It forms the bridge between raw LiDAR intensity analysis and structured lane-marker extraction.

<pre align="center">
  <img src=".\Images\pattern_matching_result.png" alt="fcn_ExtractCL_findLaneMarkersByPattern picture" width="600">
  <figcaption>Fig.7 – Extrema filtering and adaptive pattern matching for lane-marker detection along each ST strip.</figcaption>
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***


***
#### fcn_ExtractCL_matchPattern  

The function `fcn_ExtractCL_matchPattern` performs **1D sliding-window template matching** between a signal and a reference pattern, identifying the position of the best alignment.  
It is used in the **lane-marker detection pipeline** to match filtered intensity or extrema profiles against predefined lane-marker templates (e.g., solid or double lines).

##### Key Features  
- Performs efficient **sliding-window matching** between input intensity data and a known pattern.  
- Computes both **Mean Squared Error (MSE)** and **zero-mean Normalized Cross-Correlation (zNCC)**.  
- Combines multiple metrics into a single hybrid score for robust alignment.  
- Supports optional neighborhood restriction via `'CandidateIdx'` and `'HalfWindowIdx'`.  
- Fully vectorized and independent of external dependencies.  

##### Inputs  
- `intensity_data` : `[N×1]` vector — 1D signal (e.g., lateral intensity profile).  
- `pattern_template` : `[M×1]` vector — reference template pattern (M ≤ N).  
- *(Optional Name–Value pairs)*  
  - `'CandidateIdx'` — indices to constrain matching region (default: `[]`).  
  - `'HalfWindowIdx'` — half-width of neighborhood window (default: 5).  

##### Outputs  
- `best_fit_pattern` : `[N×1]` vector — template aligned to its best match position.  
- `best_fit_idx` : scalar — index of optimal match in the input signal.  
- `best_fit_error` : scalar — MSE of the best-matched region.  
- `meanSquaredError_Combined` : `[K×1]` vector — combined score across all valid shifts.  

##### Use Case  
`fcn_ExtractCL_matchPattern` is called inside `fcn_ExtractCL_findLaneMarkersByPattern` to locate lane-marker candidates that best match a reference pattern.  
By evaluating both shape similarity (zNCC) and intensity residuals (MSE), this function ensures accurate detection of solid and dashed lane markers under variable LiDAR reflectivity.


<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_projectPC_STToENU  

The function `fcn_ExtractCL_projectPC_STToENU` projects lane-marker or peak points from the **(S, T)** curvilinear coordinate frame back into **ENU coordinates** using pre-computed segment geometry.  
Each segment encodes local tangent and normal vectors, allowing precise spatial reconstruction of detected lane features in the global frame.

##### Key Features  
- Converts detected **(S, T)** coordinates to **(X, Y)** positions in ENU.  
- Uses segment-wise geometry built from a reference trajectory.  
- Supports non-uniform segment spacing and variable curvature.  
- Handles boundary conditions (first / last segment) gracefully.  
- Lightweight and vectorized — no iterative search or interpolation overhead.  

##### Inputs  
- `S_extrema` : `[N×1]` vector — station values of detected extrema or peaks [m].  
- `T_extrema` : `[N×1]` vector — lateral offsets [m].  
- `Seg` : struct containing segment-wise geometry, including:  
  - `.ref_station` — full station sequence [m]  
  - `.traj_start`, `.traj_end` — segment endpoints [X Y]  
  - `.segment` — segment vectors [end − start]  
  - `.segment_length` — Euclidean length per segment  
  - `.seg_tangent` — unit tangent vectors  
  - `.seg_normal` — unit left-normal vectors  
  - `.seg_start_station`, `.d_seg` — station start & spacing per segment  

##### Outputs  
- `XY_extrema` : `[N×2]` array — projected ENU coordinates `[X Y]` corresponding to each (S, T) pair.  

##### Use Case  
This function is typically called after lane-marker detection in **ST coordinates**, converting filtered extrema points back to the global ENU frame for visualization, map storage, or geometric validation against high-definition maps. It forms the final step of the ST→ENU back-projection pipeline in lane-centerline extraction.  

***

#### fcn_ExtractCL_separateLaneMarkers  

The function `fcn_ExtractCL_separateLaneMarkers` separates a mixed set of detected **lane-marker points** into left and right groups based solely on their **lateral coordinate (T)**.  
It applies **robust statistics (MAD-based thresholds)** to each side, automatically rejecting outliers and isolating lane islands for quality control.

##### Key Features  
- Classifies LiDAR lane-marker points using **robust median ± 3·MAD** thresholds.  
- Automatically adapts to the average lateral position using a **moving mean separator** (`movmean(T, 100)`).  
- Produces four outputs: left markers, right markers, in-between islands, and global outliers.  
- Fully data-driven — no prior knowledge of lane width required.  
- Handles noisy or sparse detections gracefully with independent side thresholds.  

##### Inputs  
- `XYZST_lane_markers_array` : `[N×5]` numeric array — columns `[X Y Z S T]`.  
  - Contains all detected lane-marker candidate points in the ST domain.  
- *(Optional)* `fig_num` : figure handle to visualize classification results.  

##### Outputs  
- `LaneMarkers` : struct with fields  
  - `.LaneMarkerLeft` — left-side inliers `[M×5]`.  
  - `.LaneMarkerRight` — right-side inliers `[K×5]`.  
- `islands` : `[Q×5]` array — points located between the two side intervals.  
- `outliers` : `[R×5]` array — points outside both threshold bounds.  

##### Use Case  
This function is called after extracting lane-marker candidates in the **ST coordinate frame**.  
It robustly separates left/right lane markers before lane-centerline computation, ensuring clean side-specific data for subsequent centerline fitting or pattern-based analysis.  


***
#### fcn_ExtractCL_computeCLwithLaneMarkers  

The function `fcn_ExtractCL_computeCLwithLaneMarkers` computes the **road centerline** from previously detected **lane markers** and **binary lane-pattern history**.  
It supports two operation modes: directly using the left markers or inferring the centerline from the right markers and lane-pattern structure.

##### Key Features  
- Derives the **road centerline** based on lane-marker geometry and per-station pattern structure.  
- Two operation modes:  
  - **'left'** – directly use left lane markers as reference.  
  - **'right'** – use right lane markers and infer lateral offset using binary lane-pattern peaks.  
- Detects and classifies **single** vs **double** lane markers automatically.  
- Handles ambiguous or missing lane markers via pattern-guided inference.  
- Outputs both the **marker centerline** and the **road centerline** in ENU coordinates.  

##### Inputs  
- `LaneMarkers` : struct  
  - `.LaneMarkerLeft`  — `[N×5]` array `[X Y Z S T]` for left markers.  
  - `.LaneMarkerRight` — `[M×5]` array `[X Y Z S T]` for right markers.  
- `mode` : `'left'` or `'right'` — determines reference side and logic.  
- `HistoryData` : struct containing per-station pattern information  
  - `.S_Ref` — `[N×1]` vector of station values.  
  - `.LanePattern` — `{N×1}` binary vectors of lateral marker presence (0/1).  
  - `.T_Ref` — `{N×1}` lateral coordinate grids.  
  - `.Z_Ref` — `{N×1}` corresponding elevation profiles.  
- `t_res` : scalar — lateral resolution [m] used to scale index offset.  
- `Seg` : struct — segment geometry for projecting (S,T) → (X,Y).  

##### Outputs  
- `RoadCenterLine` : `[K×5]` array `[X Y Z S T]` — computed road centerline points.  
- `LaneMarkerCenterLine` : `[K×5]` array `[X Y Z S T]` — averaged lane-marker reference line.  

##### Use Case  
This function fuses **geometric lane-marker detections** and **binary pattern maps** to reconstruct a consistent road centerline, even when the left lane markings are faded or occluded.  
It is typically invoked after `fcn_ExtractCL_separateLaneMarkers` and before smoothing or poly fitting in the final **lane-centerline extraction** stage.  

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_extractCenterLine_geoplot.png" alt="fcn_ExtractCL_computeCLwithLaneMarkers picture" width="600">
  <figcaption>Fig.8 – Computing the road centerline using lane-marker geometry and per-station binary lane-pattern data.</figcaption>
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>
***

### Basic Support Functions

#### fcn_ExtractCL_loadMatData  

The function `fcn_ExtractCL_loadMatData` loads example data files used throughout this repository for quick testing and demonstration.  
It provides both the **vehicle pose** (`VehiclePose_Example`) and **LiDAR point cloud** (`PointCloud_ENU_Example`) from stored `.mat` files, typically located in the `Data` folder.

##### Key Features  
- Loads pre-saved example data for reproducibility and demo scripts.  
- Supports **custom directories and filenames** for flexible data organization.  
- Performs automatic input validation and error handling.  
- Independent of any external dependencies (uses only MATLAB built-ins).  

##### Inputs  
- *(Optional)* `data_dir` — directory containing `.mat` files (default: `'Data'`).  
- *(Optional)* `vp_file` — vehicle pose filename (default: `'VehiclePoseExample.mat'`).  
- *(Optional)* `pc_file` — point cloud filename (default: `'PointCloudENUExample.mat'`).  

##### Outputs  
- `VehiclePose_Example` — vehicle pose structure or array loaded from MAT file.  
- `PointCloud_ENU_Example` — ENU-referenced LiDAR point cloud loaded from MAT file.  

##### Use Case  
This helper function is used by most **test scripts** (e.g., `script_test_fcn_ExtractCL_extractCenterLine`) to quickly load sample input data before executing the extraction pipeline.  
Users can replace the example files with their own `.mat` datasets as long as the variable names remain consistent.

***


#### fcn_ExtractCL_plotCenterLineXY

The function `fcn_ExtractCL_plotCenterLineXY` plots the center line or reference trajectory in ENU coordinates. For example, the function was used to make the plot below with sample data. It accepts the ENU-referenced reference trajectory and the LiDAR point cloud, and overlays them in a 2D XY view for quick verification.
<pre align="center">
  <img src=".\Images\fcn_ExtractCL_plotCenterLineXY.png" alt="fcn_ExtractCL_plotCenterLineXY picture" width="400" height="300">
  <figcaption>Fig.9 - The function fcn_ExtractCL_plotCenterLineXY plots the center line outputs.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_plotCenterLineLL

The function `fcn_ExtractCL_plotCenterLineLL` converts the extracted center line from ENU to geographic (LLA) coordinates and plots it on a satellite basemap. It provides a quick visual check of how the extracted lane center aligns with real-world map imagery.

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_plotCenterLineLL.png" alt="fcn_ExtractCL_plotCenterLineLL picture" width="600" >
  <figcaption>Fig.10 - The function fcn_ExtractCL_plotCenterLineXY plots the center line outputs.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_plotCenterLineLL_ZoomIn.png" alt="fcn_ExtractCL_plotCenterLineLL picture" width="600">
  <figcaption>Fig.11 - Zoom-in view of the center line geoplot.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

#### fcn_ExtractCL_comparePCinENUandST

The function `fcn_ExtractCL_comparePCinENUandST` visualizes LiDAR point clouds and the reference trajectory in both ENU (global) and ST (station–lateral) coordinates. It filters the data within a specified station range and plots side-by-side views for direct comparison.

<pre align="center">
  <img src=".\Images\fcn_ExtractCL_comparePCinENUandST.png" alt="fcn_ExtractCL_comparePCinENUandST picture" width="600">
  <figcaption>Fig.12 - The function fcn_ExtractCL_comparePCinENUandST compares point cloud in ENU and ST coordiante system.</figcaption>
  <!--font size="-2">Photo by <a href="https://unsplash.com/ko/@samuelchenard?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Samuel Chenard</a> on <a href="https://unsplash.com/photos/Bdc8uzY9EPw?utm_source=unsplash&utm_medium=referral&utm_content=creditCopyText">Unsplash</a></font -->
</pre>

<a href="#featureextraction_laneboundary_extractcl">Back to top</a>

***

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
