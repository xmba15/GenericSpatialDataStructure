# Memo about pcd data #
## Sample ##
```
# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 213
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 213
DATA ascii
0.93773 0.33763 0 4.2108e+06
0.90805 0.35641 0 4.2108e+06
0.81915 0.32 0 4.2108e+06
0.97192 0.278 0 4.2108e+06
```

## Parameters ##
- VERSION - specifies the PCD file version

- FIELDS - specifies the name of each dimension/field that a point can have
```
FIELDS x y z                                # XYZ data
FIELDS x y z rgb                            # XYZ + colors
FIELDS x y z normal_x normal_y normal_z     # XYZ + surface normals
FIELDS j1 j2 j3                             # moment invariants
...
```
- SIZE
```
SIZE - specifies the size of each dimension in bytes. Examples:

unsigned char/char has 1 byte
unsigned short/short has 2 bytes
unsigned int/int/float has 4 bytes
double has 8 bytes
```
- TYPE - specifies the type of each dimension as a char. The current accepted types are:
```
I - represents signed types int8 (char), int16 (short), and int32 (int)
U - represents unsigned types uint8 (unsigned char), uint16 (unsigned short), uint32 (unsigned int)
F - represents float types
```
- COUNT - specifies how many elements does each dimension have.
- WIDTH - specifies the width of the point cloud dataset in the number of points. WIDTH has two meanings:
  + the total number of points in the cloud (equal with POINTS see below) for unorganized datasets;
  + the width (total number of points in a row) of an organized point cloud dataset.

- VIEWPOINT: specifies an acquisition viewpoint for the points in the dataset. This could potentially be later on used for building transforms between different coordinate systems, or for aiding with features such as surface normals, that need a consistent orientation.
- POINTS: total number of points in the cloud
- DATA: specifies the data type that the point cloud data is stored in. As of version 0.7, two data types are supported: ascii and binary.

## Reference ##
- [point cloud tutorial](http://pointclouds.org/documentation/tutorials/pcd_file_format.php)
- [pypcd library](https://github.com/dimatura/pypcd)
- [tinyply](https://github.com/ddiakopoulos/tinyply)
- [happly](https://github.com/nmwsharp/happly)
- [pcl segmentation](https://gitlab.com/taketwo/snc)
- [Kanezaki-san Tutorial](https://staff.aist.go.jp/kanezaki.asako/pdf/SSII2016_AsakoKanezaki_tutorial.pdf)
- [3D Machine Learning](https://github.com/timzhang642/3D-Machine-Learning)
- [Kanezaki-san Tutorial](https://kanezaki.github.io/media/RobotSeminar20180531_AsakoKanezaki.pdf)
