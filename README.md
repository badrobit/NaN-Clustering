NaN-Clustering
==============

This is the source code repository for my masters thesis source code.

# Dependencies
This project requires the following libraries or packages: 

1. OpenNI 1.5.4
   
    ```
    sudo apt-get install libopenni-dev libopenni-sensor-primesense0 
    sudo ln -s /usr/lib/pkgconfig/libopenni.pc /usr/local/lib/pkgconfig/openni-dev.pc 
    ```

2. [PCL Patched PrimeSense Drivers](https://github.com/avin2/SensorKinect/downloads)
3. [Point Cloud Library (PCL 1.7 Required)](http://pointclouds.org/downloads/source.html)
 



== Installing  == 
Drivers: To properly set up OpenNI navigate into the /3rdparty/openni/debian folder of the pcl-trunk. Run the make command and then once it is done install the two debian packages using dpkg -i