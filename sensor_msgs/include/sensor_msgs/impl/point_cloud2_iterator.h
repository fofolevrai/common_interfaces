// Copyright (c) 2022, fofolevrai
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// This file is originally from:
// https://github.com/ros/common_msgs/blob/50ee957/sensor_msgs/include/sensor_msgs/impl/point_cloud2_iterator.h
#include <stdio.h>
#include <stdlib.h>
#include <sensor_msgs/msg/point_cloud2.h>

typedef struct _pointCloud2Generator_t_{
    char * deviceName;
    rosidl_runtime_c__String * pointFieldCapacity;
    uint16_t nbr_pointFields; 
} pointCloud2Generator_t;

/// @brief Add point field to PointCLoud2 message
/// @param cloud_msg PointCloud2 message
/// @param field_to_add The point field to add to point cloud frame
/// @return Operation status
bool addPointField(sensor_msgs__msg__PointCloud2 *cloud_msg, sensor_msgs__msg__PointField field_to_add);

/**
 * @brief Private function that adds a PointField to the "fields" member of a PointCloud2
 * @param cloud_msg the PointCloud2 to add a field to
 * @param name the name of the field
 * @param count the number of elements in the PointField
 * @param datatype the datatype of the elements
 * @param offset offset pointer of that element
 * @return Operation status
 */
bool addPointField(sensor_msgs__msg__PointCloud2 *cloud_msg,
                                     const rosidl_runtime_c__String name, uint32_t count,
                                     uint8_t datatype, uint32_t * offset);

/// @brief Return the number of elements (points) present in the cloud message
/// @param cloud_msg The PointCloud2 message pointer to fetch size from
/// @return The number of point cloud elements present into the 'cloud_msg' frame
inline size_t GetSize(sensor_msgs__msg__PointCloud2 *cloud_msg);

/// @brief 
/// @param cloud_msg 
/// @param newSize 
/// @return 
bool Resize(sensor_msgs__msg__PointCloud2 *cloud_msg, size_t newSize);

/// @brief 
/// @param cloud_msg 
/// @param width 
/// @param height 
/// @return 
inline bool Resize(sensor_msgs__msg__PointCloud2 *cloud_msg, uint32_t width, uint32_t height);

/// @brief 
/// @param cloud_msg 
/// @return 
inline bool Clear(sensor_msgs__msg__PointCloud2 *cloud_msg);

/// @brief 
/// @param cloud_msg 
/// @param fieldPoints_array 
/// @param fieldPoint_size 
/// @return 
bool SetPointCloud2Fields(sensor_msgs__msg__PointCloud2 *cloud_msg,
                            sensor_msgs__msg__PointField * fieldPoints_array,
                            uint32_t fieldPoint_size);