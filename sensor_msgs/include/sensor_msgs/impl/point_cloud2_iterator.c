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
#include <string.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/point_cloud2.h>

#include "rcutils/allocator.h"
#include "point_cloud2_iterator.h"

/// @brief Commun device
const pointCloud2Generator_t customDevice[] = _DEVICE_INITIALIZER_;

/// @brief Commun fields 
static sensor_msgs__msg__PointField communPointFields[] = {
    {.name = {.data="x", .size=2, .capacity=2}, .count = 1, .datatype = sensor_msgs__msg__PointField__FLOAT32},  // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="y", .size=2, .capacity=2}, .count = 1, .datatype = sensor_msgs__msg__PointField__FLOAT32},  // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="z", .size=2, .capacity=2}, .count = 1, .datatype = sensor_msgs__msg__PointField__FLOAT32},  // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="rgb", .size=4, .capacity=4}, .count = 3, .datatype = sensor_msgs__msg__PointField__UINT8},  // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="rgba", .size=5, .capacity=5}, .count = 3, .datatype = sensor_msgs__msg__PointField__UINT8},  // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
/*  ST VL custom Point Cloud type */
    {.name = {.data="vl_ambient", .size=11, .capacity=11}, .count = 1, .datatype = sensor_msgs__msg__PointField__UINT32},       // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="vl_target_nbr", .size=14, .capacity=14}, .count = 1, .datatype = sensor_msgs__msg__PointField__UINT8},     // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="vl_spad_nbr", .size=12, .capacity=12}, .count = 1, .datatype = sensor_msgs__msg__PointField__UINT32},      // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="intensity", .size=10, .capacity=10}, .count = 1, .datatype = sensor_msgs__msg__PointField__UINT32},        // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="vl_sigma", .size=9, .capacity=9}, .count = 1, .datatype = sensor_msgs__msg__PointField__UINT16},           // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="vl_reflectance", .size=15, .capacity=15}, .count = 1, .datatype = sensor_msgs__msg__PointField__UINT8},    // abritary offset as calculated over the 'SetPointCloud2FromDevice' function
    {.name = {.data="vl_status", .size=10, .capacity=10}, .count = 1, .datatype = sensor_msgs__msg__PointField__UINT8},         // abritary offset as calculated over the 'SetPointCloud2FromDevice' function

    //  Keep at the tail
    NULL
};

/// @brief Return the size of a datatype (which is an enum of sensor_msgs__msg__PointField__) in bytes
/// @param datatype one of the enums of sensor_msgs::msg::PointField::
/// @return Datatype size
static inline int8_t sizeOfPointField(uint8_t datatype)
{
    switch (datatype)
    {
    case sensor_msgs__msg__PointField__INT8:
    case sensor_msgs__msg__PointField__UINT8:
    {
        return (int8_t) sizeof(uint8_t);
    }
    break;
    case sensor_msgs__msg__PointField__INT16:
    case sensor_msgs__msg__PointField__UINT16:
    {
        return (int8_t) sizeof(uint16_t);
    }
    break;
    case sensor_msgs__msg__PointField__INT32:
    case sensor_msgs__msg__PointField__UINT32:
    case sensor_msgs__msg__PointField__FLOAT32:
    {
        return (int8_t) sizeof(uint32_t);
    }
    break;
    case sensor_msgs__msg__PointField__FLOAT64:
    {
        return (int8_t) sizeof(uint64_t);
    }
    break;
    default:
    {
        //  Wrong field type argv passed
        fprintf(stderr, "PointField of type %d does not exist");
        return -1;
    }
    break;

        //  Should never be reached
        return -1;
    }
}

/// @brief Add point field to PointCLoud2 message
/// @param cloud_msg PointCloud2 message
/// @param field_to_add The point field to add to point cloud frame
/// @param offset Updated field point offset
/// @return Operation status
bool addPointField(sensor_msgs__msg__PointCloud2 *cloud_msg, sensor_msgs__msg__PointField field_to_add, uint32_t * offset)
{
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sensor_msgs__msg__PointField *data = NULL;
    sensor_msgs__msg__PointField **tmp_data_ptr = NULL;

    //  Check args
    if((NULL == cloud_msg) || (NULL == cloud_msg->fields.data))
    {
        return false;
    }

    //  Do we have to increase memory
    if (cloud_msg->fields.capacity <= cloud_msg->fields.size)
    {
        //  First we're storing PointField data pointers as realloc function
        //  might not release them properly if new memory instance created
        if (!(tmp_data_ptr = allocator.zero_allocate(cloud_msg->fields.size, sizeof(sensor_msgs__msg__PointField *), allocator.state)))
        {
            //  Fatal issue : We do not have enough memory
            return false;
        }

        //  Store each fields.data pointer to tempory array
        for (uint32_t i = 0; i < cloud_msg->fields.size; i++)
        {
            tmp_data_ptr[i] = &(cloud_msg->fields.data[i]);
        }

        //  Increase PointCloud2 message field data memory
        if (NULL != (data = (sensor_msgs__msg__PointField *)allocator.reallocate(cloud_msg->fields.data,
                                                                                 (cloud_msg->fields.size + 1) * sizeof(sensor_msgs__msg__PointField),
                                                                                 allocator.state)))
        {
            //  Issue : Not enough memory space to resize PointCloud2
            //  Original pointer and size are kept
            //  variable has no operation done
            return false;
        }

        if (data != cloud_msg->fields.data)
        {
            for (uint32_t i = 0; i < cloud_msg->fields.size; i++)
            {
                if (!sensor_msgs__msg__PointField__fini(tmp_data_ptr[i]))
                {
                    allocator.deallocate(tmp_data_ptr, allocator.state);
                    return false;
                }
            }
        }
        allocator.deallocate(tmp_data_ptr, allocator.state);

        // If reallocation succeeded, memory may or may not have been moved
        // to fulfill the allocation request, invalidating cloud_msg->fields.data.
        cloud_msg->fields.data = data;
        cloud_msg->fields.capacity = cloud_msg->fields.size + 1;
    }

    if (!sensor_msgs__msg__PointField__copy(
            &field_to_add, &(cloud_msg->fields.data[cloud_msg->fields.size])))
    {
        return false;
    }

    //  Update offset
    *offset += (cloud_msg->fields.data[cloud_msg->fields.size].count * sizeOfPointField(cloud_msg->fields.data[cloud_msg->fields.size].datatype));

    //  Update cloud point step
    cloud_msg->point_step = *offset;

    //  Update field size
    cloud_msg->fields.size++;

    return true;
}

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
                                     uint8_t datatype, uint32_t * offset)
{
    sensor_msgs__msg__PointField field_to_add = {
        .name = name,
        .count = count,
        .datatype = datatype,
        .offset = *offset};
    
    if(!addPointField(cloud_msg, field_to_add, offset))
    {
        return false;
    }

    return true;
}


/// @brief Return the number of elements (points) present in the cloud message
/// @param cloud_msg The PointCloud2 message pointer to fetch size from
/// @return The number of point cloud elements present into the 'cloud_msg' frame
inline size_t GetSize(sensor_msgs__msg__PointCloud2 *cloud_msg)
{
    if (NULL != cloud_msg)
    {
        return cloud_msg->data.size / cloud_msg->point_step;
    }

    //  Issue
    fprintf(stderr, "Null argument given");
    return 0;
}

/// @brief Resize point cloud size
/// @param cloud_msg The point cloud message to update
/// @param newSize The new data size
/// @return Operation status
bool Resize(sensor_msgs__msg__PointCloud2 *cloud_msg, size_t newSize)
{
    void * data = NULL;
    size_t total_size = newSize * cloud_msg->point_step, original_size;
    rcutils_allocator_t allocator = rcutils_get_default_allocator();

    //  Check args
    if((NULL == cloud_msg) || (NULL == cloud_msg->data.data))
    {
        return false;
    }

    original_size = cloud_msg->height * cloud_msg->width;
    if(original_size == newSize)
    {
        //  Nothing to do
        return true;
    }

    //  Shall we need to expend the Point cloud data size
    if(newSize > cloud_msg->data.capacity)
    {
        //  Reallocate cloud data size
        if (NULL != (data = allocator.reallocate(cloud_msg->data.data, total_size, allocator.state)))
        {
            return false;
        }

        cloud_msg->data.data = data;
        cloud_msg->data.capacity = newSize;
    }

    //  Update data size
    cloud_msg->data.size = newSize;

    return true;
}

/// @brief Resize point cloud size
/// @param cloud_msg The point cloud message to update
/// @param width Number of width point into the message
/// @param height Number of height point into the message
/// @return Operation status
inline bool Resize(sensor_msgs__msg__PointCloud2 *cloud_msg, uint32_t width, uint32_t height)
{
    bool success = true;
    size_t size = (size_t) (width * height);

    success &= Resize(cloud_msg, (size_t) (width * height));

    cloud_msg->width = width;
    cloud_msg->height = height;
    cloud_msg->row_step = width * cloud_msg->point_step;

    return success;
}

/// @brief Clear point cloud & reset field
/// @param cloud_msg The point cloud message to clear
/// @return Operation status
inline bool Clear(sensor_msgs__msg__PointCloud2 *cloud_msg)
{
    cloud_msg->height = 0;
    cloud_msg->width = 0;
    cloud_msg->row_step = 0;
    cloud_msg->is_dense = false;

    return rosidl_runtime_c__uint8__Sequence__fini(cloud_msg->data);
}

/// @brief Set field array to point cloud fields
/// @param cloud_msg The point cloud message to update
/// @param fieldPoints_array Fields to add to the point cloud
/// @param fieldPoint_size Number of fields to add
/// @return Operation status
bool SetPointCloud2Fields(sensor_msgs__msg__PointCloud2 *cloud_msg,
                            sensor_msgs__msg__PointField * fieldPoints_array,
                            uint32_t fieldPoint_size)
{
    bool success = true;
    uint32_t offset = cloud_msg->fields.data[cloud_msg->fields.size].offset;

    for(uint32_t i = 0; i < fieldPoint_size; i++)
    {
        fieldPoints_array[i].offset = offset;
        if(!(success &= addPointField(cloud_msg, fieldPoints_array[i], &offset)))
        {
            break;
        }
    }

    return success;
}

/// @brief Add point fields to point cloud from the given device
/// @param cloud_msg The point cloud that should receive point field
/// @param deviceName Name of the device to add 
/// @return Operation status
bool SetPointFieldsFromDevice(sensor_msgs__msg__PointCloud2 *cloud_msg,
                            const char * deviceName)
{
    //  Point field offset
    uint32_t offset = cloud_msg->fields.data[cloud_msg->fields.size].offset;
    uint16_t counter = 0, k = 0;
    //  Clear PointCloud data field
    Clear(cloud_msg);

    while(NULL != &customDevice[counter])
    {
        if(0 == strcmp(customDevice[counter].deviceName, deviceName))
        {
            for(uint32_t j = 0; j < customDevice[counter].nbr_pointFields; j++)
            {
                while(NULL != &communPointFields[k])
                {
                    if(0 == memcmp(communPointFields[k].name.data, &(customDevice[counter].pointFieldCapacity[j]), communPointFields[k].name.size))
                    {
                        communPointFields[k].offset = offset;
                        if(!addPointField(cloud_msg, communPointFields[k], &offset))
                        {
                            //  Issue while adding point field
                            return false;
                        }
                        break;
                    }
                    //  Increase k counter
                    k++;
                }
            }
            break;
        }
        //  Increase counter
        counter++;
    }

    return true;
}



// /**
//  * @brief Private function that adds a PointField to the "fields" member of a PointCloud2
//  * @param cloud_msg the PointCloud2 to add a field to
//  * @param name the name of the field
//  * @param count the number of elements in the PointField
//  * @param datatype the datatype of the elements
//  * @param offset the offset of that element
//  * @return the offset of the next PointField that will be added to the PointCloud2
//  */
// static inline uint32_t addPointField(sensor_msgs__msg__PointCloud2 *cloud_msg,
//                                      const rosidl_runtime_c__String name, uint32_t count,
//                                      uint8_t datatype, uint32_t offset)
// {
//     /*********************************************************************************/
//     rcutils_allocator_t allocator = rcutils_get_default_allocator();
//     sensor_msgs__msg__PointField *pc_fields = NULL;

//     //  Does data are consistent ?
//     //  We do extend cloud fields area only if items
//     //  already reached the allowance capacity
//     if (cloud_msg->fields.capacity == cloud_msg->fields.size)
//     {
//         //  Try memory resize and check we had enough memory space for msg resizing ?
//         //  $$WARNING : Could we have memory leak on fields.name.data is realloc return diff pointer ???
//         if (NULL != (pc_fields = (sensor_msgs__msg__PointField *)allocator.reallocate(cloud_msg->fields.data, (cloud_msg->fields.capacity + 1) * sizeof(sensor_msgs__msg__PointField), allocator.state)))
//         {
//             //  Extra space created, we point to the new memory
//             cloud_msg->fields.data = pc_fields;
//             cloud_msg->fields.size++;
//             cloud_msg->fields.capacity++;
//         }
//         else
//         {
//             //  Issue : Not enough memory space to resize PointCloud2
//             //  Original pointer and size are kept so return offset
//             //  variable has no operation done
//             return offset;
//         }
//     }
//     else
//     {
//         //  Otherwise, we take the next available PointField address space
//         cloud_msg->fields.size++;
//     }

//     //  Populate cloud fields with given args
//     std_msgs__msg__String__Sequence__copy(name, &(cloud_msg->fields.data[cloud_msg->fields.size - 1].name));
//     cloud_msg->fields.data[cloud_msg->fields.size - 1].count = count;
//     cloud_msg->fields.data[cloud_msg->fields.size - 1].datatype = datatype;
//     cloud_msg->fields.data[cloud_msg->fields.size - 1].offset = offset;

//     //  Update the offset
//     return offset + count * sizeOfPointField(datatype);
// }
