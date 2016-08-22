#ifndef EXTRA_PCL_TYPES_H_
#define EXTRA_PCL_TYPES_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/impl/sac_model_circle3d.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/pcl_macros.h>
#include <pcl/register_point_struct.h>

/**
 * \file pcl/point_types.h
 * Defines all the PCL implemented PointT point type structures
 * \ingroup common
 */

/** @{*/
namespace pcl
{
    /** \brief Members: float label
     * \ingroup common
     */
    struct Probability;

    /** \brief Members: float x, y, z, dgi
     * \ingroup common
     */
    struct PointXYZDGI;

    /** \brief Members: float x, y, z, uint32_t dgi, float probability
     * \ingroup common
     */
    struct PointXYZDGIP;



#define PCL_ADD_DGI                             \
    union                                       \
    {                                           \
        union                                   \
        {                                       \
            struct                              \
            {                                   \
                uint8_t i;                      \
                uint8_t g;                      \
                uint8_t d;                      \
                uint8_t unused;                 \
            };                                  \
            float dgi;                          \
        };                                      \
        uint32_t dgiu;                          \
    };


    PCL_EXPORTS std::ostream& operator << (std::ostream& os, const Probability& p);
    struct Probability
    {
        float probability;

        friend std::ostream& operator << (std::ostream& os, const Probability& p);
    };

    struct EIGEN_ALIGN16 _PointXYZDGI
    {
        PCL_ADD_POINT4D; 
        // This adds the members x,y,z,
        // which can also be accessed using the point (which is float[4])
    
        PCL_ADD_DGI;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

    struct EIGEN_ALIGN16 _PointXYZDGIP
    {
        PCL_ADD_POINT4D; 
        // This adds the members x,y,z,
        // which can also be accessed using the point (which is float[4])
    
        PCL_ADD_DGI;
        float probability;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };

    /** \brief A point structure representing Euclidean xyz coordinates, and the DGI values.
     *
     * Due to historical reasons (PCL was first developed as a ROS package), the
     * RGB information is packed into an integer and casted to a float.
     * The DGI information was written to follow suit. This is
     * something we wish to remove in the near future, but in the meantime, the
     * following code snippet should help you pack and unpack DGI colors in your
     * PointXYZDGIP structure:
     *
     * \code
     * // pack d/g/i into dgi
     * uint8_t d = 255, g = 0, i = 255;    // Example: D light
     * uint32_t dgi = ((uint32_t)d << 16 | (uint32_t)g << 8 | (uint32_t)i);
     * p.dgi = *reinterpret_cast<float*>(&dgi);
     * \endcode
     *
     * To unpack the data into separate values, use:
     *
     * \code
     * PointXYZDGI p;
     * // unpack dgi into d/g/i
     * uint32_t dgi = *reinterpret_cast<int*>(&p.dgi_f);
     * uint8_t d = (dgi >> 16) & 0x0000ff;
     * uint8_t g = (dgi >> 8)  & 0x0000ff;
     * uint8_t i = (dgi)       & 0x0000ff;
     * \endcode
     *
     *
     * Alternatively, from 1.1.0 onwards, you can use p.d, p.g, and p.i directly.
     * [TODO(sbloch): Confirm whether the above ^ is true!]
     *
     * \ingroup common
     */
    PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZDGI& p);
    struct EIGEN_ALIGN16 PointXYZDGI : public _PointXYZDGI
    {
        inline PointXYZDGI (const _PointXYZDGI &p)
        {
            x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
            dgi = p.dgi;
        }

        inline PointXYZDGI ()
        {
            x = y = z = 0.0f;
            data[3] = 1.0f;
            d = g = i = unused = 0;
        }
        inline PointXYZDGI (uint8_t _d, uint8_t _g, uint8_t _i)
        {
            x = y = z = 0.0f;
            data[3] = 1.0f;

            d = _d;
            g = _g;
            i = _i;
            unused = 0;
        }

        inline PointXYZDGI (float _x, float _y, float _z, 
                            uint8_t _d, uint8_t _g, uint8_t _i)
        {

            x = _x;
            y = _y;
            z = _z;
            data[3] = 1.0f;

            d = _d;
            g = _g;
            i = _i;
            unused = 0;
        }

        inline Eigen::Vector3i getDGIVector3i ()
        {
            return (Eigen::Vector3i (d, g, i));
        }
        inline const Eigen::Vector3i getDGIVector3i () const {
            return (Eigen::Vector3i (d, g, i));
        }
        inline Eigen::Vector4i getDGIVector4i ()
        {
            return (Eigen::Vector4i (d, g, i, unused));
        }
        inline const Eigen::Vector4i getDGIVector4i () const {
            return (Eigen::Vector4i (d, g, i, unused));
        }

        friend std::ostream& operator << (std::ostream& os, const PointXYZDGI& p);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };


    PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZDGIP& p);
    struct EIGEN_ALIGN16 PointXYZDGIP : public _PointXYZDGIP
    {
        inline PointXYZDGIP (const _PointXYZDGIP &p)
        {
            x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
            dgiu = p.dgiu;
            probability = p.probability;
        }

        inline PointXYZDGIP ()
        {
            x = y = z = 0.0f;
            data[3] = 1.0f;
            d = g = i = unused = 0;
            probability = 0.5f;
        }

        inline PointXYZDGIP (float _x, float _y, float _z, 
                             uint8_t _d, uint8_t _g, uint8_t _i,
                             float _probability)
        {
            x = _x;
            y = _y;
            z = _z;
            data[3] = 1.0f;
            d = _d;
            g = _g;
            i = _i;
            unused = 0;
            probability = _probability;
        }

        friend std::ostream& operator << (std::ostream& os, const PointXYZDGIP& p);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            };


}  // namespace

// ==============================
// =====POINT_CLOUD_REGISTER=====
// ==============================

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZDGI,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, dgi, dgi)
                                   )
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZDGI, pcl::_PointXYZDGI)

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::_PointXYZDGIP,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (uint32_t, dgiu, dgiu)
                                   (float, probability, probability)
                                   )
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::PointXYZDGIP, pcl::_PointXYZDGIP)

#endif  //#ifndef PCL_EXTRA_TYPES_H_
