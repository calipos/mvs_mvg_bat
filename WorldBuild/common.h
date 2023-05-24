#ifndef _COMMON_H_
#define _COMMON_H_
#include <iostream>
#include <unordered_map>
#include <numeric>
#include "Eigen/Core"
#include <Eigen/Dense>
namespace rt
{

	/// Define 3D-2D tracking data: 3D landmark with its 2D observations
	struct Observation
	{
		Observation() :id_feat(std::numeric_limits<uint32_t>::max()) {  }
		Observation(const Eigen::Vector2d& p, uint32_t idFeat) : x(p), id_feat(idFeat) {}

		Eigen::Vector2d x;
		uint32_t id_feat;

		// Serialization
		template <class Archive>
		void save(Archive& ar) const;

		// Serialization
		template <class Archive>
		void load(Archive& ar);
	}; 
	struct Landmark
	{
		Eigen::Vector3d X;
		std::unordered_map<uint32_t, Observation> obs;

		// Serialization
		template <class Archive>
		void save(Archive& ar) const;

		template <class Archive>
		void load(Archive& ar);
	};

	enum class ETriangulationMethod : unsigned char
	{
		DIRECT_LINEAR_TRANSFORM, // DLT
		L1_ANGULAR,
		LINFINITY_ANGULAR,
		INVERSE_DEPTH_WEIGHTED_MIDPOINT,
		DEFAULT = INVERSE_DEPTH_WEIGHTED_MIDPOINT
	};
	/**
* @brief Defines a pose in 3d space
* [R|C] t = -RC
*/
	class Pose3
	{
	protected:

		/// Orientation matrix
		Eigen::Matrix3d rotation_;

		/// Center of rotation
		Eigen::Vector3d center_;

	public:

		/**
		* @brief Constructor
		* @param r Rotation
		* @param c Center
		* @note Default (without args) defines an Identity pose.
		*/
		Pose3
		(
			const Eigen::Matrix3d& r = std::move(Eigen::Matrix3d::Identity()),
			const Eigen::Vector3d& c = std::move(Eigen::Vector3d::Zero())
		)
			: rotation_(r), center_(c) {}

		/**
		* @brief Get Rotation matrix
		* @return Rotation matrix
		*/
		const Eigen::Matrix3d& rotation() const
		{
			return rotation_;
		}

		/**
		* @brief Get Rotation matrix
		* @return Rotation matrix
		*/
		Eigen::Matrix3d& rotation()
		{
			return rotation_;
		}

		/**
		* @brief Get center of rotation
		* @return center of rotation
		*/
		const Eigen::Vector3d& center() const
		{
			return center_;
		}

		/**
		* @brief Get center of rotation
		* @return Center of rotation
		*/
		Eigen::Vector3d& center()
		{
			return center_;
		}

		/**
		* @brief Get translation vector
		* @return translation vector
		* @note t = -RC
		*/
		inline Eigen::Vector3d translation() const
		{
			return -(rotation_ * center_);
		}


		/**
		* @brief Apply pose
		* @param p Point
		* @return transformed point
		*/
		template<typename T>
		inline typename T::PlainObject operator() (const T& p) const
		{
			return rotation_ * (p.colwise() - center_);
		}
		/// Specialization for Vec3
		inline typename Eigen::Vector3d::PlainObject operator() (const Eigen::Vector3d& p) const
		{
			return rotation_ * (p - center_);
		}


		/**
		* @brief Composition of poses
		* @param P a Pose
		* @return Composition of current pose and parameter pose
		*/
		Pose3 operator * (const Pose3& P) const
		{
			return { rotation_ * P.rotation_,
					P.center_ + P.rotation_.transpose() * center_ };
		}


		/**
		* @brief Get inverse of the pose
		* @return Inverse of the pose
		*/
		Pose3 inverse() const
		{
			return { rotation_.transpose(),  -(rotation_ * center_) };
		}

		/**
		* @brief Return the pose as a single Mat34 matrix [R|t]
		* @return The pose as a Mat34 matrix
		*/
		inline Eigen::Matrix<double, 3, 4> asMatrix() const
		{
			return (Eigen::Matrix<double, 3, 4>() << rotation_, translation()).finished();
		}

		/**
		* Serialization out
		* @param ar Archive
		*/
		template <class Archive>
		inline void save(Archive& ar) const;

		/**
		* @brief Serialization in
		* @param ar Archive
		*/
		template <class Archive>
		inline void load(Archive& ar);
	};

}
#endif // _COMMON_H_
