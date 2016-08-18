// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#ifndef __EMBREE_DIFFERENTIAL_GEOMETRY_H__
#define __EMBREE_DIFFERENTIAL_GEOMETRY_H__

#include "default.h" 

namespace embree
{
	typedef int light_mask_t;


	/**
	* \brief Stores a three-dimensional orthonormal coordinate frame
	*
	* This class is mostly used to quickly convert between different
	* cartesian coordinate systems and to efficiently compute certain
	* quantities (e.g. \ref cosTheta(), \ref tanTheta, ..).
	*
	* \ingroup libcore
	* \ingroup libpython
	*/
	struct Frame {
		Vector3f s, t;
		Vector3f n;

		__forceinline static void coordinateSystem(const Vector3f &a, Vector3f &b, Vector3f &c) {
			if (std::abs(a.x) > std::abs(a.y)) {
				const float invLen = rcp(std::sqrt(a.x * a.x + a.z * a.z));
				c = Vector3f(a.z * invLen, 0.0f, -a.x * invLen);
			}
			else {
				const float invLen = rcp(std::sqrt(a.y * a.y + a.z * a.z));
				c = Vector3f(0.0f, a.z * invLen, -a.y * invLen);
			}
			b = cross(c, a);
		}

		__forceinline static void computeShadingFrame(const Vector3f &n, const Vector3f &dpdu, Frame &frame) {
			frame.n = n;
			frame.s = normalize(dpdu - frame.n
				* dot(frame.n, dpdu));
			frame.t = cross(frame.n, frame.s);
		}

		/// Default constructor -- performs no initialization!
		__forceinline Frame() { }

		/// Construct a frame from the given orthonormal vectors
		/// Given a normal and tangent vectors, construct a new coordinate frame
		__forceinline Frame(const Vector3f &x, const Vector3f &y, const Vector3f &z)
			: s(x), t(y), n(z) {
		}

		/// Construct a new coordinate frame from a single vector
		__forceinline Frame(const Vector3f &n) : n(n) {
			coordinateSystem(n, s, t);
		}

		/*
		/// Unserialize from a binary data stream
		__forceinline Frame(Stream *stream) {
			s = Vector(stream);
			t = Vector(stream);
			n = Normal(stream);
		}

		/// Serialize to a binary data stream
		__forceinline void serialize(Stream *stream) const {
			s.serialize(stream);
			t.serialize(stream);
			n.serialize(stream);
		}
		*/

		/// Convert from world coordinates to local coordinates
		__forceinline Vector3f toLocal(const Vector3f &v) const {
			return Vector3f(
				dot(v, s),
				dot(v, t),
				dot(v, n)
				);
		}

		/// Convert from local coordinates to world coordinates
		__forceinline Vector3f toWorld(const Vector3f &v) const {
			return s * v.x + t * v.y + n * v.z;
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the squared cosine of the angle between the normal and v */
		__forceinline static float cosTheta2(const Vector3f &v) {
			return v.z * v.z;
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the cosine of the angle between the normal and v */
		inline static float cosTheta(const Vector3f &v) {
			return v.z;
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the u and v coordinates of the vector 'v' */
		__forceinline static Vec2f uv(const Vector3f &v) {
			return Vec2f(v.x, v.y);
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the squared sine of the angle between the normal and v */
		__forceinline static float sinTheta2(const Vector3f &v) {
			return 1.0f - v.z * v.z;
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the sine of the angle between the normal and v */
		__forceinline static float sinTheta(const Vector3f &v) {
			const float temp = sinTheta2(v);
			if (temp <= 0.0f)
				return 0.0f;
			return std::sqrt(temp);
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the tangent of the angle between the normal and v */
		__forceinline static float tanTheta(const Vector3f &v) {
			const float temp = 1 - v.z*v.z;
			if (temp <= 0.0f)
				return 0.0f;
			return std::sqrt(temp) / v.z;
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the squared tangent of the angle between the normal and v */
		__forceinline static float tanTheta2(const Vector3f &v) {
			const float temp = 1 - v.z*v.z;
			if (temp <= 0.0f)
				return 0.0f;
			return temp / (v.z * v.z);
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the sine of the phi parameter in spherical coordinates */
		__forceinline static float sinPhi(const Vector3f &v) {
			const float sinTheta = Frame::sinTheta(v);
			if (sinTheta == 0.0f)
				return 1.0f;
			return clamp(v.y / sinTheta, -1.0f, 1.0f);
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the cosine of the phi parameter in spherical coordinates */
		__forceinline static float cosPhi(const Vector3f &v) {
			const float sinTheta = Frame::sinTheta(v);
			if (sinTheta == 0.0f)
				return 1.0f;
			return clamp(v.x / sinTheta, -1.0f, 1.0f);
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the squared sine of the phi parameter in  spherical
		* coordinates */
		__forceinline static float sinPhi2(const Vector3f &v) {
			return clamp(v.y * v.y / sinTheta2(v), 0.0f, 1.0f);
		}

		/** \brief Assuming that the given direction is in the local coordinate
		* system, return the squared cosine of the phi parameter in  spherical
		* coordinates */
		__forceinline static float cosPhi2(const Vector3f &v) {
			return clamp(v.x * v.x / sinTheta2(v), 0.0f, 1.0f);
		}

		/// Equality test
		__forceinline bool operator==(const Frame &frame) const {
			return frame.s == s && frame.t == t && frame.n == n;
		}

		/// Inequality test
		__forceinline bool operator!=(const Frame &frame) const {
			return !operator==(frame);
		}

		/*
		/// Return a string representation of this frame
		inline std::string toString() const {
			std::ostringstream oss;
			oss << "Frame[" << std::endl
				<< "  s = " << s.toString() << "," << std::endl
				<< "  t = " << t.toString() << "," << std::endl
				<< "  n = " << n.toString() << std::endl
				<< "]";
			return oss.str();
		}
		*/
	};

	/*! Contains additional shading information for hit points. */
	struct DifferentialGeometry
	{
		/*! Default construction. */
		__forceinline DifferentialGeometry()
			: material(nullptr), light(nullptr) {}

	public:
		class Material*  material; //!< pointer to material of hit shape instance
		class AreaLight* light;    //!< pointer to area light of hit shape instance

	public:
		Vector3f P;                //!< Hit location in world coordinates.
		Vector3f Tx;               //!< Tangent in x direction
		Vector3f Ty;               //!< Tangent in y direction
		Vector3f Ng;               //!< Normalized geometry normal.
		mutable Vector3f Ns;       //!< Normalized shading normal.
		Vec2f st;                  //!< Hit location in surface parameter space.
		float error;               //!< Intersection error factor.
		light_mask_t illumMask;    //!< bit mask which light we're interested in
		light_mask_t shadowMask;   //!< bit mask which light we're interested in
		Vector3f wi;			   //!< Incident direction in the local shading frame
		//Frame shadingFrame;
	};
}

#endif
