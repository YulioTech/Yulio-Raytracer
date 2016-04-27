/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "vec2.h"
#include "vec3.h"

namespace embree
{
	template<typename T>
	struct BSphere {
		T center;
		float radius;

		/// Construct a bounding sphere at the origin having radius zero
		__forceinline BSphere() : center(0.f), radius(0.f) { }
		__forceinline BSphere& operator=(const BSphere& other) { center = other.center; radius = other.radius; return *this; }

		__forceinline BSphere(EmptyTy) : center(0.f), radius(0.f) {}
		//__forceinline BSphere(FullTy) : lower(neg_inf), upper(pos_inf) {}
		//__forceinline BSphere(FalseTy) : lower(pos_inf), upper(neg_inf) {}
		//__forceinline BSphere(TrueTy) : lower(neg_inf), upper(pos_inf) {}
		//__forceinline BSphere(NegInfTy) : lower(pos_inf), upper(neg_inf) {}
		//__forceinline BSphere(PosInfTy) : lower(neg_inf), upper(pos_inf) {}

		/*
		/// Unserialize a bounding sphere from a binary data stream
		__forceinline BSphere(Stream *stream) {
			center = Point(stream);
			radius = stream->readFloat();
		}
		*/

		/// Create a bounding sphere from a given center point and radius
		__forceinline BSphere(const T &center, float radius)
			: center(center), radius(radius) {
		}

		/// Copy constructor
		__forceinline BSphere(const BSphere &boundingSphere)
			: center(boundingSphere.center), radius(boundingSphere.radius) {
		}

		/// Return whether this bounding sphere has a radius of zero or less.
		__forceinline bool isEmpty() const {
			return radius <= 0.f;
		}

		/// Expand the bounding sphere radius to contain another point.
		__forceinline void expandBy(const T p) {
			radius = std::max(radius, (p - center).length());
		}

		/// Check whether the specified point is inside or on the sphere
		__forceinline bool contains(const T p) const {
			return (p - center).length() <= radius;
		}

		/// Equality test
		__forceinline bool operator==(const BSphere &boundingSphere) const {
			return center == boundingSphere.center && radius == boundingSphere.radius;
		}

		/// Inequality test
		__forceinline bool operator!=(const BSphere &boundingSphere) const {
			return center != boundingSphere.center || radius != boundingSphere.radius;
		}

		/**
		 * \brief Calculate the intersection points with the given ray
		 * \return \c true if the ray intersects the bounding sphere
		 *
		 * \remark In the Python bindings, this function returns the
		 * \c nearT and \c farT values as a tuple (or \c None, when no
		 * intersection was found)
		 */
		__forceinline bool rayIntersect(const Vector3f &rayO, const Vector3f &rayD, float &nearHit, float &farHit) const {
			const Vector3f o = rayO - center;
			const float A = lengthSquared(rayD);
			const float B = 2 * dot(o, rayD);
			const float C = lengthSquared(o) - radius*radius;

			return solveQuadratic(A, B, C, nearHit, farHit);
		}

		/*
		/// Serialize this bounding sphere to a binary data stream
		__forceinline void serialize(Stream *stream) const {
			center.serialize(stream);
			stream->writeFloat(radius);
		}
		*/
	};

	/*! output operator */
	template<typename T> __forceinline std::ostream& operator<<(std::ostream& cout, const BSphere<T>& s) {
		return cout << "[ center = " << s.center << ", radius = " << s.radius << "]";
	}

	/*! default template instantiations */
	typedef BSphere<Vec2f> BSphere2f;
	typedef BSphere<Vector3f> BSphere3f;

} //namespace embree
