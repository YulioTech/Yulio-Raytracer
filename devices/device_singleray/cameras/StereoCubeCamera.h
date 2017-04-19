#pragma once

#include "camera.h"

namespace embree
{
#define EYE_SEPARATION (6.35f * 0.393701f)
#define ZERO_PARALLAX (EYE_SEPARATION * 30.f) // Zero parallax distance as per Paul Bourke's recommendations

	/*! Implements the stereoscopic cube map camera model. */
	class StereoCubeCamera : public Camera
	{
	public:

		/*! Construction from parameter container. */
		StereoCubeCamera(const Parms& parms)
		{
			local2world = parms.getTransform("local2world");
			angle = 90.f;
			aspectRatio = 1.f;
			
			name = parms.getString("name", "Unassigned Camera Name");
			cubeFaceIndex = parms.getInt("cubeFaceIndex", 0);

			origin = parms.getVector3f("origin", local2world.p);
			lookAt = parms.getVector3f("lookAt", Vector3f(0.f, 0.f, -1.f));
			up = parms.getVector3f("up", Vector3f(0.f, 1.f, 0.f));
			right = cross(normalize(up), normalize(lookAt - origin));
			sceneScale = parms.getFloat("sceneScale", 1.f);
			eyeSeparation = parms.getFloat("eyeSeparation", EYE_SEPARATION) * sceneScale;
			zeroParallaxDistance = parms.getFloat("zeroParallaxDistance", ZERO_PARALLAX) * sceneScale;
			if (zeroParallaxDistance != 0.f) {
				rcpZeroParallaxDistance = 1.f / zeroParallaxDistance;
				toeIn = parms.getBool("toeIn", false);
			}
			else {
				rcpZeroParallaxDistance = 0.f;
				toeIn = false;
			}
			absolutVerticalAngleStereoStartsToFallOff = clamp(parms.getFloat("stereFalloffAngle", 30.f), 0.f, 90.f);

			const Vector3f W = xfmVector(local2world, Vector3f(-.5f * aspectRatio, -.5f, .5f * rcp(tanf(deg2rad(.5f * angle)))));
			
			// Front image
			pixel2world[0] = AffineSpace3f(aspectRatio * local2world.l.vx, local2world.l.vy, W, local2world.p);
			xyzStraight = normalize(.5f * pixel2world[0].l.vx + .5f * pixel2world[0].l.vy + pixel2world[0].l.vz);

			// Right image
			pixel2world[1] = AffineSpace3f::rotate(origin, up, deg2rad(90.f)) * pixel2world[0];
			
			// Back image
			pixel2world[2] = AffineSpace3f::rotate(origin, up, deg2rad(180.f)) * pixel2world[0];
			
			// Left image
			pixel2world[3] = AffineSpace3f::rotate(origin, up, deg2rad(-90.f)) * pixel2world[0];

			// Up image
			pixel2world[4] = AffineSpace3f::rotate(origin, right, deg2rad(-90.f)) * pixel2world[0];
			// Needs to be upside-down relative to the front image (this is how the cube map in the original Gear VR was implemented for some reason, so we stuck to it for compatibility reasons)
			pixel2world[4] = AffineSpace3f::rotate(origin, up, deg2rad(180.f)) * pixel2world[4];

			// Down image
			pixel2world[5] = AffineSpace3f::rotate(origin, right, deg2rad(90.f)) * pixel2world[0];
			// Needs to be upside-down relative to the front image (this is how the cube map in the original Gear VR was implemented for some reason, so we stuck to it for compatibility reasons)
			pixel2world[5] = AffineSpace3f::rotate(origin, up, deg2rad(180.f)) * pixel2world[5];
		}

		void ray(const Vec2f& pixel, const Vec2f& sample, Ray& ray_o) const {

			const auto &pixel2world = this->pixel2world[0];
			const auto eyeCubeFaceIndex = cubeFaceIndex % 6;
			const auto yPixel = 1.0f - pixel.y;

			// Step 1: Choose the camera orientation according to the cube face we're processing
			auto p2w = this->pixel2world[eyeCubeFaceIndex];

			// Step 2: Determine the angles for the ray matrix rotation and stereo falloff
			float theta = 0.f, phi = 0.f, omega = 0.f;
			float absoluteVerticalAngle = 0.f;
			{
				switch (eyeCubeFaceIndex) {
					case 0:
					case 1:
					case 2:
					case 3:
					{
						const Vector3f xDir = normalize(pixel.x * pixel2world.l.vx + .5f * pixel2world.l.vy + pixel2world.l.vz);
						theta = acosf( clamp(dot(xDir, xyzStraight), -1.f, 1.f) ) * sign(pixel.x - .5f);

						const Vector3f yDir = normalize(.5f * pixel2world.l.vx + yPixel * pixel2world.l.vy + pixel2world.l.vz);
						const auto yAngle = rad2deg(acosf( clamp(dot(yDir, xyzStraight), -1.f, 1.f) )) * sign(yPixel - .5f);

						absoluteVerticalAngle = fabsf(yAngle);
					}
					break;
					case 4:
					{
						Vector3f xyDir(pixel.x - .5f, yPixel - .5f, 0.f);

						Vector3f xyDirNorm = normalize(xyDir);
						static const Vector3f xyUp = Vector3f(0.f, -1.f, 0.f);

						theta = acosf( clamp(dot(xyDirNorm, xyUp), -1.f, 1.f) ) * sign(pixel.x - .5f);
						const Vector3f xyzDir = normalize(pixel.x * pixel2world.l.vx + yPixel * pixel2world.l.vy + pixel2world.l.vz);
						const auto xyzAngle = rad2deg(acosf( clamp(dot(xyzDir, xyzStraight), -1.f, 1.f) ));

						absoluteVerticalAngle = 90.f - fabsf(xyzAngle);
					}
					break;
					case 5:
					{
						Vector3f xyDir(pixel.x - .5f, yPixel - .5f, 0.f);

						Vector3f xyDirNorm = normalize(xyDir);
						static const Vector3f xyUp = Vector3f(0.f, 1.f, 0.f);

						theta = acosf( clamp(dot(xyDirNorm, xyUp), -1.f, 1.f) ) * sign(pixel.x - .5f);
						const Vector3f xyzDir = normalize(pixel.x * pixel2world.l.vx + yPixel * pixel2world.l.vy + pixel2world.l.vz);
						const auto xyzAngle = rad2deg(acosf( clamp(dot(xyzDir, xyzStraight), -1.f, 1.f) ));

						absoluteVerticalAngle = 90.f - fabsf(xyzAngle);
					}
					break;
				}
			}

			// Step 3: Calculate the effective eye separation and apply the translation to the camera matrix
			auto eyeOffset = eyeSeparation * (cubeFaceIndex < 6 ? -.5f : .5f);
			{
				if (absoluteVerticalAngle > absolutVerticalAngleStereoStartsToFallOff) {
					// Do a smooth fall-off of the stereo
					//const auto stereoFalloffCoefficient = 1.f - smootherstep(absolutVerticalAngleStereoStartsToFallOff, 90.f, absoluteVerticalAngle);
					const auto stereoFalloffCoefficient = 1.f - smoothstep(0.f, 1.f, smoothstep(absolutVerticalAngleStereoStartsToFallOff, 90.f, absoluteVerticalAngle));
					//const auto stereoFalloffCoefficient = 1.f - smootherstep(0.f, 1.f, smootherstep(absolutVerticalAngleStereoStartsToFallOff, 90.f, absoluteVerticalAngle));
					//static const float rcpFalloffAngleRange = rcp(90.f - absolutVerticalAngleStereoStartsToFallOff);
					//const auto stereoFalloffCoefficient = 1.f - (absoluteVerticalAngle > absolutVerticalAngleStereoStartsToFallOff
					//	? (absoluteVerticalAngle - absolutVerticalAngleStereoStartsToFallOff) * rcpFalloffAngleRange
					//	: 0.f);
					eyeOffset *= stereoFalloffCoefficient;
				}

				const Vector3f eyeTranslation = Vector3f(eyeOffset, 0.f, 0.f);
				p2w = p2w * AffineSpace3f::translate(eyeTranslation);
			}

			// Step 4: Orient the camera matrix according to the calculated angles and the chosen stereoscopy approach (either parallel or toe-in)
			Vector3f rayOrigin = zero;
			{
				const auto rayRotationSpace = AffineSpace3f::rotate(origin, up, theta);
				rayOrigin = (rayRotationSpace * p2w).p;

				if (toeIn) {
					const auto toeInCorrection = -atanf(eyeOffset * rcpZeroParallaxDistance);
					const auto toiInRotationSpace = AffineSpace3f::rotate(rayOrigin, up, toeInCorrection);
					p2w = toiInRotationSpace * p2w;
				}
			}

			// Final step: Generate a new ray
			new (&ray_o) Ray(rayOrigin, normalize(pixel.x * p2w.l.vx + yPixel * p2w.l.vy + p2w.l.vz));
		}

	protected:
		float aspectRatio;
		AffineSpace3f local2world;    //!< transformation from camera space to world space
		AffineSpace3f pixel2world[6];    //!< special transformation to generate rays
		std::string name;
		int cubeFaceIndex;
		Vector3f origin;
		Vector3f lookAt;
		Vector3f up, right;
		Vector3f xyzStraight;
		float zeroParallaxDistance, rcpZeroParallaxDistance;
		bool toeIn;
		float sceneScale; // A multiplier used to scale the eye separation, toe-in and other related values
		float eyeSeparation; // In cm
		float absolutVerticalAngleStereoStartsToFallOff; // Need to experiment with this value
	};
}