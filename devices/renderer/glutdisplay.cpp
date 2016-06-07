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

#include "sys/platform.h"
#include "sys/stl/string.h"
#include "glutdisplay.h"
#include "regression.h"

/* include GLUT for display */
#if defined(__MACOSX__)
#  include <OpenGL/gl.h>
#  include <GLUT/glut.h>
#  include <ApplicationServices/ApplicationServices.h>
#elif defined(__WIN32__)
#  include <windows.h>
#  include <GL/gl.h>   
#  include <GL/glut.h>
#else
#  include <GL/gl.h>   
#  include <GL/glut.h>
#endif

namespace embree
{
	/* logging settings */
	bool log_display = 1;

	/* camera settings */
	extern Vector3f g_camPos;
	extern Vector3f g_camLookAt;
	extern Vector3f g_camUp;
	extern float g_camFieldOfView;
	extern float g_camRadius;
	extern bool g_stereo;
	extern float g_eyeSeparation;
	extern bool g_toeIn;
	extern float g_zeroParallaxDistance;
	Handle<Device::RTCamera> createCamera(const AffineSpace3f& space);
	void clearGlobalObjects();

	/* orbit camera model */
	static AffineSpace3f g_camSpace;
	static float theta;
	static float phi;
	static float psi;

	/* output settings */
	extern int g_refine;
	extern bool g_fullscreen;
	extern bool g_hdrDisplay;
	extern size_t g_width, g_height;
	extern std::string g_format;
	extern int g_numBuffers;

	/* rendering device and global handles */
	extern Device *g_device;
	extern Handle<Device::RTRenderer> g_renderer;
	extern Handle<Device::RTToneMapper> g_tonemapper;
	extern Handle<Device::RTFrameBuffer> g_frameBuffer;
	extern Handle<Device::RTScene> g_render_scene;
	extern std::vector<Handle<Device::RTPrimitive>> g_prims;
	extern std::vector<Handle<Device::RTCamera>> g_stereoCubeCameras;
	extern size_t g_stereoCubeCameraIndex;

	void setLight(Handle<Device::RTPrimitive> light);

	/* other stuff */
	bool g_resetAccumulation = false;

	/* pause mode */
	bool g_pause = false;

	/* regression testing */
	extern bool g_regression;

	/* ID of created window */
	static int g_window = 0;

	float angleX = 0, angleY = 0; // controllable light

	/*************************************************************************************************/
	/*                                  Keyboard control                                             */
	/*************************************************************************************************/

	static float g_speed = 1.0f;

	void keyboardFunc(unsigned char k, int, int)
	{
		switch (k)
		{
		case ' ': {
			g_pause = !g_pause;
			break;
		}
		case 'c': {
			AffineSpace3f cam(g_camSpace.l, g_camSpace.p);
			std::cout << "-vp " << g_camPos.x << " " << g_camPos.y << " " << g_camPos.z << " " << std::endl
				<< "-vi " << g_camLookAt.x << " " << g_camLookAt.y << " " << g_camLookAt.z << " " << std::endl
				<< "-vu " << g_camUp.x << " " << g_camUp.y << " " << g_camUp.z << " " << std::endl;
			break;
		}
		case 'f': glutFullScreen(); break;
		case 'r': g_refine = !g_refine; break;
		case 't': g_regression = !g_regression; break;
		case 'l': g_camRadius = max(0.0f, g_camRadius - 1); break;
		case 'L': g_camRadius += 1; break;
		case '\033': case 'q': case 'Q':
			clearGlobalObjects();
			glutDestroyWindow(g_window);
			exit(0);
			break;

		// For testing of camera facing geometry
		case 'z':
		{
			for (size_t i = 0; i < g_prims.size(); i++) {
				g_device->rtUpdatePrimitive(g_render_scene, i, g_prims[i], g_camSpace.p, g_camUp);
			}
			g_device->rtCommit(g_render_scene);
		}
		break;
		}

		g_resetAccumulation = true;
	}

	void specialFunc(int k, int, int)
	{
		auto modifiers = glutGetModifiers();
		bool strafe = ((modifiers & GLUT_ACTIVE_ALT) == GLUT_ACTIVE_ALT);

		if ((modifiers & GLUT_ACTIVE_CTRL) == GLUT_ACTIVE_CTRL)
		{
			switch (k) {
			case GLUT_KEY_LEFT: angleX += 0.1f; break;
			case GLUT_KEY_RIGHT: angleX -= 0.1f; break;
			case GLUT_KEY_UP: angleY += 0.1f; if (angleY > 0.5f*float(pi)) angleY = 0.5f*float(pi); break;
			case GLUT_KEY_DOWN: angleY -= 0.1f; if (angleY < 0) angleY = 0.0f; break;
			}
		}
		else {
			switch (k) {
			case GLUT_KEY_LEFT:
				if (strafe)
					g_camSpace = g_camSpace * AffineSpace3f::translate(Vector3f(-g_speed, 0, 0));
				else
					g_camSpace = AffineSpace3f::rotate(g_camSpace.p, g_camUp, -0.05f) * g_camSpace;
				break;
			case GLUT_KEY_RIGHT:
				if (strafe)
					g_camSpace = g_camSpace * AffineSpace3f::translate(Vector3f(+g_speed, 0, 0));
				else
					g_camSpace = AffineSpace3f::rotate(g_camSpace.p, g_camUp, +0.05f) * g_camSpace; break;
			case GLUT_KEY_UP:
				if (strafe)
					g_camSpace = g_camSpace * AffineSpace3f::translate(Vector3f(0, +g_speed, 0));
				else
					g_camSpace = g_camSpace * AffineSpace3f::translate(Vector3f(0, 0, +g_speed));
				break;
			case GLUT_KEY_DOWN:
				if (strafe)
					g_camSpace = g_camSpace * AffineSpace3f::translate(Vector3f(0, -g_speed, 0));
				else
					g_camSpace = g_camSpace * AffineSpace3f::translate(Vector3f(0, 0, -g_speed));
				break;
			case GLUT_KEY_PAGE_UP:
				g_camSpace = AffineSpace3f::rotate(g_camSpace.p, normalize(xfmVector(g_camSpace, Vector3f(1, 0, 0))), -0.05f) * g_camSpace;
				break;
			case GLUT_KEY_PAGE_DOWN:
				g_camSpace = AffineSpace3f::rotate(g_camSpace.p, normalize(xfmVector(g_camSpace, Vector3f(1, 0, 0))), +0.05f) * g_camSpace;
				break;
			case GLUT_KEY_HOME:
				g_speed *= 1.2f; std::cout << "speed = " << g_speed << std::endl;
				break;
			case GLUT_KEY_END:
				g_speed /= 1.2f; std::cout << "speed = " << g_speed << std::endl;
				break;
			}
		}
		g_resetAccumulation = true;
	}

	/*************************************************************************************************/
	/*                                   Mouse control                                               */
	/*************************************************************************************************/

	static int mouseMode = 0;
	static int clickX = 0, clickY = 0;

	void clickFunc(int button, int state, int x, int y)
	{
		if (state == GLUT_UP) {
			mouseMode = 0;
			if (button == GLUT_LEFT_BUTTON && glutGetModifiers() == GLUT_ACTIVE_SHIFT) {
				Handle<Device::RTCamera> camera = createCamera(AffineSpace3f(g_camSpace.l, g_camSpace.p));
				Vector3f p;
				bool hit = g_device->rtPick(camera, x / float(g_width), y / float(g_height), g_render_scene, p.x, p.y, p.z);
				if (hit) {
					Vector3f delta = p - g_camLookAt;
					Vector3f right = cross(normalize(g_camUp), normalize(g_camLookAt - g_camPos));
					Vector3f offset = dot(delta, right)*right + dot(delta, g_camUp)*g_camUp;
					g_camLookAt = p;
					g_camPos += offset;
					g_camSpace = AffineSpace3f::lookAtPoint(g_camPos, g_camLookAt, g_camUp);
					g_resetAccumulation = true;
				}
			}
			else if (button == GLUT_LEFT_BUTTON && glutGetModifiers() == (GLUT_ACTIVE_CTRL | GLUT_ACTIVE_SHIFT)) {
				Handle<Device::RTCamera> camera = createCamera(AffineSpace3f(g_camSpace.l, g_camSpace.p));
				Vector3f p;
				bool hit = g_device->rtPick(camera, x / float(g_width), y / float(g_height), g_render_scene, p.x, p.y, p.z);
				if (hit) {
					Vector3f v = normalize(g_camLookAt - g_camPos);
					Vector3f d = p - g_camPos;
					g_camLookAt = g_camPos + v*dot(d, v);
					g_camSpace = AffineSpace3f::lookAtPoint(g_camPos, g_camLookAt, g_camUp);
					g_resetAccumulation = true;
				}
			}
		}
		else {
			if (glutGetModifiers() == GLUT_ACTIVE_CTRL) return;
			clickX = x; clickY = y;
			if (button == GLUT_LEFT_BUTTON && glutGetModifiers() == GLUT_ACTIVE_ALT) mouseMode = 4;
			else if (button == GLUT_LEFT_BUTTON)   mouseMode = 1;
			else if (button == GLUT_MIDDLE_BUTTON) mouseMode = 2;
			else if (button == GLUT_RIGHT_BUTTON)  mouseMode = 3;
		}
	}

	void motionFunc(int x, int y)
	{
		float dClickX = float(clickX - x), dClickY = float(clickY - y);
		clickX = x; clickY = y;

		// Rotate camera around look-at point (LMB + mouse move)
		if (mouseMode == 1) {
#define ROTATE_WITH_FIXED_UPVECTOR 1
#if ROTATE_WITH_FIXED_UPVECTOR
			float angularSpeed = 0.05f / 180.0f * float(pi);
			float theta = dClickX * angularSpeed;
			float phi = dClickY * angularSpeed;

			const Vector3f viewVec = normalize(g_camLookAt - g_camPos);
			float dist = length(g_camLookAt - g_camPos);

			const Vector3f dX = normalize(cross(viewVec, g_camUp));
			const Vector3f dY = normalize(cross(viewVec, dX));

			AffineSpace3f rot_x = AffineSpace3f::rotate(g_camLookAt, dX, phi);

			g_camSpace = rot_x * g_camSpace;
			g_camSpace = AffineSpace3f::rotate(g_camLookAt, dY, theta) * g_camSpace;
			g_camPos = g_camLookAt - dist*xfmVector(g_camSpace, Vector3f(0, 0, 1));
#else
			float angularSpeed = 0.05f / 180.0f * float(pi);
			float mapping = 1.0f;
			if (g_camUp[1] < 0) mapping = -1.0f;
			theta -= mapping * dClickX * angularSpeed;
			phi += dClickY * angularSpeed;

			if (theta < 0) theta += 2.0f * float(pi);
			if (theta > 2.0f*float(pi)) theta -= 2.0f * float(pi);
			if (phi < -1.5f*float(pi)) phi += 2.0f*float(pi);
			if (phi > 1.5f*float(pi)) phi -= 2.0f*float(pi);

			float cosPhi = cosf(phi);
			float sinPhi = sinf(phi);
			float cosTheta = cosf(theta);
			float sinTheta = sinf(theta);
			float dist = length(g_camLookAt - g_camPos);
			g_camPos = g_camLookAt + dist * Vector3f(cosPhi * sinTheta, -sinPhi, cosPhi * cosTheta);
			Vector3f viewVec = normalize(g_camLookAt - g_camPos);
			Vector3f approxUp(0.0f, 1.0f, 0.0f);
			if (phi < -0.5f*float(pi) || phi > 0.5*float(pi)) approxUp = -approxUp;
			Vector3f rightVec = normalize(cross(viewVec, approxUp));
			AffineSpace3f rotate = AffineSpace3f::rotate(viewVec, psi);
			g_camUp = xfmVector(rotate, cross(rightVec, viewVec));
#endif
		}
		// Pan camera (MMB + mouse move)
		if (mouseMode == 2) {
			float panSpeed = 0.00025f;
			float dist = length(g_camLookAt - g_camPos);
			Vector3f viewVec = normalize(g_camLookAt - g_camPos);
			Vector3f strafeVec = cross(g_camUp, viewVec);
			Vector3f deltaVec = strafeVec * panSpeed * dist * float(dClickX)
				+ g_camUp * panSpeed * dist * float(-dClickY);
			g_camPos += deltaVec;
			g_camLookAt += deltaVec;
		}
		// Dolly camera (RMB + mouse move)
		if (mouseMode == 3) {
			float dollySpeed = 0.01f;
			float delta;
			if (fabsf(dClickX) > fabsf(dClickY)) delta = float(dClickX);
			else delta = float(-dClickY);
			float k = powf((1 - dollySpeed), delta);
			float dist = length(g_camLookAt - g_camPos);
			Vector3f viewVec = normalize(g_camLookAt - g_camPos);
			g_camPos += dist * (1 - k) * viewVec;
		}
		// Roll camera (ALT + LMB + mouse move)
		if (mouseMode == 4) {
			float angularSpeed = 0.1f / 180.0f * float(pi);
			psi -= dClickX * angularSpeed;
			Vector3f viewVec = normalize(g_camLookAt - g_camPos);
			Vector3f approxUp(0.0f, 1.0f, 0.0f);
			if (phi < -0.5f*float(pi) || phi > 0.5*float(pi)) approxUp = -approxUp;
			Vector3f rightVec = normalize(cross(viewVec, approxUp));
			AffineSpace3f rotate = AffineSpace3f::rotate(viewVec, psi);
			g_camUp = xfmVector(rotate, cross(rightVec, viewVec));
		}

		g_camSpace = AffineSpace3f::lookAtPoint(g_camPos, g_camLookAt, g_camUp);
		g_resetAccumulation = true;

	}


	/*************************************************************************************************/
	/*                                   Window control                                              */
	/*************************************************************************************************/

	const size_t avgFrames = 4;
	double g_t0 = getSeconds();
	double g_dt[avgFrames] = { 0.0f };
	size_t frameID = 0;

	void displayFunc(void)
	{
		if (g_pause)
			return;

		/* create random geometry for regression test */
		if (g_regression)
			g_render_scene = createRandomScene(g_device, 1, random<int>() % 100, random<int>() % 1000);

		/* set accumulation mode */
		const int accumulate = g_resetAccumulation ? 0 : g_refine;
		g_resetAccumulation = false;

		/* render image */
#if 1
		Handle<Device::RTCamera> camera = /*g_stereo ? g_cameras[g_cameraIndex] : */createCamera(AffineSpace3f(g_camSpace.l, g_camSpace.p));
#else
		Handle<Device::RTCamera> camera = g_device->rtNewCamera("stereo");
		g_device->rtSetTransform(camera, "local2world", copyToArray(AffineSpace3f(g_camSpace.l, g_camSpace.p)));
		g_device->rtSetInt1(camera, "cubeFaceIndex", 0);
		g_device->rtSetFloat3(camera, "origin", g_camPos.x, g_camPos.y, g_camPos.z);
		g_device->rtSetFloat3(camera, "lookAt", g_camLookAt.x, g_camLookAt.y, g_camLookAt.z);
		g_device->rtSetFloat3(camera, "up", g_camUp.x, g_camUp.y, g_camUp.z);
		g_device->rtSetBool1(camera, "toeIn", g_toeIn);
		//g_device->rtSetFloat1(stereoCubeCamera, "zeroParallaxDistance", 200.f);
		g_device->rtCommit(camera);
#endif

		/* render into framebuffer */
		g_device->rtRenderFrame(g_renderer, camera, g_render_scene, g_tonemapper, g_frameBuffer, accumulate);
		g_device->rtSwapBuffers(g_frameBuffer);

		/* draw image in OpenGL */
		const void* ptr = g_device->rtMapFrameBuffer(g_frameBuffer);

		// extern double upload_time;

		// upload_time = getSeconds() - upload_time;
		// std::cout << "upload " << upload_time << std::endl;
		// exit(0);

		glRasterPos2i(-1, 1);
		glPixelZoom(1.0f, -1.0f);

		if (g_format == "RGB_FLOAT32")
			glDrawPixels((GLsizei)g_width, (GLsizei)g_height, GL_RGB, GL_FLOAT, ptr);
		else if (g_format == "RGBA8")
			glDrawPixels((GLsizei)g_width, (GLsizei)g_height, GL_RGBA, GL_UNSIGNED_BYTE, ptr);
		else if (g_format == "RGB8")
			glDrawPixels((GLsizei)g_width, (GLsizei)g_height, GL_RGB, GL_UNSIGNED_BYTE, ptr);
		else
			throw std::runtime_error("unknown framebuffer format: " + g_format);

		glFlush();
		glutSwapBuffers();

		// Lev: for testing
		if (0) {
			Ref<Image> image = null;
			if (g_format == "RGB8")  image = new Image3c(g_width, g_height, (Col3c*)ptr);
			else if (g_format == "RGBA8")  image = new Image4c(g_width, g_height, (Col4c*)ptr);
			else if (g_format == "RGB_FLOAT32")  image = new Image3f(g_width, g_height, (Col3f*)ptr);
			else if (g_format == "RGBA_FLOAT32")  image = new Image4f(g_width, g_height, (Col4f*)ptr);
			else throw std::runtime_error("unsupported framebuffer format: " + g_format);
			storeImage(image, "test.jpg", 100);
		}

		g_device->rtUnmapFrameBuffer(g_frameBuffer);

		/* calculate rendering time */
		double t1 = getSeconds();
		g_dt[frameID % avgFrames] = t1 - g_t0; g_t0 = t1;
		frameID++;

		/* print average render time of previous frames */
		size_t num = 0;
		double dt = 0.0f;
		for (size_t i = 0; i < avgFrames; i++) {
			if (g_dt[i] != 0.0f) {
				dt += g_dt[i]; num++;
			}
		}
		dt /= num;

		std::ostringstream stream;
		stream.setf(std::ios::fixed, std::ios::floatfield);
		stream.precision(2);
		stream << 1.0f / dt << " fps, ";
		stream.precision(2);
		stream << dt*1000.0f << " ms";
		stream << ", " << g_width << "x" << g_height;
		if (log_display)
			std::cout << "display " << stream.str() << std::endl;
		glutSetWindowTitle((std::string("Embree: ") + stream.str()).c_str());

	}

	void reshapeFunc(int w, int h) {
		if (g_width == size_t(w) && g_height == size_t(h)) return;
		glViewport(0, 0, w, h);
		g_width = w; g_height = h;
		g_frameBuffer = g_device->rtNewFrameBuffer(g_format.c_str(), w, h, g_numBuffers);
		glViewport(0, 0, (GLsizei)g_width, (GLsizei)g_height);
		g_resetAccumulation = true;
	}

	void idleFunc() {
		glutPostRedisplay();
	}

	void GLUTDisplay(const AffineSpace3f& camera, float s, Handle<Device::RTScene>& scene)
	{
		g_camSpace = camera;
		g_speed = s;
		g_render_scene = scene;
		scene = nullptr; // GLUT will never end this function, thus cleanup scene by hand

		/* initialize orbit camera model */
		Vector3f viewVec = normalize(g_camLookAt - g_camPos);
		theta = atan2f(-viewVec.x, -viewVec.z);
		phi = asinf(viewVec.y);
		Vector3f approxUp(0.0f, 1.0f, 0.0f);
		if (phi < -0.5f*float(pi) || phi > 0.5*float(pi)) approxUp = -approxUp;
		Vector3f rightVec = normalize(cross(viewVec, approxUp));
		Vector3f upUnrotated = cross(rightVec, viewVec);
		psi = atan2f(dot(rightVec, g_camUp), dot(upUnrotated, g_camUp));

		/* initialize GLUT */
		int argc = 1; char* argv = (char*)"";
		glutInit(&argc, &argv);
		glutInitWindowSize((GLsizei)g_width, (GLsizei)g_height);
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
		glutInitWindowPosition(0, 0);
		g_window = glutCreateWindow("Embree");
		if (g_fullscreen) glutFullScreen();
		//glutSetCursor(GLUT_CURSOR_NONE);
		glutDisplayFunc(displayFunc);
		glutIdleFunc(idleFunc);
		glutKeyboardFunc(keyboardFunc);
		glutSpecialFunc(specialFunc);
		glutMouseFunc(clickFunc);
		glutMotionFunc(motionFunc);
		glutReshapeFunc(reshapeFunc);
		glutMainLoop();
	}
}
