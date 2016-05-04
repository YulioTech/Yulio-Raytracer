#include "ColladaLoader.h"

// Assimp include files
#include <assimp/Importer.hpp>	//OO version Header!
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <assimp/cimport.h>
#include <assimp/DefaultLogger.hpp>
#include <assimp/LogStream.hpp>

#include "sys/platform.h"
#include "math/vec2.h"
#include "math/vec3.h"
#include "math/quaternion.h"
#include "math/affinespace.h"
#include "loaders.h"

#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <cstdlib>

//#pragma comment(lib,"assimp.lib")

namespace embree {

#define FPR_VIEW_CAMERA_PREFIX		"KISP_FPR_VIEW_"
#define CAMERA_ALIGNED_NODE_PREFIX	"KISP_CAMERA_ALIGNED_"

	class DAELoader
	{
	public:
		DAELoader(const FileName& fileName);
		~DAELoader();

	private:
		// Assimp related members
		Assimp::Importer importer;
		const aiScene *scene;
		void getNodeBBox(const aiScene *scene, const aiNode *nd, aiVector3D *min, aiVector3D *max, aiMatrix4x4 *trafo);

	private:
		// Embree structures initialization routines
		bool importSceneFromFile(const std::string &pFile);
		bool initSceneMaterials(const aiScene *scene);
		bool initSceneCameras(const struct aiScene *scene, bool toeIn = false);
		void initSceneMeshesRecursive(const struct aiScene *scene, const struct aiNode* node, const aiMatrix4x4 &mParent);

		static std::string getBasePath(const std::string &path) {
			auto pos = path.find_last_of("\\/");
			return (std::string::npos == pos) ? "" : path.substr(0, pos + 1);
		};

	private:
		Handle<Device::RTMaterial> defaultMaterial;
		std::string basePath;
		Vector3f sceneMin, sceneMax, sceneCenter;
		float sceneScale = 1.f;

		// Embree related members
		struct MaterialInfo {
			bool cullBackFaces;
			bool render;
			Handle<Device::RTMaterial> material;

			MaterialInfo() :
				material(null), cullBackFaces(false), render(true)
			{}

			MaterialInfo(Handle<Device::RTMaterial> mtl, bool cbf, bool r = true) :
				material(mtl), cullBackFaces(cbf), render(r)
			{}
		};
		std::map<size_t, MaterialInfo> materialMap;              //!< named materials

	public:
		std::vector<Handle<Device::RTCamera>> cameras; //!< Stores the FPR cameras
		std::vector<Handle<Device::RTPrimitive>> primitives;   //!< stores the output scene
	};

	DAELoader::DAELoader(const FileName& fileName) {

		defaultMaterial = g_device->rtNewMaterial("matte");
		g_device->rtSetFloat3(defaultMaterial, "reflectance", 0.5f, 0.5f, 0.5f);
		g_device->rtCommit(defaultMaterial);

		const std::string fn(fileName.c_str());
		basePath = getBasePath(fn);

		if (!importSceneFromFile(fn)) {
			throw std::runtime_error("Failed to load " + fn + " Collada file");
		}

		initSceneMaterials(scene);
		initSceneCameras(scene);
		initSceneMeshesRecursive(scene, scene->mRootNode, aiMatrix4x4());
	}

	DAELoader::~DAELoader() {
		materialMap.clear();
	}

	void DAELoader::getNodeBBox(
		const aiScene *scene,
		const aiNode* nd,
		aiVector3D* min,
		aiVector3D* max,
		aiMatrix4x4* trafo
		) {
		aiMatrix4x4 prev;
		unsigned int n = 0, t;

		prev = *trafo;
		aiMultiplyMatrix4(trafo, &nd->mTransformation);

		for (; n < nd->mNumMeshes; ++n) {
			const aiMesh* mesh = scene->mMeshes[nd->mMeshes[n]];
			for (t = 0; t < mesh->mNumVertices; ++t) {

				aiVector3D tmp = mesh->mVertices[t];
				aiTransformVecByMatrix4(&tmp, trafo);

				min->x = fminf(min->x, tmp.x);
				min->y = fminf(min->y, tmp.y);
				min->z = fminf(min->z, tmp.z);

				max->x = fmaxf(max->x, tmp.x);
				max->y = fmaxf(max->y, tmp.y);
				max->z = fmaxf(max->z, tmp.z);
			}
		}

		for (n = 0; n < nd->mNumChildren; ++n) {
			getNodeBBox(scene, nd->mChildren[n], min, max, trafo);
		}

		*trafo = prev;
	}

	bool DAELoader::importSceneFromFile(const std::string &pFile)
	{
		// Check if file exists
		std::ifstream fin(pFile.c_str());
		if (!fin.fail()) {
			fin.close();
		}
		else {
			const std::string desc = "Failed to open the file: " + pFile;
			return false;
		}

		// Pass a post-processing aiProcessPreset_TargetRealtime_Quality mask, to make sure everything is optimized for a typical rasterization engine
		scene = importer.ReadFile(pFile, aiProcessPreset_TargetRealtime_Quality);

		// If the import failed, report it
		if (!scene) {
			const std::string desc = importer.GetErrorString();
			return false;
		}

		// Get the scene's BBox
		auto getSceneBBox = [&](aiVector3D* min, aiVector3D* max) -> void
		{
			aiMatrix4x4 trafo;
			aiIdentityMatrix4(&trafo);

			min->x = min->y = min->z = 1e10f;
			max->x = max->y = max->z = -1e10f;
			getNodeBBox(scene, scene->mRootNode, min, max, &trafo);
		};

		aiVector3D sceneMin, sceneMax, sceneCenter;
		getSceneBBox(&sceneMin, &sceneMax);
		sceneCenter.x = (sceneMin.x + sceneMax.x) * .5f;
		sceneCenter.y = (sceneMin.y + sceneMax.y) * .5f;
		sceneCenter.z = (sceneMin.z + sceneMax.z) * .5f;

		this->sceneMin = Vector3f(sceneMin.x, sceneMin.y, sceneMin.z);
		this->sceneMax = Vector3f(sceneMax.x, sceneMax.y, sceneMax.z);
		this->sceneCenter = Vector3f(sceneCenter.x, sceneCenter.y, sceneCenter.z);

		// Update the light's position to put in the center of the "ceiling"
		//LightPosition = Vector3f(0.f, sceneMax.y, 0.f, 1.f);

		// We're done. Everything will be cleaned up by the importer destructor
		return true;
	}

	bool DAELoader::initSceneMaterials(const aiScene *scene) {
		if (scene->HasTextures()) {
			return false;
		}

		std::string materialType = "Matte";

		// Get texture Filenames and the number of textures
		for (unsigned int i = 0; i < scene->mNumMaterials; ++i) {

			const aiMaterial *aiMtl = scene->mMaterials[i];

			int texIndex = 0;

			// Diffuse texture map present
			std::string textureFilePath = "";
			if (1) {
				aiString path;	// filename
				if (AI_SUCCESS == aiMtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &path)) {
					textureFilePath = basePath + path.C_Str();
					++texIndex;

					//materialType = "MatteTextured";
					materialType = "Uber";
				}
			}
			
			// Diffuse color
			aiColor4D diffuseColor(.5f, .0f, .0f, 1.f);
			//aiColor4D diffuseColor((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX, 1.f);
			if (textureFilePath == "") {
				if (AI_SUCCESS == aiGetMaterialColor(aiMtl, AI_MATKEY_COLOR_DIFFUSE, &diffuseColor)) {
					//materialType = "Matte";
					materialType = "Uber";
				}
			}

			
			// Specular color
			aiColor4D specularColor(.0f, .0f, .0f, 1.f);
			if (1) {
				if (AI_SUCCESS == aiGetMaterialColor(aiMtl, AI_MATKEY_COLOR_SPECULAR, &specularColor)) {
					//materialType = "Plastic";
				}
			}

			//// Ambient color
			//{
			//	if (AI_SUCCESS == aiGetMaterialColor(srcMtl, AI_MATKEY_COLOR_AMBIENT, &c)) {
			//		material.ambient = glm::vec4(c.r, c.g, c.b, c.a);
			//	}
			//}

			//// Emission color
			//{
			//	if (AI_SUCCESS == aiGetMaterialColor(srcMtl, AI_MATKEY_COLOR_EMISSIVE, &c)) {
			//		material.emission = glm::vec4(c.r, c.g, c.b, c.a);
			//	}
			//}

			//// Transparency (this is different from transmission below, which is aka "transparent" in COLLADA standard, and is recalculated based on the COLLADA v1.5 specs during parsing)
			//{
			//	unsigned int max = 1;
			//	if (AI_SUCCESS == aiGetMaterialFloatArray(srcMtl, AI_MATKEY_OPACITY, &material.transparency, &max)) {

			//	}
			//}

			// Transmission color
			aiColor4D transmissionColor(.0f, .0f, .0f, 1.f);
			if (1) {
				if (AI_SUCCESS == aiGetMaterialColor(aiMtl, AI_MATKEY_COLOR_TRANSPARENT, &transmissionColor)
					&& transmissionColor.a < 1.f) {
					materialType = "ThinDielectric";
				}
			}

			//// Shininess strength
			//{
			//	unsigned int max = 1;
			//	aiReturn ret1 = aiGetMaterialFloatArray(srcMtl, AI_MATKEY_SHININESS, &material.shininess, &max);
			//	max = 1;
			//	float strength = 0.f;
			//	aiReturn ret2 = aiGetMaterialFloatArray(srcMtl, AI_MATKEY_SHININESS_STRENGTH, &strength, &max);
			//	if (ret1 == AI_SUCCESS/* && ret2 == AI_SUCCESS*/) {
			//		material.shininess *= 1.f;// strength;
			//	}
			//	else {
			//		material.shininess = 0.f;
			//	}
			//}

			//// Wireframe flag
			//{
			//	unsigned int max = 1;
			//	int wf;
			//	if (AI_SUCCESS == aiGetMaterialIntegerArray(srcMtl, AI_MATKEY_ENABLE_WIREFRAME, &wf, &max)) {
			//		material.wireframe = wf != 0;
			//	}
			//}

			//// Two-sided flag
			//{
			//	unsigned int max = 1;
			//	int ts;
			//	if (AI_SUCCESS == aiGetMaterialIntegerArray(srcMtl, AI_MATKEY_TWOSIDED, &ts, &max)) {
			//		material.twosided = ts != 0;
			//	}
			//}

			bool cullBackFaces = true;
			bool render = true;
#if 0
			Handle<Device::RTMaterial> material = defaultMaterial;
#else
			// Create a new material with an appropriate type
			Handle<Device::RTMaterial> material = g_device->rtNewMaterial(materialType.c_str());

			if (materialType == "MatteTextured") {
				g_device->rtSetTexture(material, "Kd", rtLoadTexture(FileName(textureFilePath)));
			}
			else if (materialType == "Uber") {
				if (textureFilePath != "") {
					g_device->rtSetTexture(material, "Kd", rtLoadTexture(FileName(textureFilePath)));
				}
				else {
					g_device->rtSetFloat3(material, "diffuse", diffuseColor.r, diffuseColor.g, diffuseColor.b);
				}
				const auto specIntensity = (specularColor.r * .212671) +
					(specularColor.g * .715160) +
					(specularColor.b * .072169);
				g_device->rtSetFloat1(material, "roughness", 1.f - specIntensity);

				//cullBackFaces = false;
			}
			else if (materialType == "Matte") {
				g_device->rtSetFloat3(material, "reflectance", diffuseColor.r, diffuseColor.g, diffuseColor.b);
			}
			else if (materialType == "Plastic") {
				g_device->rtSetFloat3(material, "pigmentColor", diffuseColor.r, diffuseColor.g, diffuseColor.b);
				const auto specIntensity = (specularColor.r * .212671) +
					(specularColor.g * .715160) +
					(specularColor.b * .072169);
				g_device->rtSetFloat1(material, "roughness", 1.f - specIntensity);
			}
			else if (materialType == "Dielectric") {
				g_device->rtSetFloat3(material, "transmission", transmissionColor.r, transmissionColor.g, transmissionColor.b);
				g_device->rtSetFloat1(material, "etaOutside", 1.f);
				g_device->rtSetFloat1(material, "etaInside", 1.f);
				render = true;
			}
			else if (materialType == "ThinDielectric") {
				// ToDo: the handling of transmission needs further investigation!
				g_device->rtSetFloat3(material, "transmission", 1.f - transmissionColor.r, 1.f - transmissionColor.g, 1.f - transmissionColor.b);
				//g_device->rtSetFloat3(material, "transmission", transmissionColor.r, transmissionColor.g, transmissionColor.b);
				g_device->rtSetFloat1(material, "eta", 1.4f);
				g_device->rtSetFloat1(material, "thickness", 1.f);
				render = true;
			}

			g_device->rtCommit(material);
#endif

			materialMap[i] = MaterialInfo(material, cullBackFaces, render);
		}

		return true;
	}

	bool DAELoader::initSceneCameras(const aiScene *scene, bool toeIn) {
		// Go through the scene's cameras
		for (unsigned int i = 0; i < scene->mNumCameras; ++i) {

			// Only consider cameras with the predefined FPR_VIEW_CAMERA_PREFIX prefix
			std::string cameraName = scene->mCameras[i]->mName.C_Str();
			const std::string prefix = FPR_VIEW_CAMERA_PREFIX;
			const auto pos = cameraName.find(prefix);
			if (pos != 0) {
				continue;
			}

			// Drop the prefix part, as we don't need it anymore
			cameraName.erase(pos, prefix.length());

			aiMatrix4x4 m = scene->mRootNode->mTransformation * scene->mCameras[i]->mTransformation;

			// Get the constituent transformations
			{
				aiVector3D scale;
				aiQuaternion rotation;
				aiVector3D position;
				m.Decompose(scale, rotation, position);
				sceneScale = scale.x;
			}

			AffineSpace3f sceneTransform = one;
			{
				// extract translation
				Vector3f position(m[0][3], m[1][3], m[2][3]);

				// extract the rows of the matrix
				Vector3f vRows[3] = {
					Vector3f(m[0][0], m[1][0], m[2][0]),
					Vector3f(m[0][1], m[1][1], m[2][1]),
					Vector3f(m[0][2], m[1][2], m[2][2])
				};

				sceneTransform = AffineSpace3f(vRows[0], vRows[1], vRows[2], position);
			}

			// Create 12 stereoscopic cube map cameras for each FPR viewpoint
			{
				const auto position = Vector3f(scene->mCameras[i]->mPosition.x, scene->mCameras[i]->mPosition.y, scene->mCameras[i]->mPosition.z);
				const auto lookAt = Vector3f(scene->mCameras[i]->mLookAt.x, scene->mCameras[i]->mLookAt.y, scene->mCameras[i]->mLookAt.z);
				const auto up = Vector3f(scene->mCameras[i]->mUp.z, scene->mCameras[i]->mUp.y, scene->mCameras[i]->mUp.z);

				// ToDo: the near & far planes' values should be calculated on a per camera basis depending on the position and orientation of the latter.
				//const auto nearPlane = scene->mCameras[0]->mClipPlaneNear;
				//const auto farPlane = scene->mCameras[0]->mClipPlaneFar;

				const auto camPos = xfmPoint(sceneTransform, position);
				const auto camLookAt = xfmPoint(sceneTransform, lookAt);
				const auto camUp = xfmVector(sceneTransform, up);

				const auto camSpaceOrigin = AffineSpace3f::lookAtPoint(camPos, camLookAt, camUp);

				for (int i = 0; i < 12; ++i) {
					Handle<Device::RTCamera> stereoCubeCamera = g_device->rtNewCamera("stereo");
					g_device->rtSetTransform(stereoCubeCamera, "local2world", copyToArray(camSpaceOrigin));
					g_device->rtSetInt1(stereoCubeCamera, "cubeFaceIndex", i);
					g_device->rtSetFloat3(stereoCubeCamera, "origin", camPos.x, camPos.y, camPos.z);
					g_device->rtSetFloat3(stereoCubeCamera, "lookAt", camLookAt.x, camLookAt.y, camLookAt.z);
					g_device->rtSetFloat3(stereoCubeCamera, "up", camUp.x, camUp.y, camUp.z);
					g_device->rtSetBool1(stereoCubeCamera, "toeIn", toeIn);
					g_device->rtSetFloat1(stereoCubeCamera, "sceneScale", sceneScale);
					const auto eyeSeparation = 6.35f * 0.393701f;
					g_device->rtSetFloat1(stereoCubeCamera, "eyeSeparation", eyeSeparation);
					const auto zeroParallaxDistance = eyeSeparation * 30.f;
					g_device->rtSetFloat1(stereoCubeCamera, "zeroParallaxDistance", zeroParallaxDistance);
					g_device->rtSetString(stereoCubeCamera, "name", cameraName.c_str());
					g_device->rtCommit(stereoCubeCamera);

					cameras.push_back(stereoCubeCamera);
				}
			}
		}

		if (!cameras.size()) {
			const std::string desc = "No FPR cameras found.";
			return false;
		}
		else {
			const std::string msg = "Loaded " + std::to_string(cameras.size()) + " FPR cameras.";
		}

		return true;
	}

	void DAELoader::initSceneMeshesRecursive(const struct aiScene *aiScene, const struct aiNode* node, const aiMatrix4x4 &mParent) {
		auto mNode = mParent * node->mTransformation;

		AffineSpace3f modelSpace = one;
		{
			// extract translation
			Vector3f position(mNode[0][3], mNode[1][3], mNode[2][3]);

			// extract the rows of the matrix
			Vector3f vRows[3] = {
				Vector3f(mNode[0][0], mNode[1][0], mNode[2][0]),
				Vector3f(mNode[0][1], mNode[1][1], mNode[2][1]),
				Vector3f(mNode[0][2], mNode[1][2], mNode[2][2])
			};

			modelSpace = AffineSpace3f(vRows[0], vRows[1], vRows[2], position);
		}

		// Get the constituent transformations
		//aiVector3D scale;
		//aiQuaternion rotation;
		//aiVector3D position;
		//mNode.Decompose(scale, rotation, position);

		//if (scale.x < .5) {
		//	scale *= 50.f;
		//}

		//auto t = AffineSpace3f::translate(Vector3f(position.x, position.y, position.z));
		//auto s = AffineSpace3f::scale(Vector3f(scale.x, scale.y, scale.z));
		//auto r = OrthonormalSpace3f(Quaternion3f(rotation.w, rotation.x, rotation.y, rotation.z), zero);

		for (unsigned int n = 0; n < node->mNumMeshes; ++n) {

			const struct aiMesh* aimesh = aiScene->mMeshes[node->mMeshes[n]];

			if (!materialMap[aimesh->mMaterialIndex].render)
				continue;

			// Only allow through the submeshes with the last material assigned 
			//if (aimesh->mSubMeshSize > 1 && aimesh->mSubMeshIndex != aimesh->mSubMeshSize - 1) {
			//	continue;
			//}

			// For testing
			if (0) {
				if (std::string(aimesh->mID.C_Str()) != "ID893") {
					continue;
				}
				else if (0) {
					aiString texPath;
					int texIndex = 0;
					auto srcMtl = scene->mMaterials[aimesh->mMaterialIndex];
					if (AI_SUCCESS == srcMtl->GetTexture(aiTextureType_DIFFUSE, texIndex, &texPath)) {
						std::string file = texPath.C_Str();
						//const std::string lookFor("carpet2.jpg");
						const std::string lookFor("blue_carpet.jpg");
						
						if (file.find(lookFor) == std::string::npos) {
							continue;
						}
					}
					else {
						continue;
					}
				}
			}

			// For now, our renderer supports only triangulated meshes (this might change in the future)
			if (aimesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE) {
				continue;
			}

			// Only allow through the submeshes with the last material assigned 
			//if (mesh->mSubMeshSize > 1 && mesh->mSubMeshIndex != mesh->mSubMeshSize - 1) {
			//	continue;
			//}

			if (!aimesh->mNormals) {
				continue;
			}

			size_t numPositions = aimesh->mNumVertices;
			Handle<Device::RTData> dataPositions = g_device->rtNewData("immutable", numPositions * sizeof(Vec3f), (numPositions ? aimesh->mVertices : nullptr));

			size_t numNormals = aimesh->mNormals ? aimesh->mNumVertices : 0;
			Handle<Device::RTData> dataNormals = g_device->rtNewData("immutable", numNormals * sizeof(Vec3f), (numNormals ? aimesh->mNormals : nullptr));

			size_t numTexCoords = &aimesh->mTextureCoords[0][0] ? aimesh->mNumVertices : 0;
			Handle<Device::RTData> dataTexCoords = g_device->rtNewData("immutable", numTexCoords * sizeof(Vec3f), (numTexCoords ? &aimesh->mTextureCoords[0][0] : nullptr));

			std::vector<Vec3i> indices;
			{
				for (unsigned int t = 0; t < aimesh->mNumFaces; ++t)
				{
					const aiFace *face = &aimesh->mFaces[t];

					if (face->mNumIndices != 3) {
						// Should never happen
						throw std::runtime_error("The face is NOT a triangle!");
					}

					indices.push_back(Vec3i(face->mIndices[0], face->mIndices[1], face->mIndices[2]));
				}
			}

			size_t numTriangles = indices.size();
			Handle<Device::RTData> dataIndices = g_device->rtNewData("immutable", numTriangles * sizeof(Vec3i), (numTriangles ? &indices[0] : nullptr));

			const auto material = materialMap[aimesh->mMaterialIndex].material;
			const auto cullBackFaces = materialMap[aimesh->mMaterialIndex].cullBackFaces;

			Handle<Device::RTShape> mesh = g_device->rtNewShape("trianglemesh");
			if (numPositions) g_device->rtSetArray(mesh, "positions", "float3", dataPositions, numPositions, sizeof(Vec3f), 0);
			if (numNormals) g_device->rtSetArray(mesh, "normals", "float3", dataNormals, numNormals, sizeof(Vec3f), 0);
			if (numTexCoords) g_device->rtSetArray(mesh, "texcoords", "float2", dataTexCoords, numTexCoords, sizeof(Vec3f), 0);
			if (numTriangles) g_device->rtSetArray(mesh, "indices", "int3", dataIndices, numTriangles, sizeof(Vec3i), 0);
			g_device->rtSetBool1(mesh, "cullBackFaces", cullBackFaces);
			g_device->rtSetString(mesh, "accel", g_mesh_accel.c_str());
			g_device->rtSetString(mesh, "builder", g_mesh_builder.c_str());
			g_device->rtSetString(mesh, "traverser", g_mesh_traverser.c_str());
			g_device->rtCommit(mesh);
			g_device->rtClear(mesh);

			// See if the mesh is auto-camera aligned
			const std::string meshName = aimesh->mName.C_Str();
			const std::string prefix = CAMERA_ALIGNED_NODE_PREFIX;
			const bool faceCamera = meshName.compare(0, prefix.length(), prefix) == 0;

			Handle<Device::RTPrimitive> prim = g_device->rtNewShapePrimitive(mesh, material, copyToArray(modelSpace), faceCamera);
			primitives.push_back(prim);
		}

		for (unsigned int n = 0; n < node->mNumChildren; ++n) {
			initSceneMeshesRecursive(aiScene, node->mChildren[n], mNode);
		}
	}

	std::vector<Handle<Device::RTPrimitive>> loadDAE(const FileName &fileName, std::vector<Handle<Device::RTCamera>> &cameras) {
		DAELoader loader(fileName);
		cameras = loader.cameras;
		return loader.primitives;
	}
}
