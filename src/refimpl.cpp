#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <array>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <vector>
#include <windows.h>

#include "bounding_box.hpp"
#include "bvh.hpp"
#include "ray.hpp"
#include "rayrun.hpp"
#include "vec.hpp"

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call,
                      LPVOID lpReserved) {
  switch (ul_reason_for_call) {
  case DLL_PROCESS_ATTACH:
  case DLL_THREAD_ATTACH:
  case DLL_THREAD_DETACH:
  case DLL_PROCESS_DETACH:
    break;
  }
  return TRUE;
}

using vector::Vec3;

/*
-------------------------------------------------
-------------------------------------------------
*/

static SimpleBVH g_bvh;

//
void preprocess(const float *vertices, size_t numVerts, const float *normals,
                size_t numNormals, const uint32_t *indices, size_t numFace) {

  std::vector<Vec3> verts;
  verts.reserve(numVerts);
  for (size_t vi = 0; vi < numVerts; ++vi) {
    verts.push_back(Vec3(
        {vertices[vi * 3 + 0], vertices[vi * 3 + 1], vertices[vi * 3 + 2]}));
  }

  std::vector<Vec3> ns;
  ns.reserve(numNormals);
  for (size_t ni = 0; ni < numNormals; ++ni) {
    ns.push_back(
        Vec3{normals[ni * 3 + 0], normals[ni * 3 + 1], normals[ni * 3 + 2]});
  }

  std::vector<Face> fs;
  fs.reserve(numFace);
  for (size_t fi = 0; fi < numFace; ++fi) {
    Face face;
    face.vertex_index = {indices[fi * 6 + 0], indices[fi * 6 + 2],
                         indices[fi * 6 + 4]};
    face.normal_index = {indices[fi * 6 + 1], indices[fi * 6 + 3],
                         indices[fi * 6 + 5]};
    fs.push_back(face);
  }

  MeshTriangleList mesh_list(std::move(fs), std::move(verts), std::move(ns));

  //
  g_bvh.construct(std::move(mesh_list));
}

void intersect(Ray *rays, size_t numRay, bool hitany) {
  static_cast<void>(hitany);
  //
  for (int32_t nr = 0; nr < numRay; ++nr) {
    Ray &ray = rays[nr];
    RayExt rayExt;
    rayExt.pos = ray.pos;
    rayExt.dir = ray.dir;
    const auto invSafe = [](float v) {
      return (v == 0.0f) ? std::numeric_limits<float>::infinity() : 1.0f / v;
    };
    rayExt.dinv = Vec3{invSafe(rayExt.dir.x()), invSafe(rayExt.dir.y()),
                       invSafe(rayExt.dir.z())};
    rayExt.sign[0] = (ray.dir[0] < 0.0f);
    rayExt.sign[1] = (ray.dir[1] < 0.0f);
    rayExt.sign[2] = (ray.dir[2] < 0.0f);
    rayExt.tnear = ray.tnear;
    rayExt.tfar = ray.tfar;
    //
    if (!g_bvh.intersect(rayExt)) {
      ray.isisect = false;
    } else {
      ray.isisect = true;
      const Vec3 &isectPos = rayExt.isect;
      ray.isect[0] = isectPos.x();
      ray.isect[1] = isectPos.y();
      ray.isect[2] = isectPos.z();
      ray.ns[0] = rayExt.ns.x();
      ray.ns[1] = rayExt.ns.y();
      ray.ns[2] = rayExt.ns.z();
      ray.faceid = rayExt.faceid;
    }
  }
}
