-- solution 
solution "rayrun"
	location "generated"
	configurations { "Debug", "Release" }
	platforms {"x64"}
	--
	configuration "Debug"
		defines { "DEBUG" }
		symbols "On"
	--
	configuration "Release"
		defines { "NDEBUG", "NO_ASSERT" }
		optimize "On"

-- 
project "rayrun"
	kind "ConsoleApp"
	language "C++"
	characterset "MBCS"
	files {
		"src/main.cpp",
		"src/rayrun.hpp",
	}
	includedirs {
		"thirdparty/stb/",
		"thirdparty/tinyobjloader/",
		"thirdparty/picojson/",
	}
	cppdialect "C++17"
	dependson { "refimp" }

-- 
project "refimp"
	kind "SharedLib"
	language "C++"
	characterset "MBCS"
	files {
		"src/refimpl.cpp",
		"src/rayrun.hpp",
		"src/bounding_box.hpp",
		"src/bvh.hpp",
		"src/fixed_vector.hpp",
		"src/matrix.hpp",
		"src/ray.hpp",
		"src/rayrun.hpp",
		"src/refimpl.cpp",
		"src/task_queue.hpp",
		"src/triangle.hpp",
		"src/vec.hpp"
	}
	cppdialect "C++17"