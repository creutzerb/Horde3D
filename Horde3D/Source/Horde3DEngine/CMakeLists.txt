
# CMAKE_BINARY_DIR is used for inclusion of automatically generated files
include_directories(. ../Shared ../../Bindings/C++ ../../../ ${CMAKE_BINARY_DIR} ${HORDE3D_EXTENSION_INCLUDE_DIRS})


set(HORDE3D_SOURCES 
	egAnimatables.cpp
	egAnimation.cpp
	egCamera.cpp
	egCom.cpp
	egComputeNode.cpp
	egComputeBuffer.cpp
	egExtensions.cpp
	egGeometry.cpp
	egLight.cpp
	egMain.cpp
	egMaterial.cpp
	egModel.cpp
	egModules.cpp
	egParticle.cpp
	egPipeline.cpp
	egPrimitives.cpp
	egRenderer.cpp
	egResource.cpp
	egScene.cpp
	egSceneGraphRes.cpp
	egShader.cpp
	egTexture.cpp
	utImage.cpp
#	config.h
	egAnimatables.h
	egAnimation.h
	egCamera.h
	egCom.h
	egComputeNode.h
	egComputeBuffer.h
	egExtensions.h
	egGeometry.h
	egLight.h
	egMaterial.h
	egModel.h
	egModules.h
	egParticle.h
	egPipeline.h
	egPrerequisites.h
	egPrimitives.h
	egRenderer.h
	egRendererBase.h
	egResource.h
	egScene.h
	egSceneGraphRes.h
	egShader.h
	egTexture.h
	utImage.h
	utTimer.h
    ../Shared/utPlatform.h
	../../Bindings/C++/Horde3D.h
	${HORDE3D_EXTENSION_SOURCES}
	)

add_definitions(-DCMAKE )

#add_definitions(-DH3D_USE_GL4)
set( H3D_USE_GL4 1 )

list(APPEND HORDE3D_SOURCES "egRendererBaseGL4.h")
list(APPEND HORDE3D_SOURCES "egRendererBaseGL4.cpp")

list(APPEND HORDE3D_SOURCES "utOpenGL.h")
list(APPEND HORDE3D_SOURCES "utOpenGL.cpp")



# Add renderers, specified during build, in the config.h file
configure_file( config.h.in ${CMAKE_BINARY_DIR}/config.h )

# For all platforms except iOS, engine is built as shared library
if (${CMAKE_SYSTEM_NAME} MATCHES "iOS")
	add_library(Horde3D STATIC ${HORDE3D_SOURCES} )
else(${CMAKE_SYSTEM_NAME} MATCHES "iOS")
# Other platforms
	add_library(Horde3D SHARED
		${HORDE3D_SOURCES}
		)
endif(${CMAKE_SYSTEM_NAME} MATCHES "iOS")

option(RAPIDXML_NO_EXCEPTIONS "Disabling rapidxml exceptions will terminating application on xml parsing error" ON)
if (RAPIDXML_NO_EXCEPTIONS)
	add_definitions(-DRAPIDXML_NO_EXCEPTIONS)
endif(RAPIDXML_NO_EXCEPTIONS)


if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
	IF(MSVC)
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DSECURE_SCL=0")		
	ENDIF(MSVC)
	
	if (USE_GLES3)
		FIND_PACKAGE(OpenGLESv2 REQUIRED)
		# add additional include paths for egl and gles
		include_directories(${EGL_INCLUDE_DIR} ${OPENGLES2_INCLUDE_DIR})
	
		target_link_libraries(Horde3D ${EGL_LIBRARIES} ${OPENGLES2_LIBRARIES} ${HORDE3D_EXTENSION_LIBS})
	else (USE_GLES3)
		FIND_PACKAGE(OpenGL REQUIRED)
		target_link_libraries(Horde3D ${OPENGL_gl_LIBRARY} ${HORDE3D_EXTENSION_LIBS})
	endif (USE_GLES3)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    if (USE_GLES3)
		FIND_PACKAGE(OpenGLESv2 REQUIRED)
		# add additional include paths for egl and gles
		include_directories(${EGL_INCLUDE_DIR} ${OPENGLES2_INCLUDE_DIR})
	
		target_link_libraries(Horde3D ${EGL_LIBRARIES} ${OPENGLES2_LIBRARIES} ${HORDE3D_EXTENSION_LIBS})
	else (USE_GLES3)
		FIND_PACKAGE(OpenGL REQUIRED)
		if( EGL_FOUND )
            add_definitions( -DHAVE_EGL ${EGL_DEFINITIONS} )
        endif( EGL_FOUND )

		# cmake now prefers GLVND libraries instead of raw libGL. So search for OPENGL_opengl_LIBRARY.
		# currently we are still targeting X, so we should include glx library
        target_link_libraries(Horde3D ${OPENGL_opengl_LIBRARY} ${OPENGL_glx_LIBRARY} ${EGL_LIBRARIES} ${HORDE3D_EXTENSION_LIBS})
	endif (USE_GLES3)        	

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
		
	install(TARGETS Horde3D
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib
		)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
