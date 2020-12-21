# Copyright: Universidad Carlos III de Madrid (C) 2020;
# Authors: Juan G. Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Define a project.
project(TEO_BULLET_MODELS LANGUAGES NONE)

# Find dependencies.
find_package(YCM 0.11 REQUIRED)
find_package(YARP 3.2 REQUIRED)

# Set some useful variables - paths relative to the installation prefix.
yarp_configure_external_installation(teo-bullet-models)

# Configure install rule, copy models to the build directory.
yarp_install(DIRECTORY bullet
             DESTINATION ${TEO-BULLET-MODELS_DATA_INSTALL_DIR})

# Store the package in the user registry.
set(CMAKE_EXPORT_PACKAGE_REGISTRY ON)

# Create and install config files.
include(InstallBasicPackageFiles)

install_basic_package_files(TEO_BULLET_MODELS
                            VERSION 0.1.0
                            COMPATIBILITY AnyNewerVersion
                            NO_EXPORT)

# Add uninstall rule.
include(AddUninstallTarget)
