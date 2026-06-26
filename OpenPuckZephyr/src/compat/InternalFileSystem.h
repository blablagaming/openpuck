// InternalFileSystem.h -- compat shim: the global InternalFS object.
//
// In the Arduino tree this is the Adafruit InternalFileSystem singleton over the
// nRF52 internal flash. Here it is a LittleFS instance mounted on the board's
// storage partition. Code that does `File f(InternalFS); f.open(...)` ports
// unchanged.
#pragma once
#include "Adafruit_LittleFS.h"

extern Adafruit_LittleFS_Namespace::LittleFS InternalFS;
