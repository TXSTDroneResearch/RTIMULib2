////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech
//  Copyright (c) 2014, avishorp
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// Python binding for RTIMULib

#include <Python.h>
#include "structmember.h"
#include "RTIMULib.h"

// RTIMUSettings Type
struct RTIMU_Settings {
    PyObject_HEAD
    RTIMUSettings* val;
};

// RTIMU Type
struct RTIMU_RTIMU {
  PyObject_HEAD
  RTIMU* val;
  RTIMU_Settings* settings;
};

// Create the RTIMU_Settings type
int RTIMU_Settings_create(PyObject* module);

// Check if the given object is of RTIMU_Settings type
bool RTIMU_Settings_typecheck(PyObject* obj);

// Create the RTIMU type
int RTIMU_RTIMU_create(PyObject* module);

