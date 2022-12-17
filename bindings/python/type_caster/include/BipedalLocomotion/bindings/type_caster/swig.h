/**
 * @file swig.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_BINDINGS_TYPE_CASTER_SWIG_H
#define BIPEDAL_LOCOMOTION_BINDINGS_TYPE_CASTER_SWIG_H

#include <type_traits>

#include <pybind11/detail/descr.h>
#include <pybind11/pybind11.h>

namespace pybind11::detail
{

struct PySwigObject
{
    PyObject_HEAD void* ptr;
    const char* desc;
};

inline void* extract_swig_wrapped_pointer(PyObject* obj)
{
    char thisStr[] = "this";

    // first we need to get the this attribute from the Python Object
    if (!PyObject_HasAttrString(obj, thisStr))
        return NULL;

    PyObject* thisAttr = PyObject_GetAttrString(obj, thisStr);
    if (thisAttr == NULL)
        return NULL;

    // This Python Object is a SWIG Wrapper and contains our pointer
    void* pointer = ((PySwigObject*)thisAttr)->ptr;
    const char* desc = ((PySwigObject*)thisAttr)->desc;

    // TODO this not needed? Please check
    Py_DECREF(thisAttr);
    return pointer;
};

template <class T> inline T* swig_wrapped_pointer_to_pybind(pybind11::object& obj)
{
    void* ptr = pybind11::detail::extract_swig_wrapped_pointer(obj.ptr());
    if (ptr == nullptr)
    {
        return nullptr;
    }

    T* cls = reinterpret_cast<T*>(ptr);
    return cls;
}

} // namespace pybind11::detail

#endif // BIPEDAL_LOCOMOTION_BINDINGS_TYPE_CASTER_SWIG_H
