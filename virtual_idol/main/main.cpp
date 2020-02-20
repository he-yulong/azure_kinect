// main.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <iostream>
#include "SingleKinect.h"


PYBIND11_MODULE(main, m) {
    // shorthand
    using namespace pybind11::literals;
    m.doc() = "pybind11 example plugin";

    // Creating bindings for a custom type
    py::class_<ws_tech::SingleKinect>(m, "SingleKinect")
        .def(py::init<py::function, int>())
        .def("Open", &ws_tech::SingleKinect::Open)
        .def("Running", &ws_tech::SingleKinect::Running)
        .def("Close", &ws_tech::SingleKinect::Close);
}

//PYBIND11_MODULE(main, m) {
//    // shorthand
//    using namespace pybind11::literals;
//    m.doc() = "pybind11 example plugin"; // optional module docstring
//    m.def("add", &add, "A function which adds two numbers", py::arg("i") = 1, py::arg("j") = 2);
//    m.def("add", &add, "A function which adds two numbers", "i"_a = 1, "j"_a = 3);
//
//    // Exporting variables
//    m.attr("the_answer") = 42;
//    py::object world = py::cast("World");
//    m.attr("what") = world;
//}

//int add(int i, int j) {
//    return i + j;
//}
//
//struct Pet {
//    Pet(const std::string& name) : name(name) { }
//    void setName(const std::string& name_) { name = name_; }
//    const std::string& getName() const { return name; }
//
//    std::string name;
//};
//
//PYBIND11_MODULE(main, m) {
//    // shorthand
//    using namespace pybind11::literals;
//    m.doc() = "pybind11 example plugin"; // optional module docstring
//    //m.def("add", &add, "A function which adds two numbers", py::arg("i") = 1, py::arg("j") = 2);
//    m.def("add", &add, "A function which adds two numbers", "i"_a = 1, "j"_a = 3);
//
//    // Exporting variables
//    m.attr("the_answer") = 42;
//    py::object world = py::cast("World");
//    m.attr("what") = world;
//
//    // Creating bindings for a custom type
//    py::class_<Pet>(m, "Pet")
//        .def(py::init<const std::string&>())
//        .def("setName", &Pet::setName)
//        .def("getName", &Pet::getName);
//}
