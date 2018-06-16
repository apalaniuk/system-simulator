#pragma once

#include "stdafx.h"
#include "SimEntity.h++"

class SimEntityPython final
	: public SimEntity
	, public boost::python::wrapper<SimEntity>
{
	void updatePhysics() override;
};

void SimEntityPython::updatePhysics()
{
	// call through to the python class's `updatePhysics` method
	get_override("updatePhysics")();
}


BOOST_PYTHON_MODULE(SimEntity)
{
	boost::python::class_<SimEntityPython, boost::noncopyable>("SimEntity")
		.def("updatePhysics", &SimEntity::updatePhysics)
		;
}
