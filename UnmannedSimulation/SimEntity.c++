#include "stdafx.h"
#include "SimEntity.h++"
#include "BoostPythonUtils.h++"

#include <map>

std::map<std::string, boost::python::object> SimEntity::moduleMap;

SimEntity::SimEntity(const std::string& type, const std::string& name, const std::string& filePath) : m_type(type), m_name(name) {
	boost::python::object main = boost::python::import("__main__");
	boost::python::object globals = main.attr("__dict__");

	auto typeModule = SimEntity::moduleMap.find(type);

	if (typeModule == SimEntity::moduleMap.end()) {
		SimEntity::moduleMap[type] = sim::python::utils::import(type, filePath, globals);
	}

	m_simEntity = SimEntity::moduleMap[type].attr("SimEntity")();
}

SimEntity::~SimEntity() {
}

void SimEntity::updatePhysics() {
	try {
		m_simEntity.attr("updatePhysics")();
	} catch (const boost::python::error_already_set&) {
		std::cerr << ">>> Error! Uncaught exception:\n";
		PyErr_Print();
	}
}
