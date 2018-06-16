#pragma once

#include "stdafx.h"
#include <map>

class SimEntity
{
public:
	SimEntity(const std::string& type, const std::string& name, const std::string& filePath);
	~SimEntity();

	virtual void updatePhysics();

	const std::string m_type;
	const std::string m_name;

protected:
	boost::python::object m_simEntity;
	static std::map<std::string, boost::python::object> moduleMap;
};
