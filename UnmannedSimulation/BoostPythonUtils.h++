#pragma once

#include "stdafx.h"

namespace sim {
	namespace python {
		namespace utils {
			boost::python::object import(const std::string& module, const std::string& path, boost::python::object& globals)
			{
				boost::python::dict locals;

				locals["module_name"] = module;
				locals["path"] = path;

				boost::python::exec(
					"import imp\n"
					"new_module = imp.load_module(module_name, open(path), path, ('py', 'U', imp.PY_SOURCE))\n",
					globals,
					locals
				);

				return locals["new_module"];
			}
		}
	}
}