#pragma once

#include <boost/units/systems/si.hpp>
#include <boost/units/systems/angle/revolutions.hpp>
#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

typedef boost::units::quantity<boost::units::si::length> length_units_t;
typedef boost::units::quantity<boost::units::si::plane_angle> angle_units_t;

namespace sim {
	namespace geometry {
		template <typename A, typename B>
		class point3  {
			public:
				point3(A x, A y, B z);
				virtual ~point3();

			private:
				A _x;
				A _y;
				B _z;
		};

		namespace cs {
			struct cartesian { };
			struct geocentric { };
			struct geodetic { };

			template <typename CS>
			class point3 : sim::geometry::points3 {	};

			template <sim::geometry::cs::cartesian>
			class point3 {
			public:
				length_units_t getX() { return _point3._x; };
				length_units_t getY() { return _point3._y; };
				length_units_t getZ() { return _point3._z; };

			protected:
				sim::geometry::point3<boost::units::si::length, boost::units::si::length> _point3;
			};

			template <sim::geometry::cs::geocentric>
			class point3 {
			public:
				angle_units_t  getLongitude() { return _point3._x; };
				angle_units_t  getLatitude()  { return _point3._y; };
				length_units_t getAltitude()  { return _point3._z; };

			protected:
				sim::geometry::point3<boost::units::si::plane_angle, boost::units::si::length> _point3;
			};

			template <sim::geometry::cs::geodetic>
			class point3 {
			public:
				angle_units_t  getLongitude() { return _point3._x; };
				angle_units_t  getLatitude()  { return _point3._y; };
				length_units_t getAltitude()  { return _point3._z; };

			protected:
				sim::geometry::point3<boost::units::si::plane_angle, boost::units::si::length> _point3;
			};
		}
	}
}