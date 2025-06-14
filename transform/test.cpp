#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest\doctest.h"
#include "include\se3.h"

// Doctest-based test code for Spatial::SE3<T> library.

using se3f = Spatial::SE3<float>;
using vec3f = vec3<float>;
using mat3f = mat3<float>;

TEST_CASE("Composition rotation/translation")
{
	float angle = 90.0f;
	float delta = 4.0f;

	SUBCASE("Rotation about X")
	{
		se3f T = se3f::Rotation(vec3f::UnitX(), angle);
		T *= se3f::Translation(vec3f::UnitX(), delta);
		vec3f sx = T.GetRotation() * (vec3f::UnitX() * delta);
		CHECK(sx == T.GetTranslation());

		T = se3f::Rotation(vec3f::UnitX(), angle);
		T *= se3f::Translation(vec3f::UnitY(), delta);
		vec3f sy = T.GetRotation() * (vec3f::UnitY() * delta);
		CHECK(sy == T.GetTranslation());

		T = se3f::Rotation(vec3f::UnitX(), angle);
		T *= se3f::Translation(vec3f::UnitZ(), delta);
		vec3f sz = T.GetRotation() * (vec3f::UnitZ() * delta);
		CHECK(sz == T.GetTranslation());
	}

	SUBCASE("Rotation about Y")
	{
		se3f T = se3f::Rotation(vec3f::UnitY(), angle);
		T *= se3f::Translation(vec3f::UnitX(), delta);
		vec3f sx = T.GetRotation() * (vec3f::UnitX() * delta);
		CHECK(sx == T.GetTranslation());

		T = se3f::Rotation(vec3f::UnitY(), angle);
		T *= se3f::Translation(vec3f::UnitY(), delta);
		vec3f sy = T.GetRotation() * (vec3f::UnitY() * delta);
		CHECK(sy == T.GetTranslation());

		T = se3f::Rotation(vec3f::UnitY(), angle);
		T *= se3f::Translation(vec3f::UnitZ(), delta);
		vec3f sz = T.GetRotation() * (vec3f::UnitZ() * delta);
		CHECK(sz == T.GetTranslation());
	}

	SUBCASE("Rotation about Z")
	{
		se3f T = se3f::Rotation(vec3f::UnitZ(), angle);
		T *= se3f::Translation(vec3f::UnitX(), delta);
		vec3f sx = T.GetRotation() * (vec3f::UnitX() * delta);
		CHECK(sx == T.GetTranslation());

		T = se3f::Rotation(vec3f::UnitZ(), angle);
		T *= se3f::Translation(vec3f::UnitY(), delta);
		vec3f sy = T.GetRotation() * (vec3f::UnitY() * delta);
		CHECK(sy == T.GetTranslation());

		T = se3f::Rotation(vec3f::UnitZ(), angle);
		T *= se3f::Translation(vec3f::UnitZ(), delta);
		vec3f sz = T.GetRotation() * (vec3f::UnitZ() * delta);
		CHECK(sz == T.GetTranslation());
	}
};
