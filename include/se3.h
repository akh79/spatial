#pragma once

#include <cmath>
#include <iosfwd>
#include <iomanip>
#include <Eigen/Core>

template <typename T> using vec3 = Eigen::Matrix<T, 3, 1>;
template <typename T> using mat3 = Eigen::Matrix<T, 3, 3>;
template <typename T> using mat4 = Eigen::Matrix<T, 4, 4>;
template <typename T> using mat3x = Eigen::Matrix<T, 3, Eigen::Dynamic>;

namespace Spatial
{
	template <typename T>
	constexpr T radian(T a)
	{
		return (a * (T)3.1415926535897932) / (T)180;
	}

	// Implements the semidirect product SE(3) = SO(3) * R^3

	template <typename T>
	class SE3
	{
	protected:
		mat3<T> rot_;
		vec3<T> trans_;

		SE3(const mat3<T>& r, const vec3<T>& t)
			: rot_(r)
			, trans_(t)
		{}

		SE3(const mat3<T>& r)
			: rot_(r)
			, trans_(vec3<T>::Zero())
		{}

		SE3(const vec3<T>& t)
			: rot_(mat3<T>::Identity())
			, trans_(t)
		{}

	public:
		SE3()
			: rot_(mat3<T>::Identity())
			, trans_(vec3<T>::Zero())
		{}

		SE3(const SE3<T>& other)
			: rot_(other.rot_)
			, trans_(other.trans_)
		{}

		SE3<T>& operator=(const SE3<T>& other)
		{
			rot_ = other.rot_;
			trans_ = other.trans_;
			return *this;
		}

		mat3<T> GetRotation() const
		{
			return rot_;
		}

		vec3<T> GetTranslation() const
		{
			return trans_;
		}

		mat4<T> GetMatrix() const
		{
			mat4<T> m;

			m << rot_(0, 0), rot_(0, 1), rot_(0, 2), trans_(0),
				rot_(1, 0), rot_(1, 1), rot_(1, 2), trans_(1),
				rot_(2, 0), rot_(2, 1), rot_(2, 2), trans_(2),
				(T)0, (T)0, (T)0, (T)1;

			return m;
		}

		static SE3<T> Rotation(vec3<T> u, const T phi)
		{
			// Create rotation transfrom about 
			// the vector 'u' by angle phi [°].
			// Note that 'u' must have unit norm.
			// The code is after the formula from 
			// "Computational geometry for design and manufacture"
			// by I.D.Faux and M.J.Pratt

			T s, c;

			// Note: exact equality test on purpose

			if (phi == (T)0)
			{
				s = (T)0;
				c = (T)1;
			}
			else if (phi == (T)90)
			{
				s = (T)1;
				c = (T)0;
			}
			else if (phi == (T)180)
			{
				s = (T)0;
				c = (T)(-1);
			}
			else if (phi == (T)270)
			{
				s = (T)(-1);
				c = (T)0;
			}
			else
			{
				s = sin(radian<T>(phi));
				c = cos(radian<T>(phi));
			}

			mat3<T> uut = u * u.transpose();
			mat3<T> U;
			U << (T)0, -u(2), u(1),
				u(2), (T)0, -u(0),
				-u(1), u(0), (T)0;

			mat3<T> r;
			r = uut + c * (mat3<T>::Identity() - uut) + s * U;

			return SE3<T>(r);
		}

		static SE3<T> Translation(vec3<T> axis, const T dist = (T)1)
		{
			// Create translation transform along 'axis' of the length dist
			vec3<T> t = dist * axis;
			return SE3<T>(t);
		}

		static SE3<T> Invert(const SE3<T>& t)
		{
			// For orthogonal matrices R^[-1} == R^t == R
			// and transposition is actually not needed

			mat3<T> rot = t.rot_.transpose();
			vec3<T> trans = -(rot * t.trans_);
			return SE3<T>(rot, trans);
		}

		SE3<T> operator*(const SE3<T>& other)
		{
			// Returns this * other;
			vec3<T> trans2 = rot_ * other.trans_ + trans_;
			mat3<T> rot2 = rot_ * other.rot_;
			return SE3<T>(rot2, trans2);
		}

		SE3<T> operator*=(const SE3<T>& other)
		{
			// Implements this = this * other;
			// Multiple calls result in a sequence this * other_1 * other_2 * other_3...
			trans_ = rot_ * other.trans_ + trans_;
			rot_ = rot_ * other.rot_;
			return *this;
		}

		mat3x<T> operator*(const mat3x<T>& cols)
		{
			mat3x<T> res;
			res.resize(3, cols.cols());

			for (Eigen::Index idx = 0; idx < cols.cols(); ++idx)
			{
				const vec3<T>& col = cols.col(idx);
				res.col(idx) = (*this) * col;
			}

			return res;
		}

		mat3x<T> operator^(const mat3x<T>& cols)
		{
			mat3x<T> res;
			res.resize(3, cols.cols());

			for (Eigen::Index idx = 0; idx < cols.cols(); ++idx)
			{
				const vec3<T>& col = cols.col(idx);
				res.col(idx) = (*this) ^ col;
			}

			return res;
		}

		vec3<T> operator*(const vec3<T>& v)
		{
			vec3<T> u = rot_ * v;
			return u;
		}

		vec3<T> operator^(const vec3<T>& p)
		{
			vec3<T> q = rot_ * p + trans_;
			return q;
		}

		template <typename T>
		friend std::ostream& operator<<(std::ostream& stream, const SE3<T>& t);
	};

	template <typename T>
	std::ostream& operator<<(std::ostream& stream, const SE3<T>& t)
	{
		stream << std::fixed << std::setprecision(5);
		stream << "| " << std::setw(10) << t.rot_(0, 0) << "  " << std::setw(10) << t.rot_(0, 1) << "  " << std::setw(10) << t.rot_(0, 2) << " |" << std::setw(10) << t.trans_[0] << '\n';
		stream << "| " << std::setw(10) << t.rot_(1, 0) << "  " << std::setw(10) << t.rot_(1, 1) << "  " << std::setw(10) << t.rot_(1, 2) << " |" << std::setw(10) << t.trans_[1] << '\n';
		stream << "| " << std::setw(10) << t.rot_(2, 0) << "  " << std::setw(10) << t.rot_(2, 1) << "  " << std::setw(10) << t.rot_(2, 2) << " |" << std::setw(10) << t.trans_[2] << '\n';
		return stream;
	}
}
