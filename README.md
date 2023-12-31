# Spatial

I'm planning to include here some libraries built on top of Eigen3 and needed for numerical simulation of the world of rigid bodies.

## Rigid transformations of Euclidean space R^3 (SE(3) group)

This is a first library implementing rigid motions in R^3. I was dissatisfied with the Geometry module of Eigen3 library and decided to implement my own beginning with the rigid transformations of R^3. The main feature is using overloaded operators to transform 3-vectors and 3-points using the same base type Eigen::Vector<T, 3> for both. No homogeneous coordinates or matrices are used.

## Implementation

The single header templated library (se3.h) uses the usual representation of a rigid transformation as a pair (R, T) where R is a rotation 3x3-matrix, T a 3-vector.

Two operators, operator* and operator^, are implemented to apply an SE(3) transformation to a 3-vector or 3-point. The similar overloaded operators allow transforming an Eigen::Matrix<T, 3, Eigen::Dynamic> as a row vector of 3-vectors or 3-points.

There is also a short test program based on doctest. The corresponding .vcxproj file is included.

## Dependencies

The only dependencies are:
 - Eigen3,
 - doctest.

I use vcpkg to get them installed.
