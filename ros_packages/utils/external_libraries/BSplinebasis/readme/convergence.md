# Convergence and numerical properties
## Case study: The quantum mechanical harmonic oscillator
This document analyses the numerical properties of the solution to the quantum mechanical harmonic oscillator via a BSpline basis-expansion. The Schrödinger equation takes the form `(-1/2 d^2/dx^2 + 1/2 x^2) f(x) = E f(x)` and can be solved analytically. The eigenenergies `E_n`, for example, are `E_n = n + 1/2`, where `n = 0,1,...`. 

The image shows the relative deviations between the analytically known and the numerically calculated eigenenergies of the lowest three states `n = 0,1,2`, using different spline orders and different grids. The color encodes the data type:

* **red:** double precision
* **blue:** quadruple precision (using `boost::multiprecision::cpp_bin_float_quad`)

The horizontal lines mark the epsilons for the two data types, respectively. All calculations were performed on a quadratic grid stretching over `-15 <= x <= 15` and using `n` basis functions.

![Accuracy](accuracy.png?raw=true "Accuracy")

For the corresponding code, see `readme/accuracy/`.
