#pragma once

#include "../constraints/Constraint.h"
#include "../objects/Particle.h"

class ConstraintSolver {
  public:
	static void solve(std::vector<Particle *> particles,
					  std::vector<Constraint *> constraints,
					  float Ks,
					  float Kd);
	static std::vector<std::vector<float>> multiply_matrix_by_matrix(std::vector<std::vector<float>> v1,
																	 std::vector<std::vector<float>> v2);
	static std::vector<float> multiply_matrix_by_vector(std::vector<std::vector<float>> v1,
														std::vector<float> v2);
	static std::vector<float> multiply_vector_by_scalar(std::vector<float> v, float s);
	static std::vector<std::vector<float>> multiply_matrix_by_scalar(std::vector<std::vector<float>> v,
																	 float s);
	static std::vector<float> subtract_vector_from_vector(std::vector<float> v1,
														  std::vector<float> v2);
	static std::vector<std::vector<float>> subtract_matrix_from_matrix(std::vector<std::vector<
		float>> v1, std::vector<std::vector<
		float>> v2);
	static std::vector<std::vector<float>> add_matrix_to_matrix(std::vector<std::vector<
		float>> v1, std::vector<std::vector<
		float>> v2);

};