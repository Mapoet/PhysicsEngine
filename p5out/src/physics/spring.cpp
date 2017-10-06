#include "math/math.hpp"
#include "math/vector.hpp"
#include "math/quaternion.hpp"
#include "math/matrix.hpp"
#include "physics/spring.hpp"
#include "physics/body.hpp"
#include "physics/spherebody.hpp"
#include <iostream>

namespace _462 {

Spring::Spring()
{
    body1_offset = Vector3::Zero;
    body2_offset = Vector3::Zero;
    damping = 0.0;
}

void Spring::step( real_t dt )
{
	Vector3 AB, F1, F2, dirc;
	real_t delta;
	Quaternion r1, r2;

	AB = body2->position.operator+(body2_offset).operator-(body1->position.operator+(body1_offset));
	delta = length(AB)-equilibrium;
	dirc = AB.operator/(length(AB));

	F1 = dirc.operator*(constant*delta + 5*damping*dot(dirc, body2->velocity.operator-(body1->velocity)));
	F2 = F1.operator*(-1);

	// 施加力
	body1->apply_force(F1, body1_offset);
	body2->apply_force(F2, body2_offset);

	// 更新offset
	if(length(body1->angular_velocity)!=0){
		r1 = Quaternion(body1->angular_velocity, length(body1->angular_velocity)*dt);
		body1_offset = r1.operator*(body1_offset);
	}
    if(length(body2->angular_velocity)!=0){
		r2 = Quaternion(body2->angular_velocity, length(body2->angular_velocity)*dt);
		body2_offset = r2.operator*(body2_offset);
	}
}

}



