#include "physics/spherebody.hpp"
#include "math/vector.hpp"
#include "math/matrix.hpp"
#include "scene/sphere.hpp"
#include <iostream>
#include <exception>
#include <algorithm>

namespace _462 {

SphereBody::SphereBody( Sphere* geom )
{
    sphere = geom;
	mass = 0.0;
	radius = sphere->radius;
    position = sphere->position;
    orientation = sphere->orientation;
    velocity = Vector3::Zero;
    angular_velocity = Vector3::Zero;
    force = Vector3::Zero;
    torque = Vector3::Zero;
}

Vector3 RK4(Vector3 x0, double t0, double h, Vector3 (*pf)(Vector3, double) )				// RK4算法
{
	Vector3 k1, k2, k3, k4;
	Vector3 delta;
	k1 = (*pf)(x0, t0).operator*(h);
	k2 = (*pf)(x0 + k1.operator*(0.5), t0+0.5*h).operator*(h);
	k3 = (*pf)(x0 + k2.operator*(0.5), t0+0.5*h).operator*(h);
	k4 = (*pf)(x0 + k3, t0+h).operator*(h);
	
	delta.operator+=(k1);
	delta.operator+=(k2.operator*(2));
	delta.operator+=(k3.operator*(2));
	delta.operator+=(k4);

	delta.operator/=(6);
	return x0.operator+(delta);
}

Vector3 SphereBody::step_position( real_t dt, real_t motion_damping )			
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
	Vector3 delta_pos = Vector3::Zero;
	Vector3 delta_vel = Vector3::Zero;
	Vector3 acc = Vector3::Zero;

	// 施加运动阻尼
	apply_force(velocity.operator*(-motion_damping), Vector3::Zero);

	delta_pos = this->velocity.operator*(dt);					// 更新坐标
	this->position.operator+=(delta_pos);

	acc = this->force.operator/(this->mass);					// 更新速度
	delta_vel = acc.operator*(dt); 
	this->velocity.operator+=(delta_vel);

	return Vector3::Zero;
}

Vector3 SphereBody::step_orientation( real_t dt, real_t motion_damping )				
{
    // Note: This function is here as a hint for an approach to take towards
    // programming RK4, you should add more functions to help you or change the
    // scheme
    // vec.x = rotation along x axis
    // vec.y = rotation along y axis
    // vec.z = rotation along z axis

	Vector3 alpha = Vector3::Zero;
	Quaternion r = Quaternion::Zero;
	real_t I;

	if(length(angular_velocity)!=0){
		r = Quaternion(angular_velocity, length(angular_velocity)*dt);
		orientation = orientation.operator*(r);								// 计算角位移
	}
	I = 0.4*mass*radius*radius;
	alpha = torque.operator/(I);										// 更新角速度
	angular_velocity.operator+=(alpha.operator*(dt)); 

	return Vector3::Zero;
}

void SphereBody::apply_force( const Vector3& f, const Vector3& offset )
{
    //apply force/torque to sphere

	force.operator+=(f);
	torque.operator+=(cross(offset, f));									// 叉乘得到力矩

}

void SphereBody::update()
{
	sphere->position = position;
	sphere->orientation = orientation;
}

}
