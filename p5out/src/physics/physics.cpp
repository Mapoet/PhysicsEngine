#include "physics/physics.hpp"

namespace _462 {

Physics::Physics()
{
    reset();
}

Physics::~Physics()
{
    reset();
}

void Physics::step( real_t dt )
{

    // TODO step the world forward by dt. Need to detect collisions, apply
    // forces, and integrate positions and orientations.
    //
    // Note: put RK4 here, not in any of the physics bodies
    //
    // Must use the functions that you implemented
    //
    // Note, when you change the position/orientation of a physics object,
    // change the position/orientation of the graphical object that represents
    // it
	int i,j;
	// 初始化作用力和力矩
	for(i=0; i<num_spheres(); i++){
		spheres[i]->force = Vector3::Zero;
		spheres[i]->torque = Vector3::Zero;
	}

	// 施加弹力
	for(i=0; i<num_springs(); i++)
		springs[i]->step(dt);

	for(i=0; i<num_spheres(); i++){
		// 碰撞：球-球
		for(j=0; j<i; j++)
			collides(*spheres[i], *spheres[j], this->collision_damping);
		// 碰撞：球-平面
		for(j=0; j<num_planes(); j++)
			collides(*spheres[i], *planes[j], this->collision_damping);
		// 碰撞：球-三角形
		for(j=0; j<num_triangles(); j++)
			collides(*spheres[i], *triangles[j], this->collision_damping);

		// 施加重力，更新状态
		spheres[i]->apply_force(spheres[i]->mass * gravity, Vector3::Zero);			
		spheres[i]->step_position(dt,0);
		spheres[i]->step_orientation(dt,0);
		spheres[i]->update();
	}
}

void Physics::add_sphere( SphereBody* b )
{
    spheres.push_back( b );
}

size_t Physics::num_spheres() const
{
    return spheres.size();
}

void Physics::add_plane( PlaneBody* p )
{
    planes.push_back( p );
}

size_t Physics::num_planes() const
{
    return planes.size();
}

void Physics::add_triangle( TriangleBody* t )
{
    triangles.push_back( t );
}

size_t Physics::num_triangles() const
{
    return triangles.size();
}

void Physics::add_spring( Spring* s )
{
    springs.push_back( s );
}

size_t Physics::num_springs() const
{
    return springs.size();
}

void Physics::reset()
{
    for ( SphereList::iterator i = spheres.begin(); i != spheres.end(); i++ ) {
        delete *i;
    }
    for ( PlaneList::iterator i = planes.begin(); i != planes.end(); i++ ) {
        delete *i;
    }
    for ( TriangleList::iterator i = triangles.begin(); i != triangles.end(); i++ ) {
        delete *i;
    }
    for ( SpringList::iterator i = springs.begin(); i != springs.end(); i++ ) {
        delete *i;
    }

    spheres.clear();
    planes.clear();
    triangles.clear();
    springs.clear();
    
    gravity = Vector3::Zero;					// 重力
	collision_damping = 0.0;					// 碰撞阻尼
}

}
