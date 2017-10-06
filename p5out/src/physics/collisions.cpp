#include "physics/collisions.hpp"

namespace _462 {

void tiny(Vector3* v)
{
	real_t eps = 0.001;				// 速度小于此值时记为0
	if(length(*v)<eps)
		*v = Vector3::Zero;
}


bool collides( SphereBody& body1, SphereBody& body2, real_t collision_damping )
{
    // TODO detect collision. If there is one, update velocity
	Vector3 d, v2pp, v1p, u1, u2, delta_pos, AB;			
	double k, l;

	AB = body2.position.operator-(body1.position);
	delta_pos = body2.position.operator-(body1.position);
	l = length(delta_pos);
	//1(A)相对2(B)的速度
	v1p = body1.velocity.operator-( body2.velocity);

	if( (l<=(body1.radius+body2.radius))&&(dot(AB,v1p)>0)){
		// 后一个判定条件用于防止小球出现粘连

		d = delta_pos.operator/(l);
		k = 2.0 * body1.mass / (body1.mass+body2.mass) * dot(v1p,d);
		v2pp = d.operator*(k);

		body2.velocity.operator+=(v2pp);
		body1.velocity.operator-=( v2pp.operator*( body2.mass/body1.mass));

		body1.velocity.operator*=(1.0-collision_damping);
		body2.velocity.operator*=(1.0-collision_damping);

		tiny(&body1.velocity);
		tiny(&body2.velocity);

		return true;
	}
	else
		return false;
}

bool collides( SphereBody& body1, TriangleBody& body2, real_t collision_damping )
{
	// P 为球心, Q为P在三角形上的投影
	Vector3 alpha, n, AB, AC, AP, AQ, PQ, cross_ABAC, u;		
	real_t d, la, mu;

	u = body1.velocity.operator-(body2.velocity);
	AB = body2.vertices[1].operator-(body2.vertices[0]);
	AC = body2.vertices[2].operator-(body2.vertices[0]);
	AP = body1.position.operator-(body2.vertices[0]);
	cross_ABAC = cross(AB,AC);

	n = cross_ABAC.operator/(length(cross_ABAC));
	PQ = n.operator*(-dot(AP,n));
	AQ = AP.operator+(PQ);

	la = (dot(AC,AC)*dot(AQ,AB)-dot(AC,AB)*dot(AC,AQ))/dot(cross_ABAC,cross_ABAC);
	mu = (dot(AB,AB)*dot(AQ,AC)-dot(AC,AB)*dot(AB,AQ))/dot(cross_ABAC,cross_ABAC);
	d = length(PQ);

	// 条件5：防止body1卡在body2上
	if((la+mu<1)&&(la>0)&&(mu>0)&&(d<=body1.radius)&&(dot(PQ,u)>0))			
	{
		body1.velocity.operator-=(n.operator*(2*dot(u,n)));
		body1.velocity.operator*=(1-collision_damping);

		tiny(&body1.velocity);

		return true;
	}
	else
		return false;
}

bool collides( SphereBody& body1, PlaneBody& body2, real_t collision_damping )
{
	Vector3 alpha,u;
	real_t d;

	alpha = body1.position.operator-(body2.position);
	d = dot(alpha, body2.normal);

	if(abs(d)<=body1.radius){                     
		body1.velocity.operator-=( (body2.normal.operator*( dot(body1.velocity,body2.normal) )) );
		body1.velocity.operator*=(1-collision_damping);
		tiny(&body1.velocity);
		return true;
	}
	else{
		return false;
	}
}

}
