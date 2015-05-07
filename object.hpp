#pragma once
#include <ctime>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "geometry.hpp"
#include "text.hpp"
#include <cstdio>
using namespace Geometry;

static unsigned UOID = 0;

struct WindowSize {
	static float left, right, bot, top;
};

struct Object {
	vector<vector2> drawer, convex;
	vector2 center, velocity;
	float angle;
	GLenum draw_mode;
	unsigned uoid, last_collision, score,
		last_effective_collision;
	bool sticky;
	vector2 repulsion;

	explicit Object(const vector<vector2>& points,
					GLenum draw_mode):
		drawer(points), convex(convexHull(points)),
		center(vector2(0.,0.)), velocity(vector2(0.,0.)),
		angle(0.), draw_mode(draw_mode),
		uoid(++UOID), sticky(false), score(0),
		last_effective_collision(0), last_collision(0),
		repulsion(vector2(0., 0.)) {}

	inline void stick(float dx, float dy) {
		sticky = true;
		repulsion = vector2(dx, dy);
	}

	inline void set_velocity(float dx, float dy) {
		velocity = vector2(dx, dy);
	}

	inline bool update(vector<Object>& others) {
		bool out_of_bound = false;
		// Collision with boundary.
		if (!sticky) {
			auto next = center + velocity;
			for (const auto& p: convex) {
				auto q = p - center + next;
				if (q.x < WindowSize::left)
					velocity.x = 3,
					out_of_bound = true;
				if (q.x > WindowSize::right)
					velocity.x = -3,
					out_of_bound = true;
				if (q.y > WindowSize::top)
					velocity.y = -3,
					out_of_bound = true;
				if (q.y < WindowSize::bot)
					velocity.y = 3,
					out_of_bound = true;
			}
			next = center + velocity;
			move(next.x, next.y);
		}
		// Collision with other objects.
		for (auto& poly: others)
			if (poly.uoid != uoid)
				detect_collision(poly);
		return out_of_bound;
	}
	
	inline void draw(void) {
		glPushMatrix();
		{
			if (uoid != 1 && draw_mode != GL_POLYGON) {
				char label[64] = "";
				sprintf(label, "+%u", score);
				DrawText(label, center.x, center.y, true);
			}
			glPointSize(1.);
			glTranslatef(center.x, center.y, 0.);
			glRotatef(angle / PI * 180., 0., 0., 1.);
			glBegin(draw_mode);
				for (const auto& pt: drawer)
					glVertex2f(pt.x, pt.y);
			glEnd();
		}
		glPopMatrix();
	}

	inline void move(const float x, const float y) {
		auto new_center = vector2(x, y);
		for (auto& p: convex)
			p = p - center + new_center;
		center = new_center;
	}

	inline void rotate(const float ang) {
		float da = - ang + angle;
		velocity = vector2(cos(da) * velocity.x + sin(da) * velocity.y, 
						   -sin(da) * velocity.x + cos(da) * velocity.y);
		for (auto& p: convex) {
			p = p - center;
			p = vector2(cos(da) * p.x + sin(da) * p.y,
						-sin(da) * p.x + cos(da) * p.y);
			p = p + center;
		}
		angle = ang;
	}
	
	inline bool detect_collision(Object& rhs) {
		if (abs((long long)rhs.last_collision - last_collision) < 3 &&
			last_collision && rhs.last_collision)
			return false;

		bool det = polygonIntersects(convex, rhs.convex);
		if (!det) return false;
		bool dir = (rhs.center - center).x > 0;
		float rot = fmod(angle + (dir?-1:1) * 0.1512502, 2*PI);
		if (!sticky) {
			rotate(rot);
			velocity = -velocity;
		} else {
			rot = PI;
		}

		if (!rhs.sticky) {
			if (sticky) {
				rhs.rotate(fmod(rhs.angle + PI, 2*PI));
				float dx = -rhs.velocity.x, dy = -rhs.velocity.y;
				if (repulsion.x != 0)
					dx = repulsion.x;
				if (repulsion.y != 0)
					dy = repulsion.y;
				rhs.set_velocity(dx, dy);
			} else {
				rhs.rotate(-rot);
				auto rvel = rhs.velocity - velocity;
				rhs.set_velocity(rvel.x, rvel.y);
			}
		}
		
		if (!rhs.sticky && !sticky &&
			rhs.draw_mode != GL_POLYGON &&
			draw_mode != GL_POLYGON &&
			rhs.score || score || rhs.uoid == 1 || uoid == 1) {
			if (rhs.uoid != 1)
				++rhs.score;
			if (uoid != 1)
				++score;
			last_effective_collision = clock();
			rhs.last_effective_collision = last_effective_collision;
		}

		rhs.last_collision = last_collision = clock();
		return true;
	}
};