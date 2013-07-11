/*
 * Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#include "Test.h"
#include "Render.h"

#include "GL/glut.h"

#include <iostream>

void DestructionListener::SayGoodbye(b2Joint* joint) {
	if (test->m_mouseJoint == joint) {
		test->m_mouseJoint = NULL;
	} else {
		test->JointDestroyed(joint);
	}
}

Test::Test() {

	b2Vec2 gravity;
	m_world = new b2World(gravity);
	m_bomb = NULL;
	m_textLine = 30;
	m_mouseJoint = NULL;
	m_pointCount = 0;

	m_destructionListener.test = this;
	m_world->SetDestructionListener(&m_destructionListener);
	m_world->SetDebugDraw(&m_debugDraw);

	m_stepCount = 0;

	b2BodyDef bodyDef;
	m_groundBody = m_world->CreateBody(&bodyDef);

}

Test::~Test() {
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
	m_world = NULL;
}

void Test::DrawTitle(int x, int y, const char *string) {
	m_debugDraw.DrawString(x, y, string);
	m_debugDraw.DrawString(x, y+10, "Zoom in/out with Z/X");
}

class QueryCallback: public b2QueryCallback {
public:
	QueryCallback(const b2Vec2& point) {
		m_point = point;
		m_fixture = NULL;
	}

	bool ReportFixture(b2Fixture* fixture) {
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody) {
			bool inside = fixture->TestPoint(m_point);
			if (inside) {
				m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};

void Test::MouseDown(const b2Vec2& p) {
	m_mouseWorld = p;

	if (m_mouseJoint != NULL) {
		return;
	}

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	m_world->QueryAABB(&callback, aabb);

	if (callback.m_fixture) {
		b2Body* body = callback.m_fixture->GetBody();
		b2MouseJointDef md;
		md.bodyA = m_groundBody;
		md.bodyB = body;
		md.target = p;
		md.maxForce = 1000.0f * body->GetMass();
		m_mouseJoint = (b2MouseJoint*) m_world->CreateJoint(&md);
		body->SetAwake(true);
	}
}

void Test::ShiftMouseDown(const b2Vec2& p) {
	m_mouseWorld = p;

	if (m_mouseJoint != NULL) {
		return;
	}
}

void Test::MouseUp(const b2Vec2& /*p*/) {

	if (m_mouseJoint) {
		m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
	}

}

void Test::MouseMove(const b2Vec2& p) {
	m_mouseWorld = p;

	if (m_mouseJoint) {
		m_mouseJoint->SetTarget(p);
	}
}

void Test::Step(Settings* settings) {

	float32 timeStep =
			settings->hz > 0.0f ? 1.0f / settings->hz : float32(0.0f);

	if (settings->pause) {
		if (settings->singleStep) {
			settings->singleStep = 0;
		} else {
			timeStep = 0.0f;
		}

		m_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += 15;
	}

	uint32 flags = 0;
	flags += settings->drawShapes * b2Draw::e_shapeBit;
	flags += settings->drawJoints * b2Draw::e_jointBit;
	flags += settings->drawAABBs * b2Draw::e_aabbBit;
	flags += settings->drawPairs * b2Draw::e_pairBit;
	flags += settings->drawCOMs * b2Draw::e_centerOfMassBit;
	m_debugDraw.SetFlags(flags);

	m_pointCount = 0;

	if (timeStep > 0) {
		this->StepSimulator(settings, timeStep);
	}

	m_world->DrawDebugData();

	if (timeStep > 0.0f) {
		++m_stepCount;
	}

	if (settings->drawStats) {
		m_debugDraw.DrawString(5, m_textLine,
				"bodies/contacts/joints/proxies = %d/%d/%d",
				m_world->GetBodyCount(), m_world->GetContactCount(),
				m_world->GetJointCount(), m_world->GetProxyCount());
		m_textLine += 15;
	}

	if (m_mouseJoint) {
		b2Vec2 p1 = m_mouseJoint->GetAnchorB();
		b2Vec2 p2 = m_mouseJoint->GetTarget();

		glPointSize(4.0f);
		glColor3f(0.0f, 1.0f, 0.0f);
		glBegin(GL_POINTS);
		glVertex2f(p1.x, p1.y);
		glVertex2f(p2.x, p2.y);
		glEnd();
		glPointSize(1.0f);

		glColor3f(0.8f, 0.8f, 0.8f);
		glBegin(GL_LINES);
		glVertex2f(p1.x, p1.y);
		glVertex2f(p2.x, p2.y);
		glEnd();
	}

	if (settings->drawContactPoints) {
		//const float32 k_impulseScale = 0.1f;
		const float32 k_axisScale = 0.3f;

		for (int32 i = 0; i < m_pointCount; ++i) {
			ContactPoint* point = m_points + i;

			if (point->state == b2_addState) {
				// Add
				m_debugDraw.DrawPoint(point->position, 10.0f,
						b2Color(0.3f, 0.95f, 0.3f));
			} else if (point->state == b2_persistState) {
				// Persist
				m_debugDraw.DrawPoint(point->position, 5.0f,
						b2Color(0.3f, 0.3f, 0.95f));
			}

			if (settings->drawContactNormals == 1) {
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_axisScale * point->normal;
				m_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
			} else if (settings->drawContactForces == 1) {
				//b2Vec2 p1 = point->position;
				//b2Vec2 p2 = p1 + k_forceScale * point->normalForce * point->normal;
				//DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
			}

			if (settings->drawFrictionForces == 1) {
				//b2Vec2 tangent = b2Cross(point->normal, 1.0f);
				//b2Vec2 p1 = point->position;
				//b2Vec2 p2 = p1 + k_forceScale * point->tangentForce * tangent;
				//DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
			}
		}
	}
}
