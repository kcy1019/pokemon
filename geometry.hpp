#pragma once
#include<cmath>
#include<vector>
#include<cassert>
#include<algorithm>
using std::max;
using std::swap;
using std::vector;
const float EPSILON = 1e-8;
const float PI = 2.0 * acos(0.0);
/* 출처: (대부분) <알고리즘 문제해결 전략> - 구종만 저. */
namespace Geometry {
	// 2차원 벡터를 표현한다
	struct vector2 {
		float x, y;
		explicit vector2(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}

		// 두 벡터의 비교
		bool operator == (const vector2& rhs) const { return x == rhs.x && y == rhs.y; }
		bool operator < (const vector2& rhs) const { if(x != rhs.x) return x < rhs.x; return y < rhs.y; }

		// 벡터의 덧셈과 뺄셈
		vector2 operator + (const vector2& rhs) const { return vector2(x + rhs.x, y + rhs.y); }
		vector2 operator - (const vector2& rhs) const { return vector2(x - rhs.x, y - rhs.y); }
		vector2 operator - () const { return vector2(-x, -y); }

		// 스칼라로 곱셈
		vector2 operator * (float rhs) const { return vector2(x * rhs, y * rhs); }

		// 벡터의 길이를 반환한다
		float norm() const { return hypot(x, y); }

		// 방향이 같은 단위 벡터 (unit vector) 를 반환한다
		vector2 normalize() const { float n = norm(); return vector2(x / n, y / n); }

		// x축의 양의 방향으로부터 이 벡터까지 반시계방향으로 잰 각도
		float polar() const { return fmod(atan2(y, x) + 2*PI, 2*PI); }

		// 내적/외적의 구현
		float dot(const vector2& rhs) const { return x * rhs.x + y * rhs.y; }
		float cross(const vector2& rhs) const { return x * rhs.y - rhs.x * y; }

		// 이 벡터를 rhs 에 사영한 결과
		vector2 project(const vector2& rhs) const {
			vector2 r = rhs.normalize();
			return r * r.dot(*this);
		}
	};

	// 원점에서 벡터 b 가 벡터 a 의 반시계 방향이면 양수, 시계 방향이면 음수, 평행이면 0 을 반환한다
	float ccw(vector2 a, vector2 b) {
		return a.cross(b);
	}

	// 점 p 를 기준으로 벡터 b 가 벡터 a 의 반시계 방향이면 양수, 시계 방향이면 음수, 평행이면 0 을 반환한다
	float ccw(vector2 p, vector2 a, vector2 b) {
		return ccw(a-p, b-p);
	}


	// (a,b) 를 포함하는 선과 (c,d) 를 포함하는 선의 교점을 p 에 반환한다.
	// 두 선이 평행이면 (겹치는 경우를 포함) 거짓을, 아니면 참을 반환한다.
	bool lineIntersection(vector2 a, vector2 b, vector2 c, vector2 d, vector2& p) {
		float det = (b - a).cross(d - c);
		if(det == 0) return false;
		p = a + (b - a) * ((c - a).cross(d - c) / det);
		return true;
	}

	// (a,b) 와 (c,d) 가 평행한 두 선분일 때 이들이 한 점에서 겹치나 확인한다
	bool parallelSegments(vector2 a, vector2 b, vector2 c, vector2 d, vector2& p) {
		if(b < a) swap(a, b);
		if(d < c) swap(c, d);
		// 두 선분이 겹치지 않는 경우를 우선 걸러낸다
		if(b < c || d < a) return false;
		// 두 선분은 확실히 겹친다. 교차점을 하나 찾자.
		if(a < c) p = b; else p = d;
		return true;
	}

	// p 가 (a,b) 를 감싸면서 각 변이 x,y축에 평행한 최소 사각형 내부에 있는지 확인한다
	bool inBoundingRectangle(vector2 p, vector2 a, vector2 b) {
		float x1 = a.x, x2 = b.x, y1 = a.y, y2 = b.y;
		if(x1 > x2) swap(x1, x2);
		if(y1 > y2) swap(y1, y2);
		return x1 <= p.x && p.x <= x2 && y1 <= p.y && p.y <= y2;
	}

	// (a,b) 선분과 (c,d) 선분의 교점을 p 에 반환한다.
	// 교점이 여러 개 있을 경우 아무 것이나 반환한다.
	// 두 선분이 교차하지 않을 경우 false 를 반환한다.
	bool segmentIntersection(vector2 a, vector2 b, vector2 c, vector2 d, vector2& p) {
		// 두 직선이 평행인 경우를 우선 예외로 처리한다
		if(!lineIntersection(a, b, c, d, p)) return parallelSegments(a, b, c, d, p);
		// p 가 두 선분에 포함되어 있는 경우에만 참을 반환한다
		return inBoundingRectangle(p, a, b) && inBoundingRectangle(p, c, d);

	}

	// 두 구간 [a,b], [c,d] 가 교집합이 없는지 여부를 확인한다
	float disjoint(float a, float b, float c, float d) {
		if(a > b) swap(a, b); if(c > d) swap(c, d);
		return b < c || d < a;
	}

	// 두 선분이 서로 접촉하는지 여부를 반환한다
	bool segmentIntersects(vector2 a, vector2 b, vector2 c, vector2 d) {
		float ab = ccw(a, b, c) * ccw(a, b, d);
		float cd = ccw(c, d, a) * ccw(c, d, b);
		// 두 선분이 한 직선 위에 있는 경우
		if(ab == 0 && cd == 0)
			return !disjoint(a.x, b.x, c.x, d.x) &&
			!disjoint(a.y, b.y, c.y, d.y);
		return ab <= 0 && cd <= 0;
	}

	// 점 p 에서 (a,b) 직선에 내린 수선의 발을 구한다
	vector2 perpendicularFoot(vector2 p, vector2 a, vector2 b) {
		vector2 delta = (b - a).normalize();
		return a + delta * (p - a).dot(delta);
	}

	// 점 p 와 (a,b) 직선 사이의 거리를 구한다
	float pointToLine(vector2 p, vector2 a, vector2 b) {
		return (p - perpendicularFoot(p, a, b)).norm();
	}

	// 주어진 단순 다각형 p 의 넓이를 구한다. p 는 각 꼭지점의 위치 벡터의 집합으로 주어진다.
	float area(const vector<vector2>& p) {
		float ret = 0;
		for(int i = 0; i < p.size(); ++i) {
			int j = (i + 1) % p.size();
			ret = ret + (p[i].x * p[j].y - p[j].x * p[i].y);
		}
		return fabs(ret) / 2.0;
	}

	// 점 q 가 다각형 p 안에 포함되어 있을 경우 참, 아닌 경우 거짓을 반환한다.
	// q 가 다각형의 경계 위에 있는 경우의 반환값은 정의되어 있지 않다.
	bool isInside(vector2 q, const vector<vector2>& p) {
		int crosses = 0;
		for(int i = 0; i < p.size(); ++i) {
			int j = (i + 1) % p.size();
			// (p[i], p[j]) 가 반직선을 세로로 가로지르는가?
			if((p[i].y > q.y) != (p[j].y > q.y)) {
				float atX = (p[j].x - p[i].x) * (q.y - p[i].y) / (p[j].y - p[i].y) + p[i].x;
				if(q.x < atX)
					++crosses;
			}
		}
		return crosses % 2 > 0;
	}


	typedef vector<vector2> polygon;

	// points 에 있는 점들을 모두 포함하는 최소의 볼록 다각형을 찾는다.
	// 볼록 다각형의 꼭지점은 제일 왼쪽 아래 점부터 시작해서 반시계 방향으로 반환된다.
	// 한 직선 위에 세 개 이상의 점이 있을 경우 가운데 것들은 무시된다.
	polygon giftWrap(const vector<vector2>& points) {
		int n = points.size();
		polygon hull;

		// 가장 왼쪽 아래 점을 찾는다. 이 점은 껍질에 반드시 포함된다.
		vector2 pivot = *min_element(points.begin(), points.end());
		hull.push_back(pivot);

		while(true) {
			// (p - ph) 가 가장 왼쪽인 점을 찾는다. 평행인 점이 여러 개 있으면
			// 가장 먼 것을 선택한다.
			vector2 ph = hull.back(), next = points[0];
			for(int i = 1; i < n; i++) {
				float cross = ccw(ph, next, points[i]);
				float dist = (next - ph).norm() - (points[i] - ph).norm();
				if(cross > 0 || (cross == 0 && dist < 0))
					next = points[i];
			}
			// 시작점으로 돌아왔으면 종료한다.
			if(next == pivot) break;
			hull.push_back(next);
		}

		return hull;
	}


	// (a,b) 를 포함하는 직선으로 다각형 p 를 자른 뒤, (a,b) 의 왼쪽에 있는 부분들을 반환한다
	polygon cutPoly(const polygon& p, const vector2& a, const vector2& b) {
		int n = p.size();
		vector<bool> inside(n);
		for(int i = 0; i < n; ++i)
			inside[i] = ccw(a, b, p[i]) >= 0;
		// 이외의 경우에는 일부는 직선 왼쪽에, 일부는 직선 오른쪽에 떨어진다
		polygon ret;
		for(int i = 0; i < n; ++i) {
			int j = (i + 1) % n;
			if(inside[i]) ret.push_back(p[i]);
			if(inside[i] != inside[j]) {
				vector2 intersection;
				assert(lineIntersection(p[i], p[j], a, b, intersection));
				ret.push_back(intersection);
			}
		}
		return ret;
	}

	// 시계 반대방향으로 주어진 볼록 다각형에서 가장 먼 꼭지점 쌍 사이의 거리를 반환한다.
	float diameter(const polygon& p) {
		int n = p.size();

		// 가장 왼쪽에 있는 점과 오른쪽에 있는 점을 우선 찾는다
		int left = min_element(p.begin(), p.end()) - p.begin();
		int right = max_element(p.begin(), p.end()) - p.begin();

		// p[left] 와 p[right] 에 각각 수직선을 붙인다. 두 수직선은 서로 정반대 방향을 가리키므로,
		// A 의 방향만을 표현하면 된다.
		vector2 calipersA(0, 1);
		float ret = (p[right] - p[left]).norm();

		// toNext[i] = p[i] 에서 다음 점까지의 방향을 나타내는 단위벡터
		vector<vector2> toNext(n);
		for(int i = 0; i < n; i++)
			toNext[i] = (p[(i+1) % n] - p[i]).normalize();

		// a 와 b 는 각각 두 선분이 어디에 붙은 채로 회전하고 있는가를 나타낸다
		int a = left, b = right;
		// 반 바퀴 돌아서 두 선분이 서로 위치를 바꿀 때까지 계속한다
		while(a != right || b != left) {
			// a 에서 다음 점까지의 각도와 b 에서 다음 점까지의 각도 중 어느 쪽이 작은가 확인
			float cosThetaA = calipersA.dot(toNext[a]);
			float cosThetaB = -calipersA.dot(toNext[b]);
			if(cosThetaA > cosThetaB) { // thetaA < thetaB
				calipersA = toNext[a];
				a = (a + 1) % n;
			}
			else {
				calipersA = -toNext[b];
				b = (b + 1) % n;
			}
			ret = max(ret, (p[a] - p[b]).norm());
		}
		return ret;
	}

	// 두 다각형이 서로 닿거나 겹치는지 여부를 반환한다.
	// 한 점이라도 겹친다면 true 를 반환한다.
	bool polygonIntersects(const polygon& p, const polygon& q) {
		int n = p.size(), m = q.size();
		// 우선 한 다각형이 다른 다각형에 포함되어 있는 경우를 확인하자
		if(isInside(p[0], q) || isInside(q[0], p)) return true;
		for (int i = 0; i < n; i++) {
			if (isInside(p[i], q)) return true;
		}
		// 이외의 경우, 두 다각형이 서로 겹친다면 서로 닿는 두 변이 반드시 존재한다
		for(int i = 0; i < n; i++)
			for(int j = 0; j < m; j++)
				if(segmentIntersects(p[i], p[(i+1)%n], q[j], q[(j+1)%m]) ||
				   segmentIntersects(p[i], p[(i+1)%n], q[j], q[(j+2)%m])) {
					return true;
				}
		return false;
	}
	
	/* Fast Convex Hull (Andrew's monotone chain) */
	vector<vector2> convexHull(const vector<vector2>& q)
	{
		vector<vector2> p(q.begin(), q.end());
        int n = p.size(), hull_size = 0;
        vector<vector2> ret(n << 1);
        sort(p.begin(), p.end());

        for (int i = 0; i < n; i++) { // lower
                while (hull_size >= 2 &&
                           ccw(ret[hull_size-2], ret[hull_size-1], p[i]) <= 0) --hull_size;
                ret[hull_size++] = p[i];
        }

        for (int i = n-2, t = hull_size+1; i >= 0; i--) { // upper
                while (hull_size >= t &&
                           ccw(ret[hull_size-2], ret[hull_size-1], p[i]) <= 0) --hull_size;
                ret[hull_size++] = p[i];
        }

        ret.resize(hull_size - 1);
        return ret;
	}
};
