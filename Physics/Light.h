#ifndef _LIGHT
#define _LIGHT

#include <vector>
#include <algorithm>

typedef b2Vec2 point;

inline int sign(double x, double eps = 1e-6)
{
	return x < -eps ? -1 : x > eps;
}

inline point operator * (const point &a, double k)
{
	return point(a.x * k, a.y * k);
}

inline int side(const point &p, const point &a, const point &b)
{
	return sign(det(b - a, p - a));
}

inline point reflect(const point &p, const point &a, const point &b)
{
	return project(p, a, b) * 2 - p;
}

inline bool cross(const point &a, const point &b, const point &c, const point &d)
{
	return side(a, c, d) * side(b, c, d) <= 0 && side(c, a, b) * side(d, a, b) <= 0;
}

inline point intersect(const point &a, const point &b, const point &c, const point &d)
{
	double s1 = det(b - a, c - a), s2 = det(b - a, d - a);
	return (s2 * c - s1 * d) / (s2 - s1);
}

inline std::pair <double, int> intersect(const point &a, const point &dir, const std::vector <point> &list)
{
	double l = 0, r = 1e6;
	while (r - l > 1e-6)
	{
		double mid = l + (r - l) / 2;
		bool flag = false;
		point b = a + dir * mid;
		for (int i = 0; i < (int)list.size(); i ++)
			if (cross(a, b, list[i], list[(i + 1) % (int)list.size()]))
				flag = true;
		if (flag)
			r = mid;
		else
			l = mid;
	}
	double t = l;
	point b = a + dir * (t + 1e-6);
	int id;
	for (int i = 0; i < (int)list.size(); i ++)
		if (cross(a, b, list[i], list[(i + 1) % (int)list.size()]))
			id = i;
	return std::make_pair(t, id);
}
	
struct light
{
	std::vector <point> list;

	inline light() {}

	inline void construct(point start, point dir, const std::vector <std::vector <point>> &wall, const std::vector <std::vector <point>> &mirror)
	{
		point now = start;
		list.clear();
		list.push_back(now);
		for (int times = 1; times <= 50; times ++)
		{
			std::pair <double, int> wall_min = std::make_pair(1e60, 1);
			int wall_id;
			for (int i = 0; i < (int)wall.size(); i ++)
			{
				auto tmp = intersect(now, dir, wall[i]);
				if (tmp < wall_min)
				{
					wall_min = tmp;
					wall_id = i;
				}
			}
			std::pair <double, int> mirror_min = std::make_pair(1e60, 1);
			int mirror_id;
			for (int i = 0; i < (int)mirror.size(); i ++)
			{
				auto tmp = intersect(now, dir, mirror[i]);
				if (tmp < mirror_min)
				{
					mirror_min = tmp;
					mirror_id = i;
				}
			}
			double t = std::min(mirror_min.first, wall_min.first);
			now = now + dir * t;
			list.push_back(now);
			if (wall_min.first < mirror_min.first)
				break;
			auto tmp = mirror[mirror_id];
			auto a = tmp[mirror_min.second];
			auto b = tmp[(mirror_min.second + 1) % (int)tmp.size()];
			dir = dir + a;
			dir = reflect(dir, a, b);
			dir = dir - a;
		}
	}

	inline std::vector <point> get_light()
	{
		return list;
	}
};

#endif
