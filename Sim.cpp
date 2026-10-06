#include "Sim.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr float PI = 3.14159265358979323846f;
constexpr float EPS = 1e-4f;

float cross2(glm::vec2 a, glm::vec2 b) {
	return a.x * b.y - a.y * b.x;
}

bool ray_segment(glm::vec2 origin, glm::vec2 dir, glm::vec2 a, glm::vec2 b, float &t_out, glm::vec2 *normal_out) {
	glm::vec2 ab = b - a;
	float denom = cross2(dir, ab);
	if (std::abs(denom) < 1e-8f) return false;
	glm::vec2 ao = a - origin;
	float t = cross2(ao, ab) / denom;
	float u = cross2(ao, dir) / denom;
	if (t < EPS || u < 0.0f || u > 1.0f) return false;
	t_out = t;
	if (normal_out) {
		glm::vec2 n = glm::normalize(glm::vec2(-ab.y, ab.x));
		if (glm::dot(n, dir) > 0.0f) n = -n;
		*normal_out = n;
	}
	return true;
}

void body_axes(DynamicBody const &b, glm::vec2 &axis_x, glm::vec2 &axis_y) {
	float c = std::cos(b.angle);
	float s = std::sin(b.angle);
	axis_x = glm::vec2(c, s);
	axis_y = glm::vec2(-s, c);
}

glm::vec2 body_mirror_normal_world(DynamicBody const &b) {
	glm::vec2 ax, ay;
	body_axes(b, ax, ay);
	glm::vec2 n = ax * b.mirror_face_local.x + ay * b.mirror_face_local.y;
	float len = glm::length(n);
	return (len > 1e-6f) ? (n / len) : ax;
}

bool body_hit_is_mirror_face(DynamicBody const &b, glm::vec2 hit_normal) {
	if (!b.has_mirror_face) return false;
	float nlen = glm::length(hit_normal);
	if (nlen < 1e-6f) return false;
	return glm::dot(hit_normal / nlen, body_mirror_normal_world(b)) > 0.75f;
}

void body_corners(DynamicBody const &b, glm::vec2 out[4]) {
	glm::vec2 ax, ay;
	body_axes(b, ax, ay);
	glm::vec2 hx = ax * b.half_extents.x;
	glm::vec2 hy = ay * b.half_extents.y;
	out[0] = b.position - hx - hy;
	out[1] = b.position + hx - hy;
	out[2] = b.position + hx + hy;
	out[3] = b.position - hx + hy;
}

glm::vec2 world_to_body(DynamicBody const &b, glm::vec2 p) {
	glm::vec2 ax, ay;
	body_axes(b, ax, ay);
	glm::vec2 d = p - b.position;
	return glm::vec2(glm::dot(d, ax), glm::dot(d, ay));
}

glm::vec2 closest_point_on_body(DynamicBody const &b, glm::vec2 p) {
	glm::vec2 ax, ay;
	body_axes(b, ax, ay);
	glm::vec2 local = world_to_body(b, p);
	local.x = glm::clamp(local.x, -b.half_extents.x, b.half_extents.x);
	local.y = glm::clamp(local.y, -b.half_extents.y, b.half_extents.y);
	return b.position + ax * local.x + ay * local.y;
}

float body_circle_overlap_distance(DynamicBody const &b, glm::vec2 center, float radius) {
	glm::vec2 closest = closest_point_on_body(b, center);
	return glm::length(closest - center) - radius;
}

bool ray_obb(glm::vec2 origin, glm::vec2 dir, DynamicBody const &b, float &t_out, glm::vec2 *normal_out) {
	glm::vec2 ax, ay;
	body_axes(b, ax, ay);
	glm::vec2 d = origin - b.position;
	glm::vec2 o(glm::dot(d, ax), glm::dot(d, ay));
	glm::vec2 rd(glm::dot(dir, ax), glm::dot(dir, ay));

	float tmin = -std::numeric_limits< float >::infinity();
	float tmax = std::numeric_limits< float >::infinity();
	int hit_axis = -1;
	float hit_sign = 0.0f;

	for (int i = 0; i < 2; ++i) {
		float oi = (i == 0) ? o.x : o.y;
		float di = (i == 0) ? rd.x : rd.y;
		float he = (i == 0) ? b.half_extents.x : b.half_extents.y;
		if (std::abs(di) < 1e-8f) {
			if (oi < -he || oi > he) return false;
			continue;
		}
		float inv = 1.0f / di;
		float t1 = (-he - oi) * inv;
		float t2 = (he - oi) * inv;
		float sign1 = -1.0f;
		float sign2 = 1.0f;
		if (t1 > t2) {
			std::swap(t1, t2);
			std::swap(sign1, sign2);
		}
		if (t1 > tmin) {
			tmin = t1;
			hit_axis = i;
			hit_sign = sign1;
		}
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax) return false;
	}

	float t = (tmin >= EPS) ? tmin : tmax;
	if (t < EPS) return false;
	t_out = t;
	if (normal_out && hit_axis >= 0) {
		glm::vec2 n = (hit_axis == 0) ? ax : ay;
		n *= hit_sign;
		if (glm::dot(n, dir) > 0.0f) n = -n;
		*normal_out = n;
	}
	return true;
}

bool ray_aabb(glm::vec2 origin, glm::vec2 dir, glm::vec2 center, glm::vec2 half, float &t_out) {
	glm::vec2 inv(
		std::abs(dir.x) > 1e-8f ? 1.0f / dir.x : 1e8f * (dir.x >= 0.0f ? 1.0f : -1.0f),
		std::abs(dir.y) > 1e-8f ? 1.0f / dir.y : 1e8f * (dir.y >= 0.0f ? 1.0f : -1.0f)
	);
	glm::vec2 t0 = (center - half - origin) * inv;
	glm::vec2 t1 = (center + half - origin) * inv;
	float tmin = std::max(std::min(t0.x, t1.x), std::min(t0.y, t1.y));
	float tmax = std::min(std::max(t0.x, t1.x), std::max(t0.y, t1.y));
	if (tmax < tmin || tmax < EPS) return false;
	t_out = tmin >= EPS ? tmin : tmax;
	return t_out >= EPS;
}

void mirror_endpoints(Mirror const &m, glm::vec2 &a, glm::vec2 &b) {
	glm::vec2 tangent(std::cos(m.angle), std::sin(m.angle));
	a = m.position - 0.5f * m.length * tangent;
	b = m.position + 0.5f * m.length * tangent;
}

void add_bounds_as_walls(GameState const &state, std::vector< Wall > &out) {
	glm::vec2 mn = state.world_min;
	glm::vec2 mx = state.world_max;
	out.push_back({glm::vec2(mn.x, mn.y), glm::vec2(mx.x, mn.y)});
	out.push_back({glm::vec2(mx.x, mn.y), glm::vec2(mx.x, mx.y)});
	out.push_back({glm::vec2(mx.x, mx.y), glm::vec2(mn.x, mx.y)});
	out.push_back({glm::vec2(mn.x, mx.y), glm::vec2(mn.x, mn.y)});
}

enum class HitKind : uint8_t { None, Wall, Mirror, Gate, Body, Bound };

struct Hit {
	HitKind kind = HitKind::None;
	float t = std::numeric_limits< float >::infinity();
	glm::vec2 normal = glm::vec2(0.0f, 1.0f);
	int index = -1;
};

Hit find_hit(GameState const &state, glm::vec2 origin, glm::vec2 dir, int ignore_mirror = -1, int ignore_body = -1) {
	Hit best;
	std::vector< Wall > bounds;
	add_bounds_as_walls(state, bounds);

	auto consider_wall = [&](glm::vec2 a, glm::vec2 b, HitKind kind, int idx) {
		float t;
		glm::vec2 n;
		if (ray_segment(origin, dir, a, b, t, &n) && t < best.t) {
			best.kind = kind;
			best.t = t;
			best.normal = n;
			best.index = idx;
		}
	};

	for (int i = 0; i < (int)state.walls.size(); ++i) {
		consider_wall(state.walls[i].a, state.walls[i].b, HitKind::Wall, i);
	}
	for (int i = 0; i < (int)bounds.size(); ++i) {
		consider_wall(bounds[i].a, bounds[i].b, HitKind::Bound, i);
	}
	for (int i = 0; i < (int)state.mirrors.size(); ++i) {
		if (i == ignore_mirror) continue;
		glm::vec2 a, b;
		mirror_endpoints(state.mirrors[i], a, b);
		consider_wall(a, b, HitKind::Mirror, i);
	}
	for (int i = 0; i < (int)state.gates.size(); ++i) {
		Gate const &g = state.gates[i];
		// Ray vs OBB: test in gate-local frame (supports model rotation).
		glm::vec2 rel = origin - g.position;
		float ca = std::cos(g.angle), sa = std::sin(g.angle);
		glm::vec2 local_o(ca * rel.x + sa * rel.y, -sa * rel.x + ca * rel.y);
		glm::vec2 local_d(ca * dir.x + sa * dir.y, -sa * dir.x + ca * dir.y);
		float t;
		if (ray_aabb(local_o, local_d, glm::vec2(0.0f), g.half_extents, t) && t < best.t) {
			best.kind = HitKind::Gate;
			best.t = t;
			best.index = i;
			best.normal = glm::vec2(0.0f);
		}
	}
	for (int i = 0; i < (int)state.bodies.size(); ++i) {
		if (i == ignore_body) continue;
		float t;
		glm::vec2 n;
		if (ray_obb(origin, dir, state.bodies[i], t, &n) && t < best.t) {
			best.kind = HitKind::Body;
			best.t = t;
			best.normal = n;
			best.index = i;
		}
	}
	return best;
}

void project_body(DynamicBody const &b, glm::vec2 axis, float &min_out, float &max_out) {
	glm::vec2 corners[4];
	body_corners(b, corners);
	min_out = max_out = glm::dot(corners[0], axis);
	for (int i = 1; i < 4; ++i) {
		float p = glm::dot(corners[i], axis);
		min_out = std::min(min_out, p);
		max_out = std::max(max_out, p);
	}
}

void apply_separation(DynamicBody &a, DynamicBody &b, glm::vec2 n, float overlap, float restitution) {
	float inv_a = 1.0f / a.mass;
	float inv_b = 1.0f / b.mass;
	float inv_sum = inv_a + inv_b;
	if (inv_sum <= 0.0f) return;

	a.position -= n * (overlap * (inv_a / inv_sum));
	b.position += n * (overlap * (inv_b / inv_sum));

	glm::vec2 rel = b.velocity - a.velocity;
	float vn = glm::dot(rel, n);
	if (vn >= 0.0f) return;

	float j = -(1.0f + restitution) * vn / inv_sum;
	glm::vec2 impulse = j * n;
	a.velocity -= impulse * inv_a;
	b.velocity += impulse * inv_b;

	a.angular_velocity -= cross2(n, impulse) * 0.15f / std::max(a.inertia, 0.05f);
	b.angular_velocity += cross2(n, impulse) * 0.15f / std::max(b.inertia, 0.05f);
}

void resolve_boxes(DynamicBody &a, DynamicBody &b, float restitution) {
	glm::vec2 axes[4];
	body_axes(a, axes[0], axes[1]);
	body_axes(b, axes[2], axes[3]);

	float min_overlap = std::numeric_limits< float >::infinity();
	glm::vec2 sep_axis(0.0f, 1.0f);

	for (int i = 0; i < 4; ++i) {
		glm::vec2 axis = axes[i];
		float len = glm::length(axis);
		if (len < 1e-6f) continue;
		axis /= len;
		float amin, amax, bmin, bmax;
		project_body(a, axis, amin, amax);
		project_body(b, axis, bmin, bmax);
		float overlap = std::min(amax, bmax) - std::max(amin, bmin);
		if (overlap <= 0.0f) return;
		if (overlap < min_overlap) {
			min_overlap = overlap;
			sep_axis = axis;
		}
	}

	if (glm::dot(b.position - a.position, sep_axis) < 0.0f) sep_axis = -sep_axis;
	apply_separation(a, b, sep_axis, min_overlap, restitution);
}

void resolve_box_segment(DynamicBody &body, glm::vec2 a, glm::vec2 b, float restitution) {
	glm::vec2 ab = b - a;
	float len2 = glm::dot(ab, ab);
	if (len2 < 1e-8f) return;

	// Closest point on segment to box center, then closest on box to that — penetration along that axis.
	float u = glm::clamp(glm::dot(body.position - a, ab) / len2, 0.0f, 1.0f);
	glm::vec2 on_seg = a + u * ab;
	glm::vec2 on_box = closest_point_on_body(body, on_seg);
	glm::vec2 delta = on_box - on_seg;
	float dist = glm::length(delta);

	// Also test segment endpoints against box interior:
	glm::vec2 local_a = world_to_body(body, a);
	glm::vec2 local_b = world_to_body(body, b);
	bool a_inside = std::abs(local_a.x) <= body.half_extents.x && std::abs(local_a.y) <= body.half_extents.y;
	bool b_inside = std::abs(local_b.x) <= body.half_extents.x && std::abs(local_b.y) <= body.half_extents.y;

	if (dist < 1e-8f && !a_inside && !b_inside) return;

	glm::vec2 n;
	float overlap = 0.0f;
	if (a_inside || b_inside || dist < 1e-5f) {
		// Push out along shortest local axis from midpoint of segment:
		glm::vec2 mid = 0.5f * (a + b);
		glm::vec2 local = world_to_body(body, mid);
		glm::vec2 ax, ay;
		body_axes(body, ax, ay);
		float dx = body.half_extents.x - std::abs(local.x);
		float dy = body.half_extents.y - std::abs(local.y);
		if (dx < dy) {
			n = (local.x >= 0.0f ? ax : -ax);
			overlap = dx + 0.02f;
		} else {
			n = (local.y >= 0.0f ? ay : -ay);
			overlap = dy + 0.02f;
		}
	} else {
		n = delta / dist;
		// Segment should push the box away (n from segment toward box):
		overlap = 0.02f; // grazing contact — use radius-style only when penetrating
		// If on_box is outside and distance is positive, no penetration unless we use a thickness.
		// Treat segment as having tiny thickness: collide when closest box point is near segment.
		if (dist > 0.05f) return;
		overlap = 0.05f - dist;
		if (overlap <= 0.0f) return;
	}

	body.position += n * overlap;
	float vn = glm::dot(body.velocity, n);
	if (vn < 0.0f) {
		body.velocity -= (1.0f + restitution) * vn * n;
	}
}

void resolve_box_wall(DynamicBody &body, Wall const &wall, float restitution) {
	resolve_box_segment(body, wall.a, wall.b, restitution);
}

void resolve_box_mirror(DynamicBody &body, Mirror &mirror, float restitution) {
	glm::vec2 a, b;
	mirror_endpoints(mirror, a, b);
	resolve_box_segment(body, a, b, restitution);
}

void clamp_body_in_world(DynamicBody &body, glm::vec2 mn, glm::vec2 mx) {
	float r = body.bounding_radius();
	if (body.position.x - r < mn.x) {
		body.position.x = mn.x + r;
		body.velocity.x = std::abs(body.velocity.x) * 0.4f;
	}
	if (body.position.x + r > mx.x) {
		body.position.x = mx.x - r;
		body.velocity.x = -std::abs(body.velocity.x) * 0.4f;
	}
	if (body.position.y - r < mn.y) {
		body.position.y = mn.y + r;
		body.velocity.y = std::abs(body.velocity.y) * 0.4f;
	}
	if (body.position.y + r > mx.y) {
		body.position.y = mx.y - r;
		body.velocity.y = -std::abs(body.velocity.y) * 0.4f;
	}
}

void apply_optics_and_forces(GameState &state, SimConstants const &constants, float dt) {
	std::vector< bool > body_lit(state.bodies.size(), false);
	std::vector< float > body_hit_energy(state.bodies.size(), 0.0f);
	std::vector< glm::vec2 > body_hit_normal(state.bodies.size(), glm::vec2(0.0f)); // outward face normal (toward ray)
	std::vector< LightColor > body_hit_color(state.bodies.size(), LightColor::Blue);

	for (auto &target : state.targets) {
		target.activated = false;
	}

	for (LaserSource const &laser : state.lasers) {
		if (!laser.active) continue;

		glm::vec2 pos = laser.position;
		glm::vec2 dir = glm::normalize(laser.direction);
		LightColor color = laser.color;
		float traveled = 0.0f;
		int last_mirror = -1;
		int last_body = -1;

		for (uint32_t bounce = 0; bounce < constants.max_bounces; ++bounce) {
			float remain = constants.max_beam_length - traveled;
			if (remain <= EPS) break;

			Hit hit = find_hit(state, pos, dir, last_mirror, last_body);
			if (hit.kind == HitKind::None || hit.t > remain) {
				break;
			}

			glm::vec2 hit_pos = pos + dir * hit.t;
			traveled += hit.t;
			float energy = color_energy(color);

			if (hit.kind == HitKind::Mirror) {
				dir = glm::normalize(dir - 2.0f * glm::dot(dir, hit.normal) * hit.normal);
				pos = hit_pos + dir * (EPS * 2.0f);
				last_mirror = hit.index;
				last_body = -1;
				continue;
			}

			if (hit.kind == HitKind::Gate) {
				Gate const &g = state.gates[hit.index];
				if (g.from_color == color) color = g.to_color;
				pos = hit_pos + dir * (EPS * 4.0f);
				last_mirror = -1;
				last_body = -1;
				continue;
			}

			if (hit.kind == HitKind::Body) {
				int bi = hit.index;
				DynamicBody const &body = state.bodies[bi];
				// Authored mirror face reflects; other faces charge (or block if launched).
				if (body_hit_is_mirror_face(body, hit.normal)) {
					dir = glm::normalize(dir - 2.0f * glm::dot(dir, hit.normal) * hit.normal);
					pos = hit_pos + dir * (EPS * 2.0f);
					last_body = bi;
					last_mirror = -1;
					continue;
				}
				if (body.launched) break;
				body_lit[bi] = true;
				if (energy >= body_hit_energy[bi]) {
					body_hit_energy[bi] = energy;
					body_hit_normal[bi] = hit.normal;
					body_hit_color[bi] = color;
				}
				break;
			}

			// Wall / bound: stop
			break;
		}
	}

	// Charge → exactly one impulse per crate for the whole level (then inert to light):
	for (size_t i = 0; i < state.bodies.size(); ++i) {
		DynamicBody &body = state.bodies[i];
		body.charged = false;
		if (body.launched) {
			// No more light charge/push, but stay dynamic so crate–crate hits still work.
			body.charge = 0.0f;
			if (glm::length(body.velocity) < 0.05f) {
				body.velocity = glm::vec2(0.0f);
				body.angular_velocity = 0.0f;
			}
			continue;
		}

		if (body_lit[i]) {
			body.excited_color = body_hit_color[i];
			body.charge = std::min(1.0f, body.charge + constants.charge_rate * body_hit_energy[i] * dt);
			if (body.charge >= 1.0f) {
				float E = body_hit_energy[i];
				// Shove perpendicular to the struck face (into the crate), ignore beam incidence angle.
				glm::vec2 n = body_hit_normal[i];
				float nlen = glm::length(n);
				glm::vec2 shove = (nlen > 1e-6f) ? (-n / nlen) : glm::vec2(1.0f, 0.0f);
				float along = glm::dot(body.velocity, shove);
				if (along < 0.0f) body.velocity -= along * shove;
				body.velocity += (constants.light_impulse_k * E * shove) / std::max(body.mass, 0.05f);
				body.angular_velocity *= 0.2f;
				body.charge = 0.0f;
				body.launched = true;
				body.charged = true;
			} else {
				body.velocity *= 0.5f;
				body.angular_velocity *= 0.5f;
			}
		} else {
			body.charge = std::max(0.0f, body.charge - constants.discharge_rate * dt);
		}
	}
}

} // namespace

void sync_mirror_to_track(Mirror &m);

std::vector< BeamSegment > trace_beams(GameState const &state, SimConstants const &constants) {
	std::vector< BeamSegment > segments;

	for (LaserSource const &laser : state.lasers) {
		if (!laser.active) continue;

		glm::vec2 pos = laser.position;
		glm::vec2 dir = glm::normalize(laser.direction);
		LightColor color = laser.color;
		float traveled = 0.0f;
		int last_mirror = -1;
		int last_body = -1;

		for (uint32_t bounce = 0; bounce < constants.max_bounces; ++bounce) {
			float remain = constants.max_beam_length - traveled;
			if (remain <= EPS) break;

			Hit hit = find_hit(state, pos, dir, last_mirror, last_body);
			glm::vec2 end;
			if (hit.kind == HitKind::None || hit.t > remain) {
				end = pos + dir * remain;
				segments.push_back({pos, end, color});
				break;
			}

			end = pos + dir * hit.t;
			segments.push_back({pos, end, color});
			traveled += hit.t;

			if (hit.kind == HitKind::Mirror) {
				dir = glm::normalize(dir - 2.0f * glm::dot(dir, hit.normal) * hit.normal);
				pos = end + dir * (EPS * 2.0f);
				last_mirror = hit.index;
				last_body = -1;
				continue;
			}
			if (hit.kind == HitKind::Gate) {
				Gate const &g = state.gates[hit.index];
				if (color == g.from_color) color = g.to_color;
				pos = end + dir * (EPS * 4.0f);
				last_mirror = -1;
				last_body = -1;
				continue;
			}
			if (hit.kind == HitKind::Body) {
				if (body_hit_is_mirror_face(state.bodies[hit.index], hit.normal)) {
					dir = glm::normalize(dir - 2.0f * glm::dot(dir, hit.normal) * hit.normal);
					pos = end + dir * (EPS * 2.0f);
					last_body = hit.index;
					last_mirror = -1;
					continue;
				}
				break; // non-mirror faces block
			}
			break;
		}
	}
	return segments;
}

void sim_step(GameState &state, SimConstants const &constants, float dt) {
	apply_optics_and_forces(state, constants, dt);

	// Integrate bodies:
	for (DynamicBody &body : state.bodies) {
		body.position += body.velocity * dt;
		body.angle += body.angular_velocity * dt;
		body.velocity *= constants.linear_damping;
		body.angular_velocity *= constants.angular_damping;
		clamp_body_in_world(body, state.world_min, state.world_max);
	}

	for (Mirror &m : state.mirrors) {
		if (m.mode == MirrorMode::Slide) sync_mirror_to_track(m);
	}

	// Collisions:
	std::vector< Wall > bounds;
	add_bounds_as_walls(state, bounds);

	for (int pass = 0; pass < 4; ++pass) {
		for (size_t i = 0; i < state.bodies.size(); ++i) {
			for (size_t j = i + 1; j < state.bodies.size(); ++j) {
				resolve_boxes(state.bodies[i], state.bodies[j], 0.55f);
			}
		}
	}
	for (DynamicBody &body : state.bodies) {
		for (Wall const &w : state.walls) resolve_box_wall(body, w, constants.restitution);
		for (Wall const &w : bounds) resolve_box_wall(body, w, constants.restitution);
		for (Mirror &m : state.mirrors) resolve_box_mirror(body, m, constants.restitution);
	}

	for (Target &t : state.targets) {
		for (DynamicBody const &body : state.bodies) {
			if (body_circle_overlap_distance(body, t.position, t.radius) <= 0.0f) {
				t.activated = true;
				break;
			}
		}
	}

	// Laser switches: crate overlap enables a secondary beam.
	for (int si = 0; si < (int)state.switches.size(); ++si) {
		LaserSwitch &sw = state.switches[si];
		sw.pressed = false;
		for (DynamicBody const &body : state.bodies) {
			if (body_circle_overlap_distance(body, sw.position, sw.radius) <= 0.0f) {
				sw.pressed = true;
				break;
			}
		}
		if (sw.pressed) sw.latched = true;

		if (sw.laser_index >= 0 && sw.laser_index < (int)state.lasers.size()) {
			LaserSource &laser = state.lasers[sw.laser_index];
			laser.active = sw.latched;
			laser.controlled_by_switch = si;
		}
	}

	state.won = false;
	if (!state.targets.empty()) {
		for (Target const &t : state.targets) {
			if (t.activated) {
				state.won = true;
				break;
			}
		}
	}
}

DynamicBody make_crate(glm::vec2 position, float half_size, float mass) {
	DynamicBody b;
	b.position = position;
	b.mass = mass;
	b.half_extents = glm::vec2(half_size);
	// Rectangle inertia about center: m*(w^2+h^2)/12 with full width = 2*half
	float w = 2.0f * half_size;
	b.inertia = mass * (w * w + w * w) / 12.0f;
	return b;
}

void attach_crate_mirror(DynamicBody &body, glm::vec2 local_outward) {
	if (std::abs(local_outward.x) >= std::abs(local_outward.y)) {
		body.mirror_face_local = glm::vec2(local_outward.x >= 0.0f ? 1.0f : -1.0f, 0.0f);
	} else {
		body.mirror_face_local = glm::vec2(0.0f, local_outward.y >= 0.0f ? 1.0f : -1.0f);
	}
	body.has_mirror_face = true;
}

void sync_mirror_to_track(Mirror &m) {
	glm::vec2 d = m.track_b - m.track_a;
	float len2 = glm::dot(d, d);
	if (len2 < 1e-8f) {
		m.position = m.track_a;
		m.track_t = 0.0f;
		return;
	}
	m.track_t = glm::clamp(m.track_t, 0.0f, 1.0f);
	m.position = m.track_a + m.track_t * d;
}

Mirror make_rotate_mirror(glm::vec2 position, float angle, float length) {
	Mirror m;
	m.position = position;
	m.angle = angle;
	m.length = length;
	m.mode = MirrorMode::Rotate;
	m.track_a = position;
	m.track_b = position;
	m.track_t = 0.0f;
	return m;
}

Mirror make_slide_mirror(glm::vec2 track_a, glm::vec2 track_b, float track_t, float angle, float length) {
	Mirror m;
	m.angle = angle;
	m.length = length;
	m.mode = MirrorMode::Slide;
	m.track_a = track_a;
	m.track_b = track_b;
	m.track_t = glm::clamp(track_t, 0.0f, 1.0f);
	sync_mirror_to_track(m);
	return m;
}

void rotate_selected_mirror(GameState &state, int mirror_index, float delta_angle) {
	if (mirror_index < 0 || mirror_index >= (int)state.mirrors.size()) return;
	Mirror &m = state.mirrors[mirror_index];
	if (m.mode != MirrorMode::Rotate) return;
	m.angle += delta_angle;
}

void slide_selected_mirror(GameState &state, int mirror_index, float delta_along_track) {
	if (mirror_index < 0 || mirror_index >= (int)state.mirrors.size()) return;
	Mirror &m = state.mirrors[mirror_index];
	if (m.mode != MirrorMode::Slide) return;
	float track_len = glm::length(m.track_b - m.track_a);
	if (track_len < 1e-4f) return;
	m.track_t = glm::clamp(m.track_t + delta_along_track / track_len, 0.0f, 1.0f);
	sync_mirror_to_track(m);
}

void reset_level(GameState &state, GameState const &initial) {
	state = initial;
}

std::vector< LevelInfo > make_levels() {
	std::vector< LevelInfo > levels;

	{
		LevelInfo L;
		L.name = "Light to Motion";
		GameState &s = L.initial;
		s.lasers.push_back({glm::vec2(-4.0f, 0.0f), glm::vec2(1.0f, 0.0f), LightColor::Red, false, -1});
		s.bodies.push_back(make_crate(glm::vec2(-0.5f, 0.0f), 0.4f, 0.8f));
		s.targets.push_back({glm::vec2(1.8f, 0.0f), 0.50f});
		levels.push_back(std::move(L));
	}

	{
		LevelInfo L;
		L.name = "Triple Hit";
		GameState &s = L.initial;
		s.lasers.push_back({glm::vec2(-6.5f, 0.0f), glm::vec2(1.0f, 0.0f), LightColor::Violet, false, -1});
		s.bodies.push_back(make_crate(glm::vec2(-0.4f, 0.0f), 0.4f, 0.55f)); // A on the beam
		s.bodies.push_back(make_crate(glm::vec2(1.2f, 0.0f), 0.4f, 0.75f)); // B
		s.bodies.push_back(make_crate(glm::vec2(2.8f, 0.0f), 0.4f, 0.70f));  // C → pad
		s.targets.push_back({glm::vec2(4.9f, 0.0f), 0.50f});
		levels.push_back(std::move(L));
	}

	{
		LevelInfo L;
		L.name = "Mirrors";
		GameState &s = L.initial;
		s.lasers.push_back({glm::vec2(-6.5f, 0.0f), glm::vec2(1.0f, 0.0f), LightColor::Red, false, -1});
		s.mirrors.push_back(make_rotate_mirror(glm::vec2(0.0f, 0.0f), 0.1f, 2.6f));
		s.bodies.push_back(make_crate(glm::vec2(0.0f, 1.6f), 0.4f, 0.8f));
		s.targets.push_back({glm::vec2(0.0f, 3.8f), 0.50f});
		levels.push_back(std::move(L));
	}

	{
		LevelInfo L;
		L.name = "Color Energy";
		GameState &s = L.initial;
		s.lasers.push_back({glm::vec2(-5.0f, 0.0f), glm::vec2(1.0f, 0.0f), LightColor::Red, false, -1});
		s.gates.push_back({glm::vec2(-3.5f, 0.0f), glm::vec2(0.45f, 0.7f), LightColor::Red, LightColor::Blue, PI * 0.5f});
		s.bodies.push_back(make_crate(glm::vec2(0.5f, 0.0f), 0.4f, 0.85f));
		s.targets.push_back({glm::vec2(4.0f, 0.0f), 0.50f});
		levels.push_back(std::move(L));
	}

	{
		LevelInfo L;
		L.name = "Beam Switch";
		GameState &s = L.initial;
		s.lasers.push_back({glm::vec2(-5.5f, -2.5f), glm::vec2(1.0f, 0.0f), LightColor::Blue, false, -1});
		s.lasers.push_back({glm::vec2(-5.5f, 2.5f), glm::vec2(1.0f, 0.0f), LightColor::Violet, false, 0});
		s.switches.push_back({glm::vec2(3.5f, -2.5f), 0.50f, 1, false, false});
		s.bodies.push_back(make_crate(glm::vec2(-0.5f, -2.5f), 0.4f, 0.8f));
		s.bodies.push_back(make_crate(glm::vec2(-0.5f, 2.5f), 0.4f, 0.8f));
		s.targets.push_back({glm::vec2(4.0f, 2.5f), 0.50f});
		levels.push_back(std::move(L));
	}

	{
		LevelInfo L;
		L.name = "Color Door";
		GameState &s = L.initial;
		s.lasers.push_back({glm::vec2(-6.5f, -2.0f), glm::vec2(1.0f, 0.0f), LightColor::Red, false, -1});
		s.mirrors.push_back(make_slide_mirror(
			glm::vec2(-3.5f, 0.5f), glm::vec2(-3.5f, -3.5f), 0.5f, PI * 0.25f, 2.4f));
		s.gates.push_back({glm::vec2(-1.5f, -2.0f), glm::vec2(0.4f, 0.75f), LightColor::Red, LightColor::Blue, PI * 0.5f});
		s.mirrors.push_back(make_slide_mirror(
			glm::vec2(0.0f, 0.5f), glm::vec2(0.0f, -3.5f), 0.5f, PI * 0.25f, 2.4f));
		s.mirrors.push_back(make_slide_mirror(
			glm::vec2(-4.5f, 2.5f), glm::vec2(1.0f, 2.5f), 0.5f, PI * 0.25f, 2.4f));
		s.bodies.push_back(make_crate(glm::vec2(2.2f, 2.5f), 0.4f, 0.85f));
		s.targets.push_back({glm::vec2(5.8f, 2.5f), 0.50f});
		levels.push_back(std::move(L));
	}

	{
		LevelInfo L;
		L.name = "Dual Rails";
		GameState &s = L.initial;
		s.lasers.push_back({glm::vec2(-6.5f, 3.6f), glm::vec2(1.0f, 0.0f), LightColor::Green, false, -1});
		s.mirrors.push_back(make_slide_mirror(
			glm::vec2(-0.5f, 3.6f), glm::vec2(5.0f, 3.6f), 0.40f, -PI * 0.25f, 2.2f));
		{
			DynamicBody a = make_crate(glm::vec2(0.0f, 2.2f), 0.4f, 0.8f);
			attach_crate_mirror(a, glm::vec2(1.0f, 0.0f));
			s.bodies.push_back(a);
		}
		s.mirrors.push_back(make_slide_mirror(
			glm::vec2(4.5f, 1.0f), glm::vec2(4.5f, -1.0f), 0.55f, PI / 3.0f, 2.4f));
		s.bodies.push_back(make_crate(glm::vec2(3.0f, -2.2f), 0.4f, 0.8f));
		s.targets.push_back({glm::vec2(6.2f, -2.2f), 0.50f});
		levels.push_back(std::move(L));
	}

	{
		LevelInfo L;
		L.name = "Lightbound";
		GameState &s = L.initial;
		s.lasers.push_back({glm::vec2(-6.5f, 0.0f), glm::vec2(1.0f, 0.0f), LightColor::Green, false, -1});
		s.lasers.push_back({glm::vec2(-6.5f, 3.4f), glm::vec2(1.0f, 0.0f), LightColor::Blue, false, 0});
		s.mirrors.push_back(make_slide_mirror(
			glm::vec2(-2.0f, 1.0f), glm::vec2(-2.0f, -3.5f), 0.22f, PI * 0.25f, 2.4f));
		s.mirrors.push_back(make_slide_mirror(
			glm::vec2(0.8f, 3.4f), glm::vec2(6.4f, 3.4f), 0.22f, -PI * 0.25f, 2.4f));
		s.bodies.push_back(make_crate(glm::vec2(0.2f, 0.0f), 0.4f, 0.8f));
		s.bodies.push_back(make_crate(glm::vec2(3.6f, 1.9f), 0.4f, 0.8f));
		s.switches.push_back({glm::vec2(3.6f, 0.0f), 0.50f, 1, false, false});
		s.targets.push_back({glm::vec2(3.6f, -2.2f), 0.50f});
		levels.push_back(std::move(L));
	}

	return levels;
}
