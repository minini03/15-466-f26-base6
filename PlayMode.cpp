#include "PlayMode.hpp"

#include "DrawLines.hpp"
#include "LitColorTextureProgram.hpp"
#include "Load.hpp"
#include "Mesh.hpp"
#include "data_path.hpp"
#include "gl_errors.hpp"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {
GLuint lightbound_meshes_for_lit = 0;
Load< MeshBuffer > lightbound_meshes(LoadTagDefault, []() -> MeshBuffer const * {
	MeshBuffer const *ret = new MeshBuffer(data_path("lightbound.pnct"));
	lightbound_meshes_for_lit = ret->make_vao_for_program(lit_color_texture_program->program);
	return ret;
});

glm::mat4x3 make_xf(glm::vec3 pos, glm::quat rot, glm::vec3 scale) {
	glm::mat3 R = glm::mat3_cast(rot);
	glm::mat4x3 m(1.0f);
	m[0] = R[0] * scale.x;
	m[1] = R[1] * scale.y;
	m[2] = R[2] * scale.z;
	m[3] = pos;
	return m;
}
} // namespace

PlayMode::PlayMode() {
	levels = make_levels();
	load_level(0);
}

PlayMode::~PlayMode() = default;

void PlayMode::load_level(int index) {
	if (levels.empty()) return;
	level_index = (index % (int)levels.size() + (int)levels.size()) % (int)levels.size();
	level_initial = levels[level_index].initial;
	state = level_initial;
	history.clear();
	accumulator = 0.0f;
	rewinding = false;
	selected_mirror = 0;
}

void PlayMode::push_history() {
	history.push_back(state);
	while (history.size() > MaxHistory) history.pop_front();
}

void PlayMode::do_rewind_step() {
	if (history.empty()) return;
	state = history.back();
	history.pop_back();
}

glm::vec3 PlayMode::board_center() const {
	glm::vec2 c = 0.5f * (state.world_min + state.world_max);
	return to_world(c, 0.0f);
}

glm::mat4 PlayMode::world_to_clip(glm::uvec2 const &drawable_size) const {
	float aspect = float(drawable_size.x) / float(std::max(drawable_size.y, 1u));
	glm::vec3 target = board_center();
	float pitch = glm::clamp(camera_pitch, 0.15f, 1.4f);
	glm::vec3 offset(
		camera_distance * std::cos(pitch) * std::sin(camera_yaw),
		camera_distance * std::sin(pitch),
		camera_distance * std::cos(pitch) * std::cos(camera_yaw)
	);
	glm::vec3 eye = target + offset;
	glm::mat4 world_to_camera = glm::lookAt(eye, target, glm::vec3(0.0f, 1.0f, 0.0f));
	glm::mat4 camera_to_clip = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 200.0f);
	return camera_to_clip * world_to_camera;
}

void PlayMode::draw_mesh(std::string const &name, glm::mat4x3 const &world_from_object, glm::mat4 const &clip_from_world, glm::vec4 tint) const {
	Mesh const &mesh = lightbound_meshes->lookup(name);

	glUseProgram(lit_color_texture_program->program);
	glBindVertexArray(lightbound_meshes_for_lit);

	glm::mat4 clip_from_object = clip_from_world * glm::mat4(world_from_object);
	glUniformMatrix4fv(lit_color_texture_program->CLIP_FROM_OBJECT_mat4, 1, GL_FALSE, glm::value_ptr(clip_from_object));

	glm::mat4x3 light_from_object = world_from_object; // light space == world
	glUniformMatrix4x3fv(lit_color_texture_program->LIGHT_FROM_OBJECT_mat4x3, 1, GL_FALSE, glm::value_ptr(light_from_object));

	glm::mat3 light_from_normal = glm::inverse(glm::transpose(glm::mat3(light_from_object)));
	glUniformMatrix3fv(lit_color_texture_program->LIGHT_FROM_NORMAL_mat3, 1, GL_FALSE, glm::value_ptr(light_from_normal));

	glUniform4fv(lit_color_texture_program->COLOR_MUL_vec4, 1, glm::value_ptr(tint));

	if (lit_color_texture_program_pipeline.textures[0].texture != 0) {
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(lit_color_texture_program_pipeline.textures[0].target, lit_color_texture_program_pipeline.textures[0].texture);
	}

	glDrawArrays(mesh.type, mesh.start, mesh.count);

	glUniform4f(lit_color_texture_program->COLOR_MUL_vec4, 1.0f, 1.0f, 1.0f, 1.0f);
	glBindTexture(GL_TEXTURE_2D, 0);
	glBindVertexArray(0);
	glUseProgram(0);
}

bool PlayMode::handle_event(SDL_Event const &evt, glm::uvec2 const &window_size) {
	auto set_key = [&](Button &b, bool down) {
		if (down) {
			b.downs += 1;
			b.pressed = 1;
		} else {
			b.pressed = 0;
		}
	};

	if (evt.type == SDL_EVENT_KEY_DOWN || evt.type == SDL_EVENT_KEY_UP) {
		bool down = (evt.type == SDL_EVENT_KEY_DOWN);
		switch (evt.key.key) {
		case SDLK_A: set_key(left, down); return true;
		case SDLK_D: set_key(right, down); return true;
		case SDLK_W: set_key(forward, down); return true;
		case SDLK_S: set_key(back, down); return true;
		case SDLK_Q: set_key(rot_ccw, down); return true;
		case SDLK_E: set_key(rot_cw, down); return true;
		case SDLK_R:
			if (down && !(evt.key.mod & SDL_KMOD_SHIFT)) {
				rewind_btn.downs += 1;
				rewind_btn.pressed = 1;
			} else if (!down) {
				rewind_btn.pressed = 0;
			}
			if (down && (evt.key.mod & SDL_KMOD_SHIFT)) {
				reset_btn.downs += 1;
				reset_btn.pressed = 1;
			}
			return true;
		case SDLK_BACKSPACE: set_key(reset_btn, down); return true;
		case SDLK_SPACE: set_key(laser_btn, down); return true;
		case SDLK_N: set_key(next_btn, down); return true;
		case SDLK_P: set_key(prev_btn, down); return true;
		case SDLK_TAB: set_key(select_btn, down); return true;
		default: break;
		}
	} else if (evt.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
		if (evt.button.button == SDL_BUTTON_RIGHT) {
			orbiting = true;
			SDL_SetWindowRelativeMouseMode(Mode::window, true);
			return true;
		}
	} else if (evt.type == SDL_EVENT_MOUSE_BUTTON_UP) {
		if (evt.button.button == SDL_BUTTON_RIGHT) {
			orbiting = false;
			SDL_SetWindowRelativeMouseMode(Mode::window, false);
			return true;
		}
	} else if (evt.type == SDL_EVENT_MOUSE_MOTION) {
		if (orbiting) {
			glm::vec2 motion(
				evt.motion.xrel / float(std::max(window_size.y, 1u)),
				-evt.motion.yrel / float(std::max(window_size.y, 1u))
			);
			camera_yaw += motion.x * 2.5f;
			camera_pitch = glm::clamp(camera_pitch + motion.y * 2.5f, 0.15f, 1.4f);
			return true;
		}
	} else if (evt.type == SDL_EVENT_MOUSE_WHEEL) {
		camera_distance = glm::clamp(camera_distance - evt.wheel.y * 1.2f, 6.0f, 40.0f);
		return true;
	}
	return false;
}

void PlayMode::update(float elapsed) {
	if (reset_btn.downs) {
		reset_level(state, level_initial);
		history.clear();
		accumulator = 0.0f;
	}
	if (next_btn.downs) load_level(level_index + 1);
	if (prev_btn.downs) load_level(level_index - 1);

	if (select_btn.downs && !state.mirrors.empty()) {
		selected_mirror = (selected_mirror + 1) % (int)state.mirrors.size();
	}

	if (laser_btn.downs) {
		// Only toggle lasers that are not driven by a ball switch.
		for (LaserSource &laser : state.lasers) {
			if (laser.controlled_by_switch < 0) laser.active = !laser.active;
		}
	}

	rewinding = rewind_btn.pressed;

	accumulator += elapsed;
	accumulator = std::min(accumulator, TickDt * 8.0f);

	while (accumulator >= TickDt) {
		if (rewinding) {
			do_rewind_step();
		} else {
			{
				// One DOF per mirror: Slide uses A/D (or WASD projected on track); Rotate uses Q/E only.
				if (selected_mirror >= 0 && selected_mirror < (int)state.mirrors.size()) {
					Mirror const &sel = state.mirrors[selected_mirror];
					if (sel.mode == MirrorMode::Slide) {
						float slide = 0.0f;
						float speed = 3.5f * TickDt;
						glm::vec2 track_dir = sel.track_b - sel.track_a;
						float track_len = glm::length(track_dir);
						if (track_len > 1e-4f) {
							track_dir /= track_len;
							// A/D along +track; W/S also project if useful for vertical rails:
							glm::vec2 wish(0.0f);
							if (left.pressed) wish.x -= 1.0f;
							if (right.pressed) wish.x += 1.0f;
							// Screen-forward is -sim Y (world -Z with default camera).
							if (forward.pressed) wish.y -= 1.0f;
							if (back.pressed) wish.y += 1.0f;
							if (wish != glm::vec2(0.0f)) {
								wish = glm::normalize(wish);
								slide = glm::dot(wish, track_dir) * speed;
							}
						}
						if (slide != 0.0f) slide_selected_mirror(state, selected_mirror, slide);
					} else if (sel.mode == MirrorMode::Rotate) {
						float rot_speed = 1.8f * TickDt;
						if (rot_ccw.pressed) rotate_selected_mirror(state, selected_mirror, rot_speed);
						if (rot_cw.pressed) rotate_selected_mirror(state, selected_mirror, -rot_speed);
					}
				}
			}
			push_history();
			sim_step(state, constants, TickDt);
		}
		accumulator -= TickDt;
	}

	left.downs = right.downs = back.downs = forward.downs = 0;
	rot_ccw.downs = rot_cw.downs = 0;
	rewind_btn.downs = reset_btn.downs = laser_btn.downs = 0;
	next_btn.downs = prev_btn.downs = select_btn.downs = 0;
}

void PlayMode::draw(glm::uvec2 const &drawable_size) {
	glClearColor(0.10f, 0.12f, 0.16f, 1.0f);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glm::mat4 clip_from_world = world_to_clip(drawable_size);

	// Lighting for lit meshes:
	glUseProgram(lit_color_texture_program->program);
	glUniform1i(lit_color_texture_program->LIGHT_TYPE_int, 1); // directional
	glUniform3fv(lit_color_texture_program->LIGHT_DIRECTION_vec3, 1, glm::value_ptr(glm::normalize(glm::vec3(0.35f, -1.0f, 0.25f))));
	glUniform3fv(lit_color_texture_program->LIGHT_ENERGY_vec3, 1, glm::value_ptr(glm::vec3(1.0f, 0.98f, 0.92f)));
	glUseProgram(0);

	glm::vec2 mn = state.world_min;
	glm::vec2 mx = state.world_max;
	glm::vec2 board_size = mx - mn;
	glm::vec2 board_mid = 0.5f * (mn + mx);

	// Floor:
	draw_mesh("Floor", make_xf(
		to_world(board_mid, 0.0f),
		glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
		glm::vec3(board_size.x, 1.0f, board_size.y)
	), clip_from_world);

	// Border walls (4 sides):
	{
		struct Seg { glm::vec2 a, b; };
		Seg borders[4] = {
			{glm::vec2(mn.x, mn.y), glm::vec2(mx.x, mn.y)},
			{glm::vec2(mx.x, mn.y), glm::vec2(mx.x, mx.y)},
			{glm::vec2(mx.x, mx.y), glm::vec2(mn.x, mx.y)},
			{glm::vec2(mn.x, mx.y), glm::vec2(mn.x, mn.y)},
		};
		for (Seg const &s : borders) {
			glm::vec2 d = s.b - s.a;
			float len = glm::length(d);
			if (len < 1e-4f) continue;
			float ang = std::atan2(d.y, d.x);
			glm::quat rot = glm::angleAxis(-ang, glm::vec3(0.0f, 1.0f, 0.0f));
			glm::vec2 mid = 0.5f * (s.a + s.b);
			draw_mesh("Wall", make_xf(to_world(mid, 0.0f), rot, glm::vec3(len, WallHeight, 1.0f)), clip_from_world);
		}
	}

	for (Wall const &w : state.walls) {
		glm::vec2 d = w.b - w.a;
		float len = glm::length(d);
		if (len < 1e-4f) continue;
		float ang = std::atan2(d.y, d.x);
		glm::quat rot = glm::angleAxis(-ang, glm::vec3(0.0f, 1.0f, 0.0f));
		draw_mesh("Wall", make_xf(to_world(0.5f * (w.a + w.b), 0.0f), rot, glm::vec3(len, WallHeight, 1.0f)), clip_from_world);
	}

	for (Gate const &g : state.gates) {
		glm::quat rot = glm::angleAxis(-g.angle, glm::vec3(0.0f, 1.0f, 0.0f));
		glm::vec3 scale(g.half_extents.x * 2.0f, WallHeight * 0.85f, g.half_extents.y * 2.0f);
		// Tint by conversion / filter color so doors read their wavelength role.
		LightColor show = g.to_color;
		glm::u8vec4 c = color_rgba(show);
		glm::vec4 tint(float(c.r) / 255.0f, float(c.g) / 255.0f, float(c.b) / 255.0f, 1.0f);
		draw_mesh("Gate", make_xf(to_world(g.position, 0.0f), rot, scale), clip_from_world, tint);
	}

	for (Target const &t : state.targets) {
		float s = t.activated ? t.radius * 1.1f : t.radius;
		glm::vec4 tint = t.activated
			? glm::vec4(0.25f, 0.95f, 0.45f, 1.0f)
			: glm::vec4(0.95f, 0.30f, 0.28f, 1.0f);
		draw_mesh("Pad", make_xf(to_world(t.position, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(s)), clip_from_world, tint);
	}

	// Laser switches: same flat Pad size as win pads; tint = controlled laser color.
	for (LaserSwitch const &sw : state.switches) {
		bool on = sw.pressed || sw.latched;
		LightColor lc = LightColor::Red;
		if (sw.laser_index >= 0 && sw.laser_index < (int)state.lasers.size()) {
			lc = state.lasers[sw.laser_index].color;
		}
		glm::u8vec4 c = color_rgba(lc);
		float bright = on ? 1.0f : 0.45f;
		glm::vec4 tint(
			bright * float(c.r) / 255.0f,
			bright * float(c.g) / 255.0f,
			bright * float(c.b) / 255.0f,
			1.0f
		);
		float s = on ? sw.radius * 1.1f : sw.radius;
		draw_mesh("Pad", make_xf(to_world(sw.position, 0.0f), glm::quat(1.0f, 0.0f, 0.0f, 0.0f), glm::vec3(s)), clip_from_world, tint);
	}

	for (DynamicBody const &b : state.bodies) {
		glm::quat rot = glm::angleAxis(-b.angle, glm::vec3(0.0f, 1.0f, 0.0f));
		float boost = (b.charged || b.launched) ? 1.05f : 1.0f;
		glm::vec3 scale(b.half_extents.x * boost, b.half_extents.y * boost, b.half_extents.y * boost);
		// White idle → laser color while charging / after launch.
		float t = b.launched ? 1.0f : glm::clamp(b.charge, 0.0f, 1.0f);
		glm::u8vec4 c = color_rgba(b.excited_color);
		glm::vec4 tint(
			glm::mix(1.0f, float(c.r) / 255.0f, t),
			glm::mix(1.0f, float(c.g) / 255.0f, t),
			glm::mix(1.0f, float(c.b) / 255.0f, t),
			1.0f
		);
		draw_mesh("Box", make_xf(to_world(b.position, b.half_extents.y), rot, scale), clip_from_world, tint);

		if (b.has_mirror_face) {
			float ca = std::cos(b.angle);
			float sa = std::sin(b.angle);
			glm::vec2 ax(ca, sa), ay(-sa, ca);
			glm::vec2 n = ax * b.mirror_face_local.x + ay * b.mirror_face_local.y;
			float nlen = glm::length(n);
			if (nlen > 1e-6f) n /= nlen;
			bool face_x = std::abs(b.mirror_face_local.x) >= std::abs(b.mirror_face_local.y);
			float face_he = face_x ? b.half_extents.x : b.half_extents.y;
			float face_half_w = face_x ? b.half_extents.y : b.half_extents.x;
			glm::vec2 mpos = b.position + n * (face_he * boost + 0.03f);
			glm::vec2 tangent(-n.y, n.x);
			float mang = std::atan2(tangent.y, tangent.x);
			glm::quat mrot = glm::angleAxis(-mang, glm::vec3(0.0f, 1.0f, 0.0f));
			float ml = 2.0f * face_half_w * boost * 0.92f;
			float mh = 2.0f * b.half_extents.y * boost * 0.92f;
			draw_mesh("CrateMirror", make_xf(to_world(mpos, 0.0f), mrot, glm::vec3(ml, mh, 1.0f)), clip_from_world);
		}
	}

	for (int i = 0; i < (int)state.mirrors.size(); ++i) {
		Mirror const &m = state.mirrors[i];
		glm::quat rot = glm::angleAxis(-m.angle, glm::vec3(0.0f, 1.0f, 0.0f));
		float h = WallHeight * 0.9f;
		float thick = (i == selected_mirror) ? 1.35f : 1.0f;
		draw_mesh("Mirror", make_xf(to_world(m.position, 0.0f), rot, glm::vec3(m.length, h, thick)), clip_from_world);
	}

	for (LaserSource const &laser : state.lasers) {
		// Flashlight mesh is unit-ish length along +X; aim along laser direction.
		glm::vec2 dir = glm::normalize(laser.direction);
		float ang = std::atan2(dir.y, dir.x);
		glm::quat rot = glm::angleAxis(-ang, glm::vec3(0.0f, 1.0f, 0.0f));
		float s = laser.active ? 0.55f : 0.45f;
		glm::u8vec4 c = color_rgba(laser.color);
		glm::vec4 tint(
			(laser.active ? 1.0f : 0.45f) * float(c.r) / 255.0f,
			(laser.active ? 1.0f : 0.45f) * float(c.g) / 255.0f,
			(laser.active ? 1.0f : 0.45f) * float(c.b) / 255.0f,
			1.0f
		);
		draw_mesh("Emitter", make_xf(to_world(laser.position, BeamHeight), rot, glm::vec3(s)), clip_from_world, tint);
	}

	// Tracks + lasers as lines on top:
	{
		std::vector< BeamSegment > beams = trace_beams(state, constants);
		DrawLines lines(clip_from_world);

		for (int i = 0; i < (int)state.mirrors.size(); ++i) {
			Mirror const &m = state.mirrors[i];
			if (m.mode != MirrorMode::Slide) continue;
			glm::u8vec4 rail = (i == selected_mirror)
				? glm::u8vec4(0xff, 0xe0, 0x80, 0xff)
				: glm::u8vec4(0x80, 0x70, 0x40, 0xff);
			glm::vec3 a = to_world(m.track_a, 0.05f);
			glm::vec3 b = to_world(m.track_b, 0.05f);
			lines.draw(a, b, rail);
			lines.draw(a, a + glm::vec3(0.0f, 0.25f, 0.0f), rail);
			lines.draw(b, b + glm::vec3(0.0f, 0.25f, 0.0f), rail);
		}

		for (BeamSegment const &seg : beams) {
			lines.draw(to_world(seg.a, BeamHeight), to_world(seg.b, BeamHeight), color_rgba(seg.color));
			lines.draw(to_world(seg.a, BeamHeight + 0.02f), to_world(seg.b, BeamHeight + 0.02f), color_rgba(seg.color));
		}
	}

	glDisable(GL_DEPTH_TEST);
	{
		float aspect = float(drawable_size.x) / float(std::max(drawable_size.y, 1u));
		DrawLines hud(glm::mat4(
			1.0f / aspect, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		));

		auto text = [&](char const *str, glm::vec3 at, glm::u8vec4 col) {
			constexpr float H = 0.045f;
			hud.draw_text(str, at, glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f), glm::u8vec4(0x00, 0x00, 0x00, 0xff));
			hud.draw_text(str, at + glm::vec3(0.003f, 0.003f, 0.0f), glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f), col);
		};

		char line[128];
		std::snprintf(line, sizeof(line), "%d. %s",
			level_index + 1, levels[level_index].name.c_str());
		text(line, glm::vec3(-aspect + 0.05f, 0.92f, 0.0f), glm::u8vec4(0xff, 0xff, 0xff, 0xff));
		if (state.won) {
			constexpr float H = 0.09f;
			char const *clear = "CLEAR!";
			hud.draw_text(clear, glm::vec3(-0.28f, 0.02f, 0.0f), glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f), glm::u8vec4(0x00, 0x00, 0x00, 0xff));
			hud.draw_text(clear, glm::vec3(-0.277f, 0.023f, 0.0f), glm::vec3(H, 0.0f, 0.0f), glm::vec3(0.0f, H, 0.0f), glm::u8vec4(0x70, 0xff, 0x90, 0xff));
		}
	}

	GL_ERRORS();
}
