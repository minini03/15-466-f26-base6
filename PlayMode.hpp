#pragma once

#include "Mode.hpp"
#include "Sim.hpp"

#include <glm/glm.hpp>

#include <deque>
#include <string>
#include <vector>

struct PlayMode : Mode {
	PlayMode();
	virtual ~PlayMode();

	virtual bool handle_event(SDL_Event const &, glm::uvec2 const &window_size) override;
	virtual void update(float elapsed) override;
	virtual void draw(glm::uvec2 const &drawable_size) override;

	struct Button {
		uint8_t downs = 0;
		uint8_t pressed = 0;
	} left, right, back, forward, rot_ccw, rot_cw, rewind_btn, reset_btn, laser_btn, next_btn, prev_btn, select_btn;

	static constexpr float TickDt = 1.0f / 60.0f;
	static constexpr size_t MaxHistory = 60 * 45;
	static constexpr float BeamHeight = 0.35f;
	static constexpr float WallHeight = 1.0f;

	SimConstants constants;
	std::vector< LevelInfo > levels;
	int level_index = 0;
	GameState state;
	GameState level_initial;
	std::deque< GameState > history;

	float accumulator = 0.0f;
	bool rewinding = false;
	int selected_mirror = 0;

	// 3D view (Y-up). Sim (x,y) → world (x, 0, y).
	float camera_yaw = 0.6f;
	float camera_pitch = 0.85f;
	float camera_distance = 16.0f;
	bool orbiting = false;

	void load_level(int index);
	void push_history();
	void do_rewind_step();

	static glm::vec3 to_world(glm::vec2 p, float height = 0.0f) {
		return glm::vec3(p.x, height, p.y);
	}

	glm::vec3 board_center() const;
	glm::mat4 world_to_clip(glm::uvec2 const &drawable_size) const;

	// Draw a named mesh from lightbound.pnct with an object→world transform:
	void draw_mesh(std::string const &name, glm::mat4x3 const &world_from_object, glm::mat4 const &clip_from_world, glm::vec4 tint = glm::vec4(1.0f)) const;
};
