#pragma once

/*
 * Lightbound simulation: lasers, mirrors, color gates, 2D rigid bodies.
 * Fixed-timestep friendly — all state lives in GameState for rewind.
 */

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

enum class LightColor : uint8_t {
	Red = 0,
	Green = 1,
	Blue = 2,
	Violet = 3
};

inline float color_energy(LightColor c) {
	// Relative photon energy E ∝ 1/λ, normalized so Red = 1.
	// Representative wavelengths: R 700, G 550, B 450, V 400 nm.
	switch (c) {
	case LightColor::Red: return 1.00f;   // 700/700
	case LightColor::Green: return 1.27f;  // 700/550
	case LightColor::Blue: return 1.56f;   // 700/450
	case LightColor::Violet: return 1.75f; // 700/400
	default: return 1.0f;
	}
}

inline glm::u8vec4 color_rgba(LightColor c, uint8_t a = 0xff) {
	switch (c) {
	case LightColor::Red: return glm::u8vec4(0xe0, 0x40, 0x40, a);
	case LightColor::Green: return glm::u8vec4(0x40, 0xd0, 0x50, a);
	case LightColor::Blue: return glm::u8vec4(0x40, 0x90, 0xf0, a);
	case LightColor::Violet: return glm::u8vec4(0xb0, 0x50, 0xe0, a);
	default: return glm::u8vec4(0xff, 0xff, 0xff, a);
	}
}

struct LaserSource {
	glm::vec2 position = glm::vec2(0.0f);
	glm::vec2 direction = glm::vec2(1.0f, 0.0f);
	LightColor color = LightColor::Red;
	bool active = false; // Space starts the beam; off until the player presses start
	// If >= 0, this laser is driven by switches[controlled_by_switch] (Space won't toggle it).
	int controlled_by_switch = -1;
};

// Ball pressure switch that enables another laser beam.
struct LaserSwitch {
	glm::vec2 position = glm::vec2(0.0f);
	float radius = 0.50f;
	int laser_index = 0; // which laser this switch turns on
	bool pressed = false;
	bool latched = false;
};

// Player mirrors have exactly one degree of freedom:
//   Rotate — fixed position, Q/E changes angle
//   Slide  — fixed angle, WASD moves along a track
enum class MirrorMode : uint8_t {
	Rotate = 0,
	Slide = 1
};

struct Mirror {
	glm::vec2 position = glm::vec2(0.0f);
	float angle = 0.0f; // radians; mirror tangent along (cos,sin)
	float length = 1.5f;
	MirrorMode mode = MirrorMode::Rotate;

	// Slide track (sim plane). Position is always projected onto this segment.
	glm::vec2 track_a = glm::vec2(0.0f);
	glm::vec2 track_b = glm::vec2(0.0f);
	float track_t = 0.5f; // 0..1 along track_a → track_b
};

// Helpers for level authoring:
Mirror make_rotate_mirror(glm::vec2 position, float angle, float length = 2.0f);
Mirror make_slide_mirror(glm::vec2 track_a, glm::vec2 track_b, float track_t, float angle, float length = 2.0f);

struct Gate {
	glm::vec2 position = glm::vec2(0.0f);
	glm::vec2 half_extents = glm::vec2(0.35f, 0.8f); // local size (before angle)
	LightColor from_color = LightColor::Red;
	LightColor to_color = LightColor::Blue;
	float angle = 0.0f; // sim-plane rotation (radians); meshes yaw by -angle
};

struct DynamicBody {
	glm::vec2 position = glm::vec2(0.0f);
	glm::vec2 velocity = glm::vec2(0.0f);
	float angle = 0.0f;
	float angular_velocity = 0.0f;
	float mass = 1.0f;
	float inertia = 0.5f;
	glm::vec2 half_extents = glm::vec2(0.4f); // OBB half-size in local XY (sim plane)

	// Light charges the crate; at full charge ONE impulse fires (no second push).
	float charge = 0.0f; // 0..1
	bool charged = false; // true briefly / for VFX on the launch frame
	bool launched = false; // once true: never charged/pushed again
	LightColor excited_color = LightColor::Blue; // color of the beam that last charged it

	// Optional authored mirror on one face (set via attach_crate_mirror).
	// Local outward axis: (±1,0) or (0,±1). Only that face reflects.
	bool has_mirror_face = false;
	glm::vec2 mirror_face_local = glm::vec2(0.0f);

	float bounding_radius() const {
		return glm::length(half_extents);
	}
};

// Axis-aligned square crate (optionally already rotated):
DynamicBody make_crate(glm::vec2 position, float half_size = 0.4f, float mass = 1.0f);
// Stick a mirror on one local face. local_outward snaps to nearest ±X / ±Y
// (e.g. (-1,0)=left, (1,0)=right, (0,1)=+simY, (0,-1)=-simY when angle=0).
void attach_crate_mirror(DynamicBody &body, glm::vec2 local_outward);

struct Wall {
	glm::vec2 a = glm::vec2(0.0f);
	glm::vec2 b = glm::vec2(1.0f, 0.0f);
};

struct Target {
	glm::vec2 position = glm::vec2(0.0f);
	float radius = 0.45f;
	bool activated = false;
};

struct BeamSegment {
	glm::vec2 a = glm::vec2(0.0f);
	glm::vec2 b = glm::vec2(0.0f);
	LightColor color = LightColor::Red;
};

struct GameState {
	std::vector< LaserSource > lasers;
	std::vector< LaserSwitch > switches;
	std::vector< Mirror > mirrors;
	std::vector< Gate > gates;
	std::vector< DynamicBody > bodies;
	std::vector< Wall > walls;
	std::vector< Target > targets;

	glm::vec2 world_min = glm::vec2(-8.0f, -5.0f);
	glm::vec2 world_max = glm::vec2(8.0f, 5.0f);

	bool won = false;
};

struct SimConstants {
	float light_impulse_k = 1.0f;  // one-shot Δv scale: v += (k * E * face_normal) / mass
	float charge_rate = 1.2f;      // ~0.8s full charge with Red (E≈1)
	float discharge_rate = 0.5f;   // charge drain per second when not hit (pre-launch only)
	float linear_damping = 0.992f; // coast after the single shove
	float angular_damping = 0.98f;
	float restitution = 0.15f;     // low so crates don't ping-pong off walls
	float max_beam_length = 40.0f;
	uint32_t max_bounces = 24;
};

struct LevelInfo {
	std::string name;
	GameState initial;
};

// Build built-in progressive levels:
std::vector< LevelInfo > make_levels();

// One fixed physics + optics step:
void sim_step(GameState &state, SimConstants const &constants, float dt);

// Trace lasers for rendering (does not mutate forces — call after step or use last frame beams):
std::vector< BeamSegment > trace_beams(GameState const &state, SimConstants const &constants);

// Apply player mirror controls (one DOF each):
void rotate_selected_mirror(GameState &state, int mirror_index, float delta_angle);
void slide_selected_mirror(GameState &state, int mirror_index, float delta_along_track);
void reset_level(GameState &state, GameState const &initial);
