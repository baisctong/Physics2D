// physics2d_testbed - interactive testbed entry point (Week 1 D5/D6; contact
// overlay in Week 2; selectable scenes in Weeks 3-4; Week 5 adds the Solver and
// Contacts panels, the clickable contact highlight and the FrictionRamp scene; Week 6
// adds the broad-phase switch/default, auto camera framing and the Pyramid200 /
// StressBenchmark scenes; Week 7 adds the Dominoes / IslandSleep500 scenes, the
// island+sleep counters and controls, keyboard playback shortcuts and the contact
// impulse bars).
//
// Responsibilities (and nothing else):
//   1. Create the GLFW window / OpenGL 3.3 core context and initialise Dear ImGui
//      (docking branch) with the GLFW + OpenGL3 backends.
//   2. Run the frame loop: poll input -> draw UI -> run fixed physics steps ->
//      render world and UI -> swap.
//   3. Own the World, the demo registry and the camera; demos only see those.
//   4. Draw engine-level debug overlays (axes, AABBs, contact points/normals, the
//      highlighted contact and the optional velocity arrows) after the demo. They
//      live here because World owns that data and the Demo interface has no
//      display-option parameter, so every scene gets the same debug view from one
//      place.
//   5. Apply the DebugUI material defaults when a scene is built (Week 5 solver
//      experiments: e.g. friction 0 on BoxStack), install the broad phase (Week 6:
//      default SAP so smoke runs also go through the broad-phase path) and focus the
//      camera once per scene build (Week 6 scenes are bigger than the default view).
//      The solver settings themselves are written by DebugUI straight into
//      World::SetSolverSettings; the broad-phase dropdown writes World::SetBroadPhase.
//
// Fixed timestep: p2::StepAccumulator turns the variable frame time into a whole
// number of 1/60 s steps (Section 4.3), at most kMaxStepsPerFrame per frame to stop
// the "death spiral" where a slow frame schedules even more work. The leftover
// fraction is available as interpolation alpha but the testbed does not use it yet:
// the design document schedules render interpolation for Week 7, and demos would need
// previous-state caching that does not exist until then.
//
// CLI:
//   --demo NAME choose the initial scene (FallingBall | SphereStack | BoxStack |
//               FrictionRamp | Dominoes | IslandSleep500 | Pyramid200 |
//               StressBenchmark, default FallingBall). Matching is case-insensitive
//               and ignores spaces, underscores and hyphens, so "spherestack" and
//               "Sphere Stack" both work. An unknown name prints the available list
//               and exits with code 2 so smoke automation can distinguish a typo
//               from a physics failure.
//   --broadphase NAME  broad phase: None | BruteForce (bf) | SAP (default SAP). Same
//               loose name matching. Unknown values are a usage error (exit 1).
//   --smoke N   render N frames, print a machine-checkable summary, exit 0.
//               In smoke mode the loop feeds the accumulator a deterministic
//               1/60 s per frame, so N frames == N physics steps regardless of
//               machine speed. Exit code 2 means "no usable GL environment".
//   --hidden    create the window with GLFW_VISIBLE=false (used by smoke runs).
//   --capture-dir DIR   record the deterministic smoke frames as binary PPM (P6) files
//               in DIR (Week 8: the README GIFs). The framebuffer is read back AFTER
//               the world and the ImGui panels are drawn and BEFORE glfwSwapBuffers, so
//               the capture is exactly the finished window frame. Requires --smoke;
//               default off (no readback cost when the flag is absent).
//   --capture-every K   capture one frame every K rendered frames (default 1 when
//               --capture-dir is given). Must not be combined with a non-capture run.
//   --capture-start N   first rendered frame (1-based) that may be captured (default 1);
//               lets a recorder skip an intro without reimplementing the smoke loop.
//   --capture-hide-ui   skip the DebugUI panels while capturing, so the full window is
//               scene (used for the dominoes GIF: the floating panels would cover the
//               two ends of the 20 m chain). The simulation state and render path are
//               unchanged - only the ImGui pass is omitted.
//   --capture-display OPTS  comma-separated display overrides for the capture, e.g.
//               "contacts=off,normals=on" (keys: contacts, normals, aabb, axes,
//               velocities; values on/off). Same switches as the Solver Settings panel;
//               recording-only convenience so a GIF can show normals without a click.
//   --help      print usage.

#include "Camera.h"
#include "DebugUI.h"
#include "GLLoader.h"
#include "Renderer.h"
#include "demos/BoxStack.h"
#include "demos/Dominoes.h"
#include "demos/FallingBall.h"
#include "demos/FrictionRamp.h"
#include "demos/IslandSleep500.h"
#include "demos/Pyramid200.h"
#include "demos/SphereStack.h"
#include "demos/StressBenchmark.h"

#include "physics2d/Core/Settings.h"
#include "physics2d/Core/TimeStep.h"
#include "physics2d/Core/World.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {

constexpr int kExitSuccess = 0;
constexpr int kExitFailure = 1;
// Environment-unsupported is distinct from a failing test: the smoke harness uses
// it to say "this machine has no desktop/OpenGL 3.3", which is not a code bug.
constexpr int kExitEnvironmentUnsupported = 2;
// Unknown scene name shares code 2 by contract (task card): the smoke harness treats
// it as "the requested configuration does not exist here", not as a physics failure.
constexpr int kExitUnknownDemo = 2;

constexpr int kInitialWindowWidth = 1280;
constexpr int kInitialWindowHeight = 800;
constexpr const char* kWindowTitle = "Physics2D Testbed";
// GLSL 330 core matches the requested OpenGL 3.3 core context.
constexpr const char* kGlslVersion = "#version 330 core";

// Contact overlay styling. The normal arrow has a fixed length (0.5 m): scaling it
// by penetration would make the arrow vanish exactly when the contact is resting and
// would visually conflate two different quantities.
constexpr float kContactPointRadius = 0.07f;
constexpr float kContactNormalLength = 0.5f;
constexpr float kContactArrowHeadLength = 0.15f;
constexpr p2::Color kContactPointColor{0.95f, 0.25f, 0.25f, 1.0f};
constexpr p2::Color kContactNormalColor{1.00f, 0.85f, 0.20f, 1.0f};

// Contact highlighted in the DebugUI Contacts panel: bigger marker, longer arrow and
// a cyan no scene uses, so the clicked pair stays identifiable over any demo colour.
constexpr float kHighlightedContactPointRadius = 0.12f;
constexpr float kHighlightedContactNormalLength = 1.0f;
constexpr p2::Color kHighlightedContactColor{0.20f, 0.90f, 1.00f, 1.0f};

// Engine-level velocity overlay (DebugUI "Velocity arrows"): scenes that do not draw
// their own arrows can use it. A blue distinct from the demos' green on purpose - a
// doubled view must not look like one arrow.
constexpr float kVelocityArrowScale = 0.10f;
constexpr float kVelocityArrowThresholdSquared = 0.02f * 0.02f;
constexpr p2::Color kVelocityArrowColor{0.30f, 0.75f, 1.00f, 1.0f};

// AABB overlay cutoff: the infinite plane's proxy box is kPlaneAabbExtent on a side
// (1e6 m), while every finite scene here stays orders of magnitude below this. The
// overlay skips anything bigger so it never submits a 2000 km line to the renderer.
constexpr float kMaxDrawableAabbExtent = 1.0e5f;

struct CommandLineOptions {
    int smokeFrames = 0; // 0 means interactive mode
    bool hidden = false;
    bool helpRequested = false;
    std::string demoName = "FallingBall"; // scene id, matched against Demo::Name()
    std::string broadPhaseName = "Auto";  // resolved to a DebugUI::BroadPhaseMode in RunTestbed
    // Week 8 GIF recording (all default-off: interactive and plain smoke runs are
    // unchanged, and no capture code runs unless captureDirectory is non-empty).
    std::string captureDirectory; // empty = capture disabled
    int captureFrameInterval = 0; // --capture-every K; 0 = not given (defaults to 1)
    int captureStartFrame = 0;    // --capture-start N; 0 = not given (defaults to 1)
    bool captureHideUi = false;   // --capture-hide-ui: scene-only frames
    std::string captureDisplay;   // --capture-display overrides, e.g. "contacts=off,normals=on"
};

// The binary name is spelled out instead of using argv[0]: the repository path may
// contain non-ASCII characters, and printing argv[0] to a GBK/UTF-8 mixed console
// turns the usage line into mojibake.
void PrintUsage() {
    std::printf("Usage: physics2d_testbed [--demo NAME] [--broadphase NAME] [--smoke N] [--hidden] [--help]\n"
                "  --demo NAME start in this scene (FallingBall | SphereStack | BoxStack |\n"
                "              FrictionRamp | Dominoes | IslandSleep500 | Pyramid200 |\n"
                "              StressBenchmark; default FallingBall).\n"
                "              Matching is case-insensitive and ignores spaces/underscores.\n"
                "  --broadphase NAME  broad phase: Auto | None | BruteForce (bf) | SAP.\n"
                "              Default Auto: BruteForce below 32 bodies, SAP at/above (bench\n"
                "              crossover); a manual choice sticks until Auto is picked again,\n"
                "              or switch live in the Solver Settings panel.\n"
                "  Keys: Space play/pause, '.' single step (while paused), 1/2/3 speed,\n"
                "        R reset camera (middle-drag pan, wheel zoom)\n"
                "  --smoke N   render N frames, print a summary and exit (0 = pass, 2 = no GL environment\n"
                "              or unknown demo name)\n"
                "  --hidden    hide the window (rendering still happens)\n"
                "  --capture-dir DIR  write every captured smoke frame as binary PPM (P6) into DIR\n"
                "              (requires --smoke; read back after the world + UI are drawn, before swap)\n"
                "  --capture-every K  capture one frame every K frames (default 1)\n"
                "  --capture-start N  first 1-based rendered frame to capture (default 1)\n"
                "  --capture-hide-ui  omit the ImGui panels while capturing (scene-only GIF frames)\n"
                "  --capture-display OPTS  comma-separated on/off display overrides, e.g.\n"
                "              contacts=off,normals=on (keys: contacts, normals, aabb, axes, velocities)\n"
                "  --help      show this help\n");
}

[[nodiscard]] bool ParseCommandLine(int argc, char** argv, CommandLineOptions& options) {
    for (int argumentIndex = 1; argumentIndex < argc; ++argumentIndex) {
        const char* argument = argv[argumentIndex];
        if (std::strcmp(argument, "--hidden") == 0) {
            options.hidden = true;
        } else if (std::strcmp(argument, "--smoke") == 0) {
            if (argumentIndex + 1 >= argc) {
                std::fprintf(stderr, "[testbed] --smoke requires a frame count, e.g. --smoke 120\n");
                return false;
            }
            const char* value = argv[++argumentIndex];
            char* parseEnd = nullptr;
            const long frames = std::strtol(value, &parseEnd, 10);
            if (parseEnd == value || *parseEnd != '\0' || frames <= 0 || frames > 1000000) {
                std::fprintf(stderr, "[testbed] invalid --smoke frame count: '%s'\n", value);
                return false;
            }
            options.smokeFrames = static_cast<int>(frames);
        } else if (std::strcmp(argument, "--demo") == 0) {
            if (argumentIndex + 1 >= argc) {
                std::fprintf(stderr, "[testbed] --demo requires a scene name, e.g. --demo SphereStack\n");
                return false;
            }
            options.demoName = argv[++argumentIndex];
        } else if (std::strcmp(argument, "--broadphase") == 0) {
            if (argumentIndex + 1 >= argc) {
                std::fprintf(stderr, "[testbed] --broadphase requires a name: None, BruteForce or SAP\n");
                return false;
            }
            options.broadPhaseName = argv[++argumentIndex];
        } else if (std::strcmp(argument, "--capture-dir") == 0) {
            if (argumentIndex + 1 >= argc) {
                std::fprintf(stderr, "[testbed] --capture-dir requires a directory, e.g. --capture-dir "
                                     "build/capture/box_stack\n");
                return false;
            }
            options.captureDirectory = argv[++argumentIndex];
        } else if (std::strcmp(argument, "--capture-every") == 0) {
            if (argumentIndex + 1 >= argc) {
                std::fprintf(stderr,
                             "[testbed] --capture-every requires a frame interval, e.g. --capture-every 8\n");
                return false;
            }
            const char* value = argv[++argumentIndex];
            char* parseEnd = nullptr;
            const long interval = std::strtol(value, &parseEnd, 10);
            if (parseEnd == value || *parseEnd != '\0' || interval <= 0 || interval > 1000000) {
                std::fprintf(stderr, "[testbed] invalid --capture-every interval: '%s'\n", value);
                return false;
            }
            options.captureFrameInterval = static_cast<int>(interval);
        } else if (std::strcmp(argument, "--capture-start") == 0) {
            if (argumentIndex + 1 >= argc) {
                std::fprintf(
                    stderr,
                    "[testbed] --capture-start requires a 1-based frame number, e.g. --capture-start 60\n");
                return false;
            }
            const char* value = argv[++argumentIndex];
            char* parseEnd = nullptr;
            const long startFrame = std::strtol(value, &parseEnd, 10);
            if (parseEnd == value || *parseEnd != '\0' || startFrame < 1 || startFrame > 1000000) {
                std::fprintf(stderr, "[testbed] invalid --capture-start frame: '%s' (must be >= 1)\n", value);
                return false;
            }
            options.captureStartFrame = static_cast<int>(startFrame);
        } else if (std::strcmp(argument, "--capture-hide-ui") == 0) {
            options.captureHideUi = true;
        } else if (std::strcmp(argument, "--capture-display") == 0) {
            if (argumentIndex + 1 >= argc) {
                std::fprintf(stderr,
                             "[testbed] --capture-display requires a comma-separated override list, e.g. "
                             "contacts=off,normals=on\n");
                return false;
            }
            options.captureDisplay = argv[++argumentIndex];
        } else if (std::strcmp(argument, "--help") == 0 || std::strcmp(argument, "-h") == 0) {
            options.helpRequested = true;
            return true;
        } else {
            std::fprintf(stderr, "[testbed] unknown argument: '%s'\n", argument);
            return false;
        }
    }
    // Capture options only make sense together (and only in the deterministic smoke
    // loop): rejecting the combinations here keeps the loop free of repeated checks.
    if (!options.captureDirectory.empty()) {
        if (options.captureFrameInterval == 0) {
            options.captureFrameInterval = 1;
        }
        if (options.captureStartFrame == 0) {
            options.captureStartFrame = 1;
        }
    } else if (options.captureFrameInterval != 0 || options.captureStartFrame != 0) {
        std::fprintf(stderr, "[testbed] --capture-every/--capture-start require --capture-dir\n");
        return false;
    } else if (options.captureHideUi) {
        std::fprintf(stderr, "[testbed] --capture-hide-ui requires --capture-dir\n");
        return false;
    } else if (!options.captureDisplay.empty()) {
        std::fprintf(stderr, "[testbed] --capture-display requires --capture-dir\n");
        return false;
    }
    return true;
}

// ---- Week 8: deterministic frame capture for the README GIFs -------------------
// The demo GIFs must show the exact scene and panel text the smoke run asserts on, so
// they are recorded from the same fixed-step smoke loop rather than by screen recording
// a human session. The readback happens before glfwSwapBuffers: at that point GL_BACK
// holds the finished frame (world + ImGui panels), while after the swap it already
// belongs to the next frame. Binary PPM (P6) is the intermediate format because Pillow
// decodes it without an extra dependency (this machine has no ffmpeg) and its header
// keeps the frame dimensions.
constexpr int kPpmMaxValue = 255;
constexpr int kCaptureFileNameDigits = 6;

[[nodiscard]] bool WritePpmFrame(const std::filesystem::path& directory, const std::string& directoryLabel,
                                 int frameNumber, int width, int height,
                                 const std::vector<unsigned char>& topDownRgb, std::string& error) {
    char fileName[40];
    std::snprintf(fileName, sizeof(fileName), "frame_%0*d.ppm", kCaptureFileNameDigits, frameNumber);
    // std::filesystem::path (not a char* concatenation) keeps non-ASCII parent paths
    // (this repository lives under a Chinese directory name) intact on MSVC.
    const std::filesystem::path filePath = directory / fileName;
    std::ofstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        error = "cannot open frame " + std::to_string(frameNumber) + " under '" + directoryLabel + "'";
        return false;
    }
    file << "P6\n" << width << ' ' << height << '\n' << kPpmMaxValue << '\n';
    file.write(reinterpret_cast<const char*>(topDownRgb.data()),
               static_cast<std::streamsize>(topDownRgb.size()));
    if (!file.good()) {
        error = "write failed for frame " + std::to_string(frameNumber) + " under '" + directoryLabel + "'";
        return false;
    }
    return true;
}

// glReadPixels returns rows bottom-up (the OpenGL origin is bottom-left) while every
// image file stores them top-down, so the rows are reversed while copying. GL_PACK_ALIGNMENT
// defaults to 4 and an RGB row of a 1280 px frame is not a multiple of 4 bytes, so the
// default stride would corrupt every other row; it is set to 1 and restored for the
// renderer's next frame.
[[nodiscard]] bool CaptureBackBufferToPpm(const std::filesystem::path& directory,
                                          const std::string& directoryLabel, int frameNumber, int width,
                                          int height, std::string& error) {
    while (glGetError() != GL_NO_ERROR) {
        // Drain errors left by the render pass so the check below attributes a failure to
        // glReadPixels itself (some drivers refuse back-buffer reads on hidden windows).
    }
    std::vector<unsigned char> pixels(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) *
                                      3u);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    const GLenum readError = glGetError();
    if (readError != GL_NO_ERROR) {
        error = "glReadPixels(GL_BACK) failed with GL error " + std::to_string(static_cast<int>(readError));
        return false;
    }

    const std::size_t rowBytes = static_cast<std::size_t>(width) * 3u;
    std::vector<unsigned char> topDown(pixels.size());
    for (int row = 0; row < height; ++row) {
        const std::size_t sourceOffset = static_cast<std::size_t>(height - 1 - row) * rowBytes;
        const std::size_t destinationOffset = static_cast<std::size_t>(row) * rowBytes;
        std::copy(pixels.begin() + static_cast<std::ptrdiff_t>(sourceOffset),
                  pixels.begin() + static_cast<std::ptrdiff_t>(sourceOffset + rowBytes),
                  topDown.begin() + static_cast<std::ptrdiff_t>(destinationOffset));
    }
    return WritePpmFrame(directory, directoryLabel, frameNumber, width, height, topDown, error);
}

void GlfwErrorCallback(int errorCode, const char* description) {
    std::fprintf(stderr, "[testbed] GLFW error %d: %s\n", errorCode,
                 description != nullptr ? description : "(no description)");
}

// ---- scene-name matching ------------------------------------------------------
// The UI labels contain spaces ("Sphere Stack") while the CLI ids users type are
// compact ("SphereStack"), so the comparison ignores spaces/underscores/hyphens and
// folds ASCII case. Scene names are our own ASCII identifiers, therefore a hand-rolled
// fold is enough and avoids locale surprises from std::tolower(char).
[[nodiscard]] char FoldSceneNameCharacter(char character) noexcept {
    if (character >= 'A' && character <= 'Z') {
        return static_cast<char>(character - 'A' + 'a');
    }
    return character;
}

[[nodiscard]] bool IsSceneNameSeparator(char character) noexcept {
    return character == ' ' || character == '_' || character == '-';
}

[[nodiscard]] bool MatchesSceneName(const char* displayName, const char* requestedName) noexcept {
    std::size_t displayIndex = 0;
    std::size_t requestedIndex = 0;
    for (;;) {
        while (IsSceneNameSeparator(displayName[displayIndex])) {
            ++displayIndex;
        }
        while (IsSceneNameSeparator(requestedName[requestedIndex])) {
            ++requestedIndex;
        }
        const char displayCharacter = displayName[displayIndex];
        const char requestedCharacter = requestedName[requestedIndex];
        if (displayCharacter == '\0' || requestedCharacter == '\0') {
            // Both must end together: "Sphere" must not match "SphereStack".
            return displayCharacter == '\0' && requestedCharacter == '\0';
        }
        if (FoldSceneNameCharacter(displayCharacter) != FoldSceneNameCharacter(requestedCharacter)) {
            return false;
        }
        ++displayIndex;
        ++requestedIndex;
    }
}

// Machine-readable scene id for the one-line smoke summary: strip the spaces from the
// UI label so the line stays simple key=value tokens ("demo=SphereStack").
[[nodiscard]] std::string CompactSceneName(const char* displayName) {
    std::string compactName;
    for (const char* character = displayName; *character != '\0'; ++character) {
        if (!IsSceneNameSeparator(*character)) {
            compactName.push_back(*character);
        }
    }
    return compactName;
}

[[nodiscard]] int FindDemoIndex(const std::vector<std::unique_ptr<p2::Demo>>& demos,
                                const std::string& requestedName) {
    for (int demoIndex = 0; demoIndex < static_cast<int>(demos.size()); ++demoIndex) {
        if (MatchesSceneName(demos[static_cast<std::size_t>(demoIndex)]->Name(), requestedName.c_str())) {
            return demoIndex;
        }
    }
    return -1;
}

// ---- Week 8: capture-time display overrides ----------------------------------
// The display toggles normally live in the Solver Settings panel; a recording script
// cannot click them. This parser accepts the same names as "key=on|off" pairs so a GIF
// can show contact normals (contact_debug) or hide the many moving contact markers that
// dominate a 400-body frame's GIF size (sap_stress). Anything unknown is a usage error
// instead of a silent no-op, because a typo in a recording command must not be hidden.
[[nodiscard]] bool ApplyDisplayOverrides(const std::string& overrides, p2::DebugUI::DisplayOptions& display) {
    std::size_t tokenStart = 0;
    while (tokenStart <= overrides.size()) {
        const std::size_t comma = overrides.find(',', tokenStart);
        const std::string token =
            overrides.substr(tokenStart, comma == std::string::npos ? std::string::npos : comma - tokenStart);
        if (!token.empty()) {
            const std::size_t equals = token.find('=');
            if (equals == std::string::npos) {
                std::fprintf(stderr, "[testbed] --capture-display entry '%s' is not key=value\n",
                             token.c_str());
                return false;
            }
            const std::string key = token.substr(0, equals);
            const std::string value = token.substr(equals + 1);
            // The scene-name comparator is reused on purpose: it is the project's single
            // case-folding helper for these loose CLI values (on/ON/On all work).
            const bool enabled = MatchesSceneName("on", value.c_str()) ||
                                 MatchesSceneName("true", value.c_str()) || value == "1";
            const bool disabled = MatchesSceneName("off", value.c_str()) ||
                                  MatchesSceneName("false", value.c_str()) || value == "0";
            if (!enabled && !disabled) {
                std::fprintf(stderr, "[testbed] --capture-display value '%s' must be on/off\n",
                             value.c_str());
                return false;
            }
            bool* target = nullptr;
            if (key == "contacts") {
                target = &display.showContacts;
            } else if (key == "normals") {
                target = &display.showNormals;
            } else if (key == "aabb") {
                target = &display.showAabb;
            } else if (key == "axes") {
                target = &display.showAxes;
            } else if (key == "velocities") {
                target = &display.showVelocities;
            }
            if (target == nullptr) {
                std::fprintf(stderr,
                             "[testbed] --capture-display unknown key '%s' (contacts, normals, aabb, axes, "
                             "velocities)\n",
                             key.c_str());
                return false;
            }
            *target = enabled;
        }
        if (comma == std::string::npos) {
            break;
        }
        tokenStart = comma + 1;
    }
    return true;
}

// ---- Week 6: broad-phase selection -------------------------------------------
// The testbed owns the policy (DebugUI::BroadPhaseMode), World owns the object:
//  - Auto (CLI default): after every scene build the host calls
//    DebugUI::ApplyBroadPhase, which installs BruteForce for small scenes and SAP above
//    the bench-measured crossover (DebugUI::kBroadPhaseAutoThreshold = 32 bodies);
//  - None / BruteForce / SAP are explicit overrides (--broadphase or the panel
//    dropdown) that survive scene switches until Auto is selected again.
// All choices must produce identical physics because World canonicalises the candidate
// pairs before the narrow phase; only proxies/candidate-pair counts differ.
// (The actual installation lives in DebugUI.cpp; main only mirrors the CLI string.)
[[nodiscard]] bool ResolveBroadPhaseMode(const std::string& requestedName,
                                         p2::DebugUI::BroadPhaseMode& mode) {
    if (MatchesSceneName("Auto", requestedName.c_str())) {
        mode = p2::DebugUI::BroadPhaseMode::Auto;
        return true;
    }
    if (MatchesSceneName("None", requestedName.c_str())) {
        mode = p2::DebugUI::BroadPhaseMode::None;
        return true;
    }
    if (MatchesSceneName("BruteForce", requestedName.c_str()) ||
        MatchesSceneName("Brute", requestedName.c_str()) || MatchesSceneName("BF", requestedName.c_str())) {
        mode = p2::DebugUI::BroadPhaseMode::BruteForce;
        return true;
    }
    if (MatchesSceneName("SAP", requestedName.c_str())) {
        mode = p2::DebugUI::BroadPhaseMode::SAP;
        return true;
    }
    return false;
}

// ---- Week 6: camera framing ---------------------------------------------------
// The Week 6 scenes are bigger than the default 32x20 m view (Pyramid200 is 20 m tall,
// the StressBenchmark field spans ~36 m). The host focuses the camera ONCE per scene
// build; focusing every frame would fight the user's pan/zoom. The infinite ground
// plane is skipped: its proxy AABB is deliberately huge and would dominate any union.
[[nodiscard]] bool ComputeFiniteSceneBounds(const p2::World& world, p2::AABB& bounds) {
    bool hasBounds = false;
    for (const p2::Body& body : world.GetBodies()) {
        if (!body.handle.IsValid() || !world.IsValid(body.handle) || !world.IsValidShape(body.shape)) {
            continue;
        }
        const p2::Shape* shape = world.GetShape(body.shape);
        if (shape == nullptr || shape->type == p2::ShapeType::Plane) {
            continue;
        }
        // body.aabb already includes the Week 6 fat margin (+kAabbMargin); FocusOnWorld
        // adds its own padding ratio on top, so the framing has a little breathing room.
        bounds = hasBounds ? p2::Union(bounds, body.aabb) : body.aabb;
        hasBounds = true;
    }
    return hasBounds;
}

void FocusCameraOnScene(const p2::World& world, p2::Camera& camera) {
    p2::AABB bounds;
    if (!ComputeFiniteSceneBounds(world, bounds)) {
        return;
    }
    // Small scenes keep the conventional origin-centred 1x view: a close-up framing of
    // FallingBall would push its ground line off screen and change the look of the
    // Week 1-5 scenes for no benefit. The test is bounds-containment in the default view
    // rectangle (not just extent comparison) because scenes sit on the ground: a 16 m
    // tall pyramid has 8 m half extents but spans y=0..16, which the centred view clips.
    // 0.85 leaves a margin so a scene touching the window edge still gets framed.
    const p2::Vec2 defaultView = camera.GetDefaultHalfExtents();
    constexpr float kViewFitMarginRatio = 0.85f;
    const bool fitsDefaultView = bounds.lowerBound.x >= -defaultView.x * kViewFitMarginRatio &&
                                 bounds.upperBound.x <= defaultView.x * kViewFitMarginRatio &&
                                 bounds.lowerBound.y >= -defaultView.y * kViewFitMarginRatio &&
                                 bounds.upperBound.y <= defaultView.y * kViewFitMarginRatio;
    if (fitsDefaultView) {
        // Reset (not "leave as is"): after a zoomed-out Week 6 scene the next small
        // scene must not inherit that camera state.
        camera.Reset();
        return;
    }
    camera.FocusOnWorld(bounds);
}

// ---- contact / velocity debug overlays (Week 2, extended Week 5) --------------
// Contact points are the manifold anchors. A point is stored in each body's local
// frame (localPointA/localPointB); both describe the same world position, so
// transforming one with its owner's transform is enough.

[[nodiscard]] bool IsHighlightedContact(const p2::Contact& contact,
                                        const p2::DebugUI::DisplayOptions& display) noexcept {
    return display.highlightedContactBodyA.IsValid() && contact.bodyA == display.highlightedContactBodyA &&
           contact.bodyB == display.highlightedContactBodyB;
}

// One pass over the touching contacts draws every point and/or normal the display
// options ask for. The contact highlighted in the Contacts panel is always drawn
// (larger, cyan), so clicking a row has a visible effect even when the normal
// overlay is off - that is what the panel's "click to inspect" hint promises.
void DrawContactOverlay(p2::Renderer& renderer, const p2::World& world,
                        const p2::DebugUI::DisplayOptions& display) {
    for (const p2::Contact& contact : world.GetContacts()) {
        if (!contact.manifold.IsTouching()) {
            continue;
        }
        const bool highlighted = IsHighlightedContact(contact, display);
        const bool drawPoints = display.showContacts || highlighted;
        const bool drawNormals = display.showNormals || highlighted;
        if (!drawPoints && !drawNormals) {
            continue;
        }
        const float pointRadius = highlighted ? kHighlightedContactPointRadius : kContactPointRadius;
        const float normalLength = highlighted ? kHighlightedContactNormalLength : kContactNormalLength;
        const p2::Color pointColor = highlighted ? kHighlightedContactColor : kContactPointColor;
        const p2::Color normalColor = highlighted ? kHighlightedContactColor : kContactNormalColor;
        // The manifold normal points from body A toward body B (Section 4.2,
        // normative). World normalises the pair order by body handle index, so in
        // most scenes the ground is created first and is therefore A: the arrow
        // points up out of the surface. The overlay draws the stored direction and
        // never flips it locally, exactly like the solver, so a wrong normal
        // convention stays visible instead of being masked.
        const p2::Transform transformA = world.GetTransform(contact.bodyA);
        for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
            const p2::Vec2 worldPoint = p2::Mul(transformA, contact.manifold.points[pointIndex].localPointA);
            if (drawPoints) {
                renderer.DrawCircle(worldPoint, pointRadius, pointColor, 12);
                renderer.DrawPoint(worldPoint, pointColor, pointRadius);
            }
            if (drawNormals) {
                renderer.DrawArrow(worldPoint, worldPoint + contact.manifold.normal * normalLength,
                                   normalColor, kContactArrowHeadLength);
            }
        }
    }
}

// Optional engine-level velocity arrows for every non-static body. Kept separate
// from the per-scene arrows (most demos draw their own): this one exists for scenes
// that do not, and for a quick "which body is still moving" scan.
void DrawVelocityOverlay(p2::Renderer& renderer, const p2::World& world) {
    for (const p2::Body& body : world.GetBodies()) {
        if (!body.handle.IsValid() || !world.IsValid(body.handle) || body.type == p2::BodyType::Static) {
            continue;
        }
        const p2::Vec2 velocity = world.GetLinearVelocity(body.handle);
        if (p2::LengthSquared(velocity) <= kVelocityArrowThresholdSquared) {
            continue;
        }
        const p2::Vec2 position = world.GetPosition(body.handle);
        renderer.DrawArrow(position, position + velocity * kVelocityArrowScale, kVelocityArrowColor);
    }
}

// ---- Week 5 material defaults ------------------------------------------------
// The DebugUI material sliders are experiment defaults ("set friction to 0 and the
// ten-box tower must collapse"). They are written onto every body when a scene is
// built and on demand from the panel. Static bodies are included on purpose: the
// contact combines friction as sqrt(fA*fB) and restitution as max(eA,eB), so
// skimming the ground would silently change the experiment value.
void ApplyMaterialDefaults(p2::World& world, const p2::DebugUI::MaterialDefaults& material) {
    for (const p2::Body& body : world.GetBodies()) {
        if (!body.handle.IsValid() || !world.IsValid(body.handle)) {
            continue;
        }
        world.SetFriction(body.handle, material.friction);
        world.SetRestitution(body.handle, material.restitution);
    }
}

// --- RAII wrappers ------------------------------------------------------------
// Startup can fail at several points; these wrappers guarantee the matching
// teardown runs in the reverse order (Renderer -> ImGui -> GLFW window -> GLFW),
// which is the order ImGui and OpenGL require.

class GlfwRuntime {
  public:
    GlfwRuntime() = default;
    ~GlfwRuntime() {
        if (m_initialized) {
            glfwTerminate();
        }
    }
    GlfwRuntime(const GlfwRuntime&) = delete;
    GlfwRuntime& operator=(const GlfwRuntime&) = delete;

    [[nodiscard]] bool Initialize() {
        m_initialized = glfwInit() == GLFW_TRUE;
        return m_initialized;
    }

  private:
    bool m_initialized = false;
};

class GlfwWindow {
  public:
    GlfwWindow() = default;
    ~GlfwWindow() {
        if (m_window != nullptr) {
            glfwDestroyWindow(m_window);
        }
    }
    GlfwWindow(const GlfwWindow&) = delete;
    GlfwWindow& operator=(const GlfwWindow&) = delete;

    [[nodiscard]] GLFWwindow* Get() const noexcept { return m_window; }
    void Set(GLFWwindow* window) noexcept { m_window = window; }

  private:
    GLFWwindow* m_window = nullptr;
};

class ImGuiRuntime {
  public:
    ImGuiRuntime() = default;
    ~ImGuiRuntime() {
        if (m_openGlBackendInitialized) {
            ImGui_ImplOpenGL3_Shutdown();
        }
        if (m_glfwBackendInitialized) {
            ImGui_ImplGlfw_Shutdown();
        }
        if (m_contextCreated) {
            ImGui::DestroyContext();
        }
    }
    ImGuiRuntime(const ImGuiRuntime&) = delete;
    ImGuiRuntime& operator=(const ImGuiRuntime&) = delete;

    [[nodiscard]] bool Initialize(GLFWwindow* window, std::string& error) {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        m_contextCreated = true;
        ImGuiIO& io = ImGui::GetIO();
        io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
        // No persistent imgui.ini in Week 1: the layout is deliberately basic and
        // the smoke test must not create files in the working directory.
        io.IniFilename = nullptr;
        ImGui::StyleColorsDark();

        if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
            error = "ImGui_ImplGlfw_InitForOpenGL returned false";
            return false;
        }
        m_glfwBackendInitialized = true;
        if (!ImGui_ImplOpenGL3_Init(kGlslVersion)) {
            error = "ImGui_ImplOpenGL3_Init returned false";
            return false;
        }
        m_openGlBackendInitialized = true;
        return true;
    }

  private:
    bool m_contextCreated = false;
    bool m_glfwBackendInitialized = false;
    bool m_openGlBackendInitialized = false;
};

int RunTestbed(const CommandLineOptions& options) {
    const bool smokeMode = options.smokeFrames > 0;
    const int environmentExitCode = smokeMode ? kExitEnvironmentUnsupported : kExitFailure;

    // Week 8 demo-GIF capture is defined on the deterministic smoke loop (one fixed
    // 1/60 s step per rendered frame), so it is a usage error without --smoke. The
    // directory is created before GLFW starts so a bad path fails fast, before any
    // window/GL state has to be torn down.
    const bool captureEnabled = !options.captureDirectory.empty();
    if (captureEnabled) {
        if (!smokeMode) {
            std::fprintf(stderr,
                         "[testbed] --capture-dir requires --smoke N: capture records the deterministic "
                         "fixed-step frames\n");
            return kExitFailure;
        }
        std::error_code directoryError;
        std::filesystem::create_directories(options.captureDirectory, directoryError);
        if (directoryError) {
            std::fprintf(stderr, "[testbed] cannot create capture directory '%s': %s\n",
                         options.captureDirectory.c_str(), directoryError.message().c_str());
            return kExitFailure;
        }
        std::printf("[testbed] capture: every %d frame(s) starting at frame %d, PPM (P6) into '%s' "
                    "(world%s, read back before swap)\n",
                    options.captureFrameInterval, options.captureStartFrame, options.captureDirectory.c_str(),
                    options.captureHideUi ? " only, panels hidden" : " + UI");
    }
    // Panels are skipped only for capture runs that asked for it; the world render and
    // the fixed-step simulation are untouched either way.
    const bool captureHideUi = captureEnabled && options.captureHideUi;

    // ---- demo registry + CLI scene selection --------------------------------
    // Built before GLFW/GL: an unknown --demo is a configuration error and must be
    // reported (exit code 2 by contract) even on a machine that cannot open a window.
    std::vector<std::unique_ptr<p2::Demo>> demos;
    demos.push_back(std::make_unique<p2::FallingBallDemo>());
    demos.push_back(std::make_unique<p2::SphereStackDemo>());
    demos.push_back(std::make_unique<p2::BoxStackDemo>());
    demos.push_back(std::make_unique<p2::FrictionRampDemo>());
    demos.push_back(std::make_unique<p2::DominoesDemo>());
    demos.push_back(std::make_unique<p2::IslandSleep500Demo>());
    demos.push_back(std::make_unique<p2::Pyramid200Demo>());
    demos.push_back(std::make_unique<p2::StressBenchmarkDemo>());
    std::vector<const char*> demoNames;
    demoNames.reserve(demos.size());
    for (const std::unique_ptr<p2::Demo>& demo : demos) {
        demoNames.push_back(demo->Name());
    }

    const int initialDemoIndex = FindDemoIndex(demos, options.demoName);
    if (initialDemoIndex < 0) {
        std::fprintf(stderr,
                     "[testbed] unknown demo '%s'\n[testbed] available demos:", options.demoName.c_str());
        for (const char* demoName : demoNames) {
            std::fprintf(stderr, " '%s'", demoName);
        }
        std::fprintf(stderr,
                     "\n[testbed] scene names are matched case-insensitively; spaces/underscores/hyphens are "
                     "ignored\n");
        return kExitUnknownDemo;
    }
    int selectedDemo = initialDemoIndex;

    // Broad phase is a CLI configuration choice like the scene name: resolve it before
    // touching GLFW/GL so a typo is diagnosable on any machine (and with --hidden).
    p2::DebugUI::BroadPhaseMode broadPhaseMode = p2::DebugUI::BroadPhaseMode::Auto;
    if (!ResolveBroadPhaseMode(options.broadPhaseName, broadPhaseMode)) {
        std::fprintf(stderr,
                     "[testbed] unknown broad phase '%s'\n[testbed] available broad phases: Auto, None, "
                     "BruteForce (bf), SAP\n",
                     options.broadPhaseName.c_str());
        return kExitFailure;
    }
    if (broadPhaseMode == p2::DebugUI::BroadPhaseMode::Auto) {
        std::printf("[testbed] broad phase: Auto (BruteForce < %d bodies, SAP >= %d; bench crossover)\n",
                    p2::DebugUI::kBroadPhaseAutoThreshold, p2::DebugUI::kBroadPhaseAutoThreshold);
    } else {
        std::printf("[testbed] broad phase: %s (manual override, --broadphase)\n",
                    p2::DebugUI::BroadPhaseModeName(broadPhaseMode));
    }

    glfwSetErrorCallback(&GlfwErrorCallback);
    GlfwRuntime glfwRuntime;
    if (!glfwRuntime.Initialize()) {
        std::fprintf(stderr, "[testbed] glfwInit() failed\n");
        return environmentExitCode;
    }
    std::printf("[testbed] GLFW %s\n", glfwGetVersionString());

    // OpenGL 3.3 core. macOS additionally requires the forward-compatible hint or
    // window creation returns null (Week 1 warning in the design document).
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
    if (options.hidden) {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    }

    GlfwWindow windowGuard;
    windowGuard.Set(
        glfwCreateWindow(kInitialWindowWidth, kInitialWindowHeight, kWindowTitle, nullptr, nullptr));
    if (windowGuard.Get() == nullptr) {
        const char* description = nullptr;
        const int errorCode = glfwGetError(&description);
        std::fprintf(stderr, "[testbed] failed to create an OpenGL 3.3 core window (GLFW error %d: %s)\n",
                     errorCode, description != nullptr ? description : "no description");
        std::fprintf(
            stderr,
            "[testbed] no usable desktop session or OpenGL 3.3 driver is available in this environment\n");
        return environmentExitCode;
    }
    GLFWwindow* window = windowGuard.Get();
    glfwMakeContextCurrent(window);
    // VSync for the interactive view; off for smoke runs so average frame time
    // measures rendering instead of waiting for the monitor.
    glfwSwapInterval(smokeMode ? 0 : 1);

    const GLubyte* glVersionString = glGetString(GL_VERSION);
    const GLubyte* glRendererString = glGetString(GL_RENDERER);
    std::printf("[testbed] OpenGL %s (%s)\n",
                glVersionString != nullptr ? reinterpret_cast<const char*>(glVersionString) : "unknown",
                glRendererString != nullptr ? reinterpret_cast<const char*>(glRendererString) : "unknown");

    // Camera input callbacks must exist before the ImGui backend installs its own
    // callbacks: the backend chains pre-existing callbacks (see Camera.h).
    p2::Camera camera;
    camera.SetViewportSize(kInitialWindowWidth, kInitialWindowHeight);
    camera.InstallInputCallbacks(window);

    ImGuiRuntime imguiRuntime;
    std::string startupError;
    if (!imguiRuntime.Initialize(window, startupError)) {
        std::fprintf(stderr, "[testbed] Dear ImGui initialization failed: %s\n", startupError.c_str());
        return environmentExitCode;
    }

    p2::Renderer renderer;
    if (!renderer.Initialize(startupError)) {
        std::fprintf(stderr, "[testbed] renderer initialization failed: %s\n", startupError.c_str());
        // A missing core entry point means the driver did not really provide 3.3;
        // anything else (shader compile/link) is a real failure of our code.
        const bool environmentIssue = startupError.rfind("required OpenGL 3.3 entry point", 0) == 0;
        return (smokeMode && environmentIssue) ? kExitEnvironmentUnsupported : kExitFailure;
    }

    // ---- scene setup --------------------------------------------------------
    // The registry and the CLI selection were resolved above; this is the only part
    // that needs the (now fully initialized) simulation/render stack.
    p2::World world;
    p2::StepAccumulator accumulator(p2::kFixedTimeStep, p2::kMaxStepsPerFrame);
    // DebugUI is constructed before the first scene build so the material defaults
    // (Week 5 solver experiments) and the Week 6 broad-phase policy are applied
    // uniformly to the initial scene and to every later scene switch.
    p2::DebugUI debugUi;
    p2::DebugUI::DisplayOptions displayOptions;
    // Week 8: recording-only display overrides, applied once before the first frame (the
    // same switches the Solver Settings panel writes). captureEnabled is already false
    // without --capture-dir, so interactive runs keep the panel defaults.
    if (captureEnabled && !options.captureDisplay.empty()) {
        if (!ApplyDisplayOverrides(options.captureDisplay, displayOptions)) {
            std::fprintf(stderr, "[testbed] invalid --capture-display '%s'\n",
                         options.captureDisplay.c_str());
            return kExitFailure;
        }
    }
    debugUi.SetBroadPhaseMode(broadPhaseMode);
    demos[static_cast<std::size_t>(selectedDemo)]->Setup(world);
    // Week 6: the broad phase is chosen AFTER the scene exists, because Auto dispatches
    // on the live body count (small scenes: BruteForce, large: SAP). Nothing has stepped
    // yet, so installing now is equivalent to installing before setup.
    debugUi.ApplyBroadPhase(world);
    ApplyMaterialDefaults(world, debugUi.GetMaterialDefaults());
    // Week 6: frame the scene once. The 20 m pyramid and the random stress field are
    // larger than the default 32x20 m view; focusing per frame would fight pan/zoom.
    FocusCameraOnScene(world, camera);

    using Clock = std::chrono::steady_clock;
    Clock::time_point previousFrameTime = Clock::now();
    float smoothedFrameMilliseconds = 0.0f;
    float smoothedStepMilliseconds = 0.0f;
    int32_t physicsStepsLastFrame = 0;
    int frameIndex = 0;
    int capturedFrameCount = 0;
    int32_t totalPhysicsSteps = 0;
    double totalFrameMilliseconds = 0.0;
    // Week 7: raw Step time of the previous frame, handed to the demos through
    // DemoFrameStats.rawStepMilliseconds (IslandSleep500 averages its sleep windows from
    // raw values so the host's smoothing does not blur the awake -> asleep transition).
    float previousRawStepMilliseconds = 0.0f;
    // Week 7 keyboard-shortcut edge detection (declared outside the loop so a held key
    // fires once). Space = play/pause, '.' = single step while paused, 1/2/3 = speed.
    bool spaceHeld = false;
    bool periodHeld = false;
    bool speed1Held = false;
    bool speed2Held = false;
    bool speed3Held = false;

    for (;;) {
        if (smokeMode && frameIndex >= options.smokeFrames) {
            break;
        }

        const Clock::time_point frameStart = Clock::now();
        const float frameMilliseconds =
            std::chrono::duration<float, std::milli>(frameStart - previousFrameTime).count();
        previousFrameTime = frameStart;

        glfwPollEvents();
        if (glfwWindowShouldClose(window) != GLFW_FALSE) {
            break;
        }
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
            // A minimized window has a zero-sized framebuffer: the viewport math
            // would divide by zero and ImGui warns about a 0x0 display. Wait for
            // the next event instead of spinning through broken frames.
            glfwWaitEventsTimeout(0.05);
            previousFrameTime = Clock::now();
            continue;
        }

        // ---- UI --------------------------------------------------------------
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        // Full-viewport dockspace; the central node is transparent so the world
        // render below shows through it while the panels dock at the edges. Skipped
        // when --capture-hide-ui removes the panels altogether.
        if (!captureHideUi) {
            ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);
        }

        const ImGuiIO& io = ImGui::GetIO();
        camera.UpdateInput(window, !io.WantCaptureMouse && !io.WantCaptureKeyboard);

        // Week 7: keyboard playback shortcuts. Suppressed while ImGui owns the keyboard
        // (typing in a panel / the ImGui demo window), so they never fight a text field.
        // The on-screen buttons stay the discoverable path; README documents the keys.
        const bool shortcutsAllowed = !io.WantCaptureKeyboard;
        const bool spaceDown = shortcutsAllowed && glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS;
        if (spaceDown && !spaceHeld) {
            debugUi.TogglePaused();
        }
        spaceHeld = spaceDown;
        const bool periodDown = shortcutsAllowed && glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS;
        const bool singleStepByKey = periodDown && !periodHeld && debugUi.IsPaused();
        periodHeld = periodDown;
        const bool speed1Down = shortcutsAllowed && glfwGetKey(window, GLFW_KEY_1) == GLFW_PRESS;
        const bool speed2Down = shortcutsAllowed && glfwGetKey(window, GLFW_KEY_2) == GLFW_PRESS;
        const bool speed3Down = shortcutsAllowed && glfwGetKey(window, GLFW_KEY_3) == GLFW_PRESS;
        if (speed1Down && !speed1Held) {
            debugUi.SetSpeedIndex(0); // 1x
        }
        if (speed2Down && !speed2Held) {
            debugUi.SetSpeedIndex(1); // 0.5x
        }
        if (speed3Down && !speed3Held) {
            debugUi.SetSpeedIndex(2); // 0.25x
        }
        speed1Held = speed1Down;
        speed2Held = speed2Down;
        speed3Held = speed3Down;

        int windowWidth = kInitialWindowWidth;
        int windowHeight = kInitialWindowHeight;
        glfwGetWindowSize(window, &windowWidth, &windowHeight);
        camera.SetViewportSize(windowWidth, windowHeight);

        p2::DebugUI::FrameStats stats;
        stats.framesPerSecond = smoothedFrameMilliseconds > 0.0f ? 1000.0f / smoothedFrameMilliseconds : 0.0f;
        stats.frameMilliseconds = smoothedFrameMilliseconds;
        stats.physicsStepMilliseconds = smoothedStepMilliseconds;
        stats.physicsSteps = physicsStepsLastFrame;

        // Week 6/7: scenes that display host timings (StressBenchmark, IslandSleep500)
        // receive them before DebugUI::Draw calls Demo::Ui later in this frame. The raw
        // field is the previous frame's value: main only knows this frame's Step cost
        // after the physics block below.
        demos[static_cast<std::size_t>(selectedDemo)]->SetFrameStats(p2::DemoFrameStats{
            stats.framesPerSecond, stats.physicsStepMilliseconds, previousRawStepMilliseconds});

        const p2::DebugUI::FrameActions actions =
            captureHideUi ? p2::DebugUI::FrameActions{}
                          : debugUi.Draw(world, *demos[static_cast<std::size_t>(selectedDemo)], demoNames,
                                         selectedDemo, stats, displayOptions);
        if (actions.quitRequested) {
            break;
        }
        if (actions.demoChanged) {
            // Switching scenes rebuilds the World from scratch: leftover bodies
            // and accumulated time from the previous scene must not leak in.
            world.Clear();
            demos[static_cast<std::size_t>(selectedDemo)]->Setup(world);
            accumulator.Reset();
            // Week 6: re-apply the broad-phase mode after every scene build. In Auto it
            // re-derives BruteForce/SAP from the new body count; a manual override is kept
            // (the panel says so). World::Clear released the old proxies, the new ones are
            // inserted on the first Step.
            debugUi.ApplyBroadPhase(world);
            // Week 5: a newly built scene picks up the panel's material defaults
            // (friction/restitution experiments). Reset() deliberately does not: a
            // demo's own Reset keeps the material state the user tuned in its Ui.
            ApplyMaterialDefaults(world, debugUi.GetMaterialDefaults());
            // Week 6: frame the new scene once.
            FocusCameraOnScene(world, camera);
        }
        if (actions.applyMaterialDefaultsRequested) {
            ApplyMaterialDefaults(world, debugUi.GetMaterialDefaults());
        }
        if (actions.resetRequested) {
            demos[static_cast<std::size_t>(selectedDemo)]->Reset(world);
            // A rebuild can change the scene size (Pyramid200 layer count, StressBenchmark
            // box count), so re-apply the broad-phase mode (Auto re-dispatches) and reframe.
            debugUi.ApplyBroadPhase(world);
            FocusCameraOnScene(world, camera);
        }
        // Re-bind after the actions: the combo box may have changed the selection,
        // and stepping/rendering must use the new scene's demo object.
        p2::Demo& currentDemo = *demos[static_cast<std::size_t>(selectedDemo)];

        // ---- physics ---------------------------------------------------------
        // In smoke mode every frame simulates exactly one fixed step, keeping the
        // automated check deterministic: 120 frames always means 120 steps.
        const float simulationFrameSeconds = smokeMode ? p2::kFixedTimeStep : frameMilliseconds * 0.001f;
        int32_t stepsThisFrame = 0;
        if (actions.singleStepRequested || singleStepByKey) {
            // Keyboard '.' is a Week 7 shortcut that injects the same one-step action as
            // the UI button (only reachable while paused, see the key handling above).
            stepsThisFrame = 1;
        } else if (!debugUi.IsPaused()) {
            // The speed multiplier scales accumulated simulated time, never the
            // fixed dt itself: warm starting and determinism depend on dt being
            // constant (Section 4.3).
            stepsThisFrame = accumulator.Advance(simulationFrameSeconds * debugUi.GetSpeedMultiplier());
        }

        float stepMilliseconds = 0.0f;
        if (stepsThisFrame > 0) {
            const Clock::time_point stepStart = Clock::now();
            for (int32_t stepIndex = 0; stepIndex < stepsThisFrame; ++stepIndex) {
                world.Step(p2::kFixedTimeStep);
            }
            const Clock::time_point stepEnd = Clock::now();
            stepMilliseconds = std::chrono::duration<float, std::milli>(stepEnd - stepStart).count() /
                               static_cast<float>(stepsThisFrame);
        }
        totalPhysicsSteps += stepsThisFrame;
        physicsStepsLastFrame = stepsThisFrame;
        previousRawStepMilliseconds = stepMilliseconds;

        // Demos advance with simulated time, not wall-clock time: when paused the
        // scene is frozen, and at 0.25x speed the visual animation slows down too.
        if (stepsThisFrame > 0) {
            currentDemo.Update(world, p2::kFixedTimeStep * static_cast<float>(stepsThisFrame));
        }

        // ---- render ----------------------------------------------------------
        int frameBufferWidth = kInitialWindowWidth;
        int frameBufferHeight = kInitialWindowHeight;
        glfwGetFramebufferSize(window, &frameBufferWidth, &frameBufferHeight);
        glViewport(0, 0, frameBufferWidth, frameBufferHeight);
        glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        renderer.Begin(camera);
        currentDemo.Draw(renderer, world);
        // Engine-level overlays before the static debug geometry so the contact
        // markers are the brightest thing on screen when a contact exists.
        if (displayOptions.showVelocities) {
            DrawVelocityOverlay(renderer, world);
        }
        DrawContactOverlay(renderer, world, displayOptions);
        if (displayOptions.showAxes) {
            renderer.DrawAxes(5.0f);
        }
        if (displayOptions.showAabb) {
            // Week 6: Synchronize() now stores a FAT AABB (tight bounds + kAabbMargin) -
            // the broad phase sees that box, so the overlay intentionally shows the
            // inflated bound rather than the tight one. The infinite plane's AABB is
            // deliberately enormous; skip it (and collider-less bodies) so the overlay
            // does not try to draw a 2000 km box.
            for (const p2::Body& body : world.GetBodies()) {
                if (!body.handle.IsValid() || !world.IsValid(body.handle)) {
                    continue;
                }
                if (body.aabb.GetWidth() <= 0.0f && body.aabb.GetHeight() <= 0.0f) {
                    continue;
                }
                if (body.aabb.GetWidth() > kMaxDrawableAabbExtent ||
                    body.aabb.GetHeight() > kMaxDrawableAabbExtent) {
                    continue; // infinite plane / degenerate proxy
                }
                renderer.DrawAABB(body.aabb, p2::Color{0.45f, 0.75f, 0.45f, 0.6f});
            }
        }
        renderer.End();

        // UI is drawn after the world so panels overlay the scene; both share the
        // same framebuffer, cleared once above.
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Week 8: read the finished frame back before the swap. frameIndex is 0-based
        // while the capture contract (and --capture-start) is 1-based, so frame 1 is the
        // first rendered frame. Nothing in this block runs when capture is disabled.
        if (captureEnabled) {
            const int frameNumber = frameIndex + 1;
            if (frameNumber >= options.captureStartFrame &&
                (frameNumber - options.captureStartFrame) % options.captureFrameInterval == 0) {
                std::string captureError;
                if (!CaptureBackBufferToPpm(options.captureDirectory, options.captureDirectory, frameNumber,
                                            frameBufferWidth, frameBufferHeight, captureError)) {
                    std::fprintf(stderr, "[testbed] frame capture failed: %s\n", captureError.c_str());
                    return kExitFailure;
                }
                ++capturedFrameCount;
            }
        }
        glfwSwapBuffers(window);

        smoothedFrameMilliseconds = smoothedFrameMilliseconds <= 0.0f
                                        ? frameMilliseconds
                                        : smoothedFrameMilliseconds * 0.9f + frameMilliseconds * 0.1f;
        if (stepsThisFrame > 0) {
            smoothedStepMilliseconds = smoothedStepMilliseconds <= 0.0f
                                           ? stepMilliseconds
                                           : smoothedStepMilliseconds * 0.9f + stepMilliseconds * 0.1f;
        }

        ++frameIndex;
        totalFrameMilliseconds += static_cast<double>(frameMilliseconds);
    }

    if (smokeMode) {
        const double averageFrameMilliseconds =
            frameIndex > 0 ? totalFrameMilliseconds / static_cast<double>(frameIndex) : 0.0;
        const p2::Demo& reportedDemo = *demos[static_cast<std::size_t>(selectedDemo)];
        const std::string compactDemoName = CompactSceneName(reportedDemo.Name());

        // Contact aggregates for the one-line verdict. "contacts" counts touching
        // manifolds, "contact_points" counts the individual anchors (two for a
        // face-face manifold, one for a vertex/circle hit), and the penetration is the
        // milestone metric (10-box tower: < 10 mm).
        int32_t touchingContactCount = 0;
        int32_t contactPointCount = 0;
        float maxPenetration = 0.0f;
        for (const p2::Contact& contact : world.GetContacts()) {
            if (!contact.manifold.IsTouching()) {
                continue;
            }
            ++touchingContactCount;
            contactPointCount += contact.manifold.pointCount;
            maxPenetration = std::max(maxPenetration, contact.manifold.penetration);
        }

        // Body aggregates for the stress scenes: the Week 6 pyramid/random-field verdict
        // cannot be read off 400 per-body lines, so the summary carries the worst tilt,
        // the fastest body and the highest centre (the top-height evidence for a stack
        // that sank or toppled). Static bodies are skipped: they are the reference.
        float maxSpeedMetersPerSecond = 0.0f;
        float maxTiltDegrees = 0.0f;
        float maxCenterY = 0.0f;
        for (const p2::Body& body : world.GetBodies()) {
            if (!body.handle.IsValid() || !world.IsValid(body.handle) || body.type == p2::BodyType::Static) {
                continue;
            }
            maxSpeedMetersPerSecond =
                std::max(maxSpeedMetersPerSecond, p2::Length(world.GetLinearVelocity(body.handle)));
            maxTiltDegrees = std::max(maxTiltDegrees, std::fabs(world.GetAngle(body.handle) * p2::kRadToDeg));
            maxCenterY = std::max(maxCenterY, world.GetPosition(body.handle).y);
        }

        // Week 6: which broad phase produced this run and what it handed to the narrow
        // phase. Having these on the summary line turns the SAP/BruteForce/None
        // comparison into a one-line diff: candidate_pairs/proxies differ by
        // implementation, while every body/contact line below must stay identical.
        const p2::IBroadPhase* installedBroadPhase = world.GetBroadPhase();
        const char* broadPhaseName = installedBroadPhase != nullptr ? installedBroadPhase->Name() : "None";
        const int32_t proxyCount = installedBroadPhase != nullptr ? installedBroadPhase->GetProxyCount() : 0;
        const int32_t candidatePairCount = world.GetBroadPhasePairCount();

        // One machine-readable line for automated verification: scene id, counts,
        // penetration, motion/height aggregates, Week 7 island/sleep state, broad-phase
        // numbers and timing. The per-body / per-contact lines below carry the state a
        // verdict actually needs (final heights and velocities, which pairs still touch,
        // and the accumulated solver impulses).
        std::printf("[smoke] demo=%s frames=%d physics_steps=%d bodies=%d shapes=%d islands=%d awake=%d "
                    "sleeping=%d contacts=%d contact_points=%d max_penetration_mm=%.3f max_speed_mps=%.3f "
                    "max_tilt_deg=%.2f max_center_y=%.3f broadphase=%s candidate_pairs=%d proxies=%d "
                    "avg_frame_ms=%.3f\n",
                    compactDemoName.c_str(), frameIndex, totalPhysicsSteps, world.GetBodyCount(),
                    world.GetShapeCount(), world.GetIslandCount(), world.GetAwakeBodyCount(),
                    world.GetSleepingBodyCount(), touchingContactCount, contactPointCount,
                    maxPenetration * 1000.0f, maxSpeedMetersPerSecond, maxTiltDegrees, maxCenterY,
                    broadPhaseName, candidatePairCount, proxyCount, averageFrameMilliseconds);

        // The solver configuration the run actually used: quoting smoke numbers
        // without these would make a warm-start / split-impulse comparison meaningless.
        const p2::SolverSettings& solverSettings = world.GetSolverSettings();
        std::printf(
            "[smoke] solver velocity_iterations=%d position_iterations=%d warm_starting=%d "
            "warm_start_factor=%.2f split_impulse=%d split_impulse_beta=%.2f linear_slop_m=%.4f "
            "sat_cache=%d gravity_y=%.1f enable_sleeping=%d use_island_solving=%d\n",
            solverSettings.velocityIterations, solverSettings.positionIterations,
            solverSettings.useWarmStarting ? 1 : 0, static_cast<double>(solverSettings.warmStartFactor),
            solverSettings.useSplitImpulse ? 1 : 0, static_cast<double>(solverSettings.splitImpulseBeta),
            static_cast<double>(solverSettings.linearSlop), solverSettings.useSatCache ? 1 : 0,
            static_cast<double>(world.GetGravity().y), solverSettings.enableSleeping ? 1 : 0,
            solverSettings.useIslandSolving ? 1 : 0);

        // 16 is a readable sample: the Week 6 stress scenes have 210/400 bodies, and
        // their verdict is the aggregate line above (counts + penetration + broad-phase
        // numbers), not another 400 lines of per-body state.
        constexpr int kMaxBodiesPrinted = 16;
        int printedBodyCount = 0;
        for (const p2::Body& body : world.GetBodies()) {
            if (!body.handle.IsValid() || !world.IsValid(body.handle)) {
                continue;
            }
            const p2::Vec2 position = world.GetPosition(body.handle);
            const p2::Vec2 velocity = world.GetLinearVelocity(body.handle);
            // Angle is reported in degrees: the acceptance check for BoxStack is "how
            // far did each box tilt", and radians are hard to judge in a log.
            const float angleDegrees = world.GetAngle(body.handle) * p2::kRadToDeg;
            std::printf("[smoke] body='%s' position=(%.3f, %.3f) velocity=(%.3f, %.3f) angle=%.2fdeg\n",
                        body.debugName.c_str(), position.x, position.y, velocity.x, velocity.y, angleDegrees);
            if (++printedBodyCount >= kMaxBodiesPrinted) {
                break;
            }
        }

        // Contact state: pair names, manifold point count, penetration in mm and the
        // accumulated impulses (summed over the manifold points). A resting tower
        // prints its contacts here, which is how the log tells a standing stack apart
        // from a collapsed one that scattered into free fall; normal_impulse > 0 with
        // |tangent_impulse| <= mu*normal_impulse is the friction-cone evidence.
        constexpr int kMaxContactsPrinted = 16;
        int printedContactCount = 0;
        for (const p2::Contact& contact : world.GetContacts()) {
            if (!contact.manifold.IsTouching()) {
                continue;
            }
            float normalImpulse = 0.0f;
            float tangentImpulse = 0.0f;
            for (int32_t pointIndex = 0; pointIndex < contact.manifold.pointCount; ++pointIndex) {
                normalImpulse += contact.manifold.points[pointIndex].normalImpulse;
                tangentImpulse += contact.manifold.points[pointIndex].tangentImpulse;
            }
            const p2::Body* bodyA = world.GetBody(contact.bodyA);
            const p2::Body* bodyB = world.GetBody(contact.bodyB);
            std::printf("[smoke] contact '%s'|'%s' points=%d penetration=%.3fmm normal=(%.3f, %.3f) "
                        "normal_impulse=%.3f tangent_impulse=%.3f\n",
                        bodyA != nullptr ? bodyA->debugName.c_str() : "?",
                        bodyB != nullptr ? bodyB->debugName.c_str() : "?", contact.manifold.pointCount,
                        contact.manifold.penetration * 1000.0f, contact.manifold.normal.x,
                        contact.manifold.normal.y, normalImpulse, tangentImpulse);
            if (++printedContactCount >= kMaxContactsPrinted) {
                break;
            }
        }

        // Week 7: scene-specific smoke lines (Dominoes chain state, IslandSleep500
        // awake-vs-asleep Step-time comparison). The optional Demo hook keeps main.cpp
        // generic instead of special-casing scene names here.
        reportedDemo.PrintSmokeSummary(world);
    } else if (frameIndex > 0) {
        std::printf("[testbed] closed after %d frames, %d physics steps\n", frameIndex, totalPhysicsSteps);
    }

    // One machine-checkable line for the recording script: count of PPM frames written and
    // the sampling the run used, so the encoder can cross-check GIF length/rate.
    if (captureEnabled) {
        std::printf("[capture] frames=%d every=%d start=%d dir=%s\n", capturedFrameCount,
                    options.captureFrameInterval, options.captureStartFrame,
                    options.captureDirectory.c_str());
    }

    return kExitSuccess;
}

} // namespace

int main(int argc, char** argv) {
    CommandLineOptions options;
    if (!ParseCommandLine(argc, argv, options)) {
        PrintUsage();
        return kExitFailure;
    }
    if (options.helpRequested) {
        PrintUsage();
        return kExitSuccess;
    }
    return RunTestbed(options);
}
