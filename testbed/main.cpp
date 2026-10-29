// physics2d_testbed - interactive testbed entry point (Week 1 D5/D6; contact
// overlay in Week 2; selectable scenes in Weeks 3-4; Week 5 adds the Solver and
// Contacts panels, the clickable contact highlight and the FrictionRamp scene).
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
//      experiments: e.g. friction 0 on BoxStack). The solver settings themselves are
//      written by DebugUI straight into World::SetSolverSettings.
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
//               FrictionRamp, default FallingBall). Matching is case-insensitive and
//               ignores spaces, underscores and hyphens, so "spherestack" and
//               "Sphere Stack" both work. An unknown name prints the available list
//               and exits with code 2 so smoke automation can distinguish a typo
//               from a physics failure.
//   --smoke N   render N frames, print a machine-checkable summary, exit 0.
//               In smoke mode the loop feeds the accumulator a deterministic
//               1/60 s per frame, so N frames == N physics steps regardless of
//               machine speed. Exit code 2 means "no usable GL environment".
//   --hidden    create the window with GLFW_VISIBLE=false (used by smoke runs).
//   --help      print usage.

#include "Camera.h"
#include "DebugUI.h"
#include "GLLoader.h"
#include "Renderer.h"
#include "demos/BoxStack.h"
#include "demos/FallingBall.h"
#include "demos/FrictionRamp.h"
#include "demos/SphereStack.h"

#include "physics2d/Core/Settings.h"
#include "physics2d/Core/TimeStep.h"
#include "physics2d/Core/World.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

struct CommandLineOptions {
    int smokeFrames = 0; // 0 means interactive mode
    bool hidden = false;
    bool helpRequested = false;
    std::string demoName = "FallingBall"; // scene id, matched against Demo::Name()
};

// The binary name is spelled out instead of using argv[0]: the repository path may
// contain non-ASCII characters, and printing argv[0] to a GBK/UTF-8 mixed console
// turns the usage line into mojibake.
void PrintUsage() {
    std::printf("Usage: physics2d_testbed [--demo NAME] [--smoke N] [--hidden] [--help]\n"
                "  --demo NAME start in this scene (FallingBall | SphereStack | BoxStack |\n"
                "              FrictionRamp; default FallingBall).\n"
                "              Matching is case-insensitive and ignores spaces/underscores.\n"
                "  --smoke N   render N frames, print a summary and exit (0 = pass, 2 = no GL environment\n"
                "              or unknown demo name)\n"
                "  --hidden    hide the window (rendering still happens)\n"
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
        } else if (std::strcmp(argument, "--help") == 0 || std::strcmp(argument, "-h") == 0) {
            options.helpRequested = true;
            return true;
        } else {
            std::fprintf(stderr, "[testbed] unknown argument: '%s'\n", argument);
            return false;
        }
    }
    return true;
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

    // ---- demo registry + CLI scene selection --------------------------------
    // Built before GLFW/GL: an unknown --demo is a configuration error and must be
    // reported (exit code 2 by contract) even on a machine that cannot open a window.
    std::vector<std::unique_ptr<p2::Demo>> demos;
    demos.push_back(std::make_unique<p2::FallingBallDemo>());
    demos.push_back(std::make_unique<p2::SphereStackDemo>());
    demos.push_back(std::make_unique<p2::BoxStackDemo>());
    demos.push_back(std::make_unique<p2::FrictionRampDemo>());
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
    // (Week 5 solver experiments) are applied uniformly to the initial scene and to
    // every later scene switch.
    p2::DebugUI debugUi;
    p2::DebugUI::DisplayOptions displayOptions;
    demos[static_cast<std::size_t>(selectedDemo)]->Setup(world);
    ApplyMaterialDefaults(world, debugUi.GetMaterialDefaults());

    using Clock = std::chrono::steady_clock;
    Clock::time_point previousFrameTime = Clock::now();
    float smoothedFrameMilliseconds = 0.0f;
    float smoothedStepMilliseconds = 0.0f;
    int32_t physicsStepsLastFrame = 0;
    int frameIndex = 0;
    int32_t totalPhysicsSteps = 0;
    double totalFrameMilliseconds = 0.0;

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
        // render below shows through it while the panels dock at the edges.
        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        const ImGuiIO& io = ImGui::GetIO();
        camera.UpdateInput(window, !io.WantCaptureMouse && !io.WantCaptureKeyboard);

        int windowWidth = kInitialWindowWidth;
        int windowHeight = kInitialWindowHeight;
        glfwGetWindowSize(window, &windowWidth, &windowHeight);
        camera.SetViewportSize(windowWidth, windowHeight);

        p2::DebugUI::FrameStats stats;
        stats.framesPerSecond = smoothedFrameMilliseconds > 0.0f ? 1000.0f / smoothedFrameMilliseconds : 0.0f;
        stats.frameMilliseconds = smoothedFrameMilliseconds;
        stats.physicsStepMilliseconds = smoothedStepMilliseconds;
        stats.physicsSteps = physicsStepsLastFrame;

        const p2::DebugUI::FrameActions actions =
            debugUi.Draw(world, *demos[static_cast<std::size_t>(selectedDemo)], demoNames, selectedDemo,
                         stats, displayOptions);
        if (actions.quitRequested) {
            break;
        }
        if (actions.demoChanged) {
            // Switching scenes rebuilds the World from scratch: leftover bodies
            // and accumulated time from the previous scene must not leak in.
            world.Clear();
            demos[static_cast<std::size_t>(selectedDemo)]->Setup(world);
            accumulator.Reset();
            // Week 5: a newly built scene picks up the panel's material defaults
            // (friction/restitution experiments). Reset() deliberately does not: a
            // demo's own Reset keeps the material state the user tuned in its Ui.
            ApplyMaterialDefaults(world, debugUi.GetMaterialDefaults());
        }
        if (actions.applyMaterialDefaultsRequested) {
            ApplyMaterialDefaults(world, debugUi.GetMaterialDefaults());
        }
        if (actions.resetRequested) {
            demos[static_cast<std::size_t>(selectedDemo)]->Reset(world);
        }
        // Re-bind after the actions: the combo box may have changed the selection,
        // and stepping/rendering must use the new scene's demo object.
        p2::Demo& currentDemo = *demos[static_cast<std::size_t>(selectedDemo)];

        // ---- physics ---------------------------------------------------------
        // In smoke mode every frame simulates exactly one fixed step, keeping the
        // automated check deterministic: 120 frames always means 120 steps.
        const float simulationFrameSeconds = smokeMode ? p2::kFixedTimeStep : frameMilliseconds * 0.001f;
        int32_t stepsThisFrame = 0;
        if (actions.singleStepRequested) {
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
            // Week 2: Synchronize() fills Body::aabb from the attached shapes. The
            // infinite plane's AABB is deliberately enormous and lies outside the
            // view, so only finite shapes (the ball) show a visible box. The
            // degeneracy guard keeps bodies without a shape from littering the origin.
            for (const p2::Body& body : world.GetBodies()) {
                if (!body.handle.IsValid() || !world.IsValid(body.handle)) {
                    continue;
                }
                if (body.aabb.GetWidth() <= 0.0f && body.aabb.GetHeight() <= 0.0f) {
                    continue;
                }
                renderer.DrawAABB(body.aabb, p2::Color{0.45f, 0.75f, 0.45f, 0.6f});
            }
        }
        renderer.End();

        // UI is drawn after the world so panels overlay the scene; both share the
        // same framebuffer, cleared once above.
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
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

        // One machine-readable line for automated verification: scene id, counts,
        // penetration and timing. The per-body / per-contact lines below carry the
        // state a verdict actually needs (final heights and velocities, which pairs
        // still touch, and the accumulated solver impulses).
        std::printf("[smoke] demo=%s frames=%d physics_steps=%d bodies=%d shapes=%d contacts=%d "
                    "contact_points=%d max_penetration_mm=%.3f avg_frame_ms=%.3f\n",
                    compactDemoName.c_str(), frameIndex, totalPhysicsSteps, world.GetBodyCount(),
                    world.GetShapeCount(), touchingContactCount, contactPointCount, maxPenetration * 1000.0f,
                    averageFrameMilliseconds);

        // The solver configuration the run actually used: quoting smoke numbers
        // without these would make a warm-start / split-impulse comparison meaningless.
        const p2::SolverSettings& solverSettings = world.GetSolverSettings();
        std::printf(
            "[smoke] solver velocity_iterations=%d position_iterations=%d warm_starting=%d "
            "warm_start_factor=%.2f split_impulse=%d split_impulse_beta=%.2f linear_slop_m=%.4f "
            "sat_cache=%d gravity_y=%.1f\n",
            solverSettings.velocityIterations, solverSettings.positionIterations,
            solverSettings.useWarmStarting ? 1 : 0, static_cast<double>(solverSettings.warmStartFactor),
            solverSettings.useSplitImpulse ? 1 : 0, static_cast<double>(solverSettings.splitImpulseBeta),
            static_cast<double>(solverSettings.linearSlop), solverSettings.useSatCache ? 1 : 0,
            static_cast<double>(world.GetGravity().y));

        // 16 bodies / 16 contacts covers the largest current scene (BoxStack: 11 bodies,
        // 10 contacts) without turning the smoke log into a wall of text.
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
    } else if (frameIndex > 0) {
        std::printf("[testbed] closed after %d frames, %d physics steps\n", frameIndex, totalPhysicsSteps);
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
