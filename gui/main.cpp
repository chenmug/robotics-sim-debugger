#include <SDL2/SDL.h>  // For using SDL2
#include <GL/gl.h>     // For using OpenGL

#include "imgui.h"                        // For using ImGui
#include "backends/imgui_impl_sdl2.h"     // For using backends
#include "backends/imgui_impl_opengl3.h"  // For using backends

#include <iostream>    // For std::cerr


int main(int argc, char** argv)
{
    // =========================
    // Initialize SDL
    // =========================
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "SDL Init failed: " << SDL_GetError() << std::endl;

        return -1;
    }

    // =========================
    // Configure OpenGL
    // =========================
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                        SDL_GL_CONTEXT_PROFILE_CORE);

    // =========================
    // Create Window
    // =========================
    SDL_Window* window = SDL_CreateWindow(
        "Robotics Simulation Debugger",           // Window title
        SDL_WINDOWPOS_CENTERED,                   // X position (centered)
        SDL_WINDOWPOS_CENTERED,                   // Y position (centered)
        1280,                                     // Width
        720,                                      // Height
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
    );

    // Check if window creation failed
    if (!window)
    {
        std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }

    // =========================
    // Create OpenGL Context
    // =========================
    SDL_GLContext glContext = SDL_GL_CreateContext(window);

    if (!glContext)
    {
        std::cerr << "OpenGL context creation failed: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();

        return -1;
    }

    SDL_GL_SetSwapInterval(1);

    // =========================
    // Initialize ImGui
    // =========================
    IMGUI_CHECKVERSION();

    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForOpenGL(window, glContext);
    ImGui_ImplOpenGL3_Init("#version 330");

    // =========================
    // Main Loop
    // =========================
    bool running = true;

    SDL_Event event;

    while (running)
    {
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);

            // Check if the user requested to close the window
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
        }

        // =========================
        // Start ImGui Frame
        // =========================
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // Docking space
        ImGui::DockSpaceOverViewport(
            ImGui::GetID("MainDockSpace"),
            ImGui::GetMainViewport()
        );

        // =========================
        // Debugger Window
        // =========================
        ImGui::Begin("Robotics Simulation Debugger");
        ImGui::Text("GUI initialized successfully!");
        ImGui::Separator();
        ImGui::Text("Tick: %d", 0);

        if (ImGui::Button("Step Forward"))
        {
            std::cout << "Step Forward\n";
        }

        ImGui::End();

        // =========================
        // Render
        // =========================
        ImGui::Render();

        int width, height;
        SDL_GetWindowSize(window, &width, &height);

        glViewport(0, 0, width, height);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Multi-viewport support
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            SDL_Window* backupCurrentWindow = SDL_GL_GetCurrentWindow();
            SDL_GLContext backupCurrentContext = SDL_GL_GetCurrentContext();

            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();

            SDL_GL_MakeCurrent(backupCurrentWindow,
                               backupCurrentContext);
        }

        SDL_GL_SwapWindow(window);
    }

    // =========================
    // Cleanup
    // =========================
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();

    ImGui::DestroyContext();

    SDL_GL_DeleteContext(glContext);

    SDL_DestroyWindow(window);

    SDL_Quit();

    return 0;
}