#include <SDL2/SDL.h>         // For using SDL2
#include <SDL2/SDL_opengl.h>  // For using OpenGL
#include <imgui.h>            // For using ImGui
#include <imgui_impl_sdl2.h>
#include <imgui_impl_opengl3.h>

#include <iostream>  // For std::cerr


int main()
{
    // Initialize SDL (video subsystem)
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "SDL Init failed: " << SDL_GetError() << std::endl;
        return -1;
    }

    // OpenGL attributes 
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    // Create a window centered on the screen
    SDL_Window* window = SDL_CreateWindow(
        "Robotics Sim GUI",                   // Window title
        SDL_WINDOWPOS_CENTERED,               // X position (centered)
        SDL_WINDOWPOS_CENTERED,               // Y position (centered)
        1000,                                 // Width
        700,                                  // Height
        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN  // Make window visible
    );

    // Check if window creation failed
    if (!window)
    {
        std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return -1;
    }

    // OpenGL context
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); 

    // ImGui setup 
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    bool running = true;
    SDL_Event event;  // Event structure for handling input/events

    while (running)
    {
        // Events
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);

            // Check if the user requested to close the window
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
        }

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        // UI
        ImGui::Begin("Robotics Debugger");

        ImGui::Text("Simulation GUI is running");

        if (ImGui::Button("Test Button"))
        {
            std::cout << "Button clicked!\n";
        }

        ImGui::End();

        ImGui::Render();

        glViewport(0, 0, 1000, 700);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // Clean up resources before exiting
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}