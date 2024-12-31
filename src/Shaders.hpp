#pragma once

namespace Shaders {

const char* const VERTEX_SHADER_SOURCE = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;
    out vec3 Color;
    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
        Color = aColor;
        gl_PointSize = 5.0;
    }
)";

const char* const FRAGMENT_SHADER_SOURCE = R"(
    #version 330 core
    in vec3 Color;
    out vec4 FragColor;
    void main() {
        FragColor = vec4(Color, 1.0);
    }
)";

const char* const TEXT_VERTEX_SHADER_SOURCE = R"(
    #version 330 core
    layout (location = 0) in vec4 vertex; // <vec2 pos, vec2 tex>
    out vec2 TexCoords;
    
    uniform mat4 projection;
    
    void main() {
        gl_Position = projection * vec4(vertex.xy, 0.0, 1.0);
        TexCoords = vertex.zw;
    }
)";

const char* const TEXT_FRAGMENT_SHADER_SOURCE = R"(
    #version 330 core
    in vec2 TexCoords;
    out vec4 color;
    
    uniform sampler2D text;
    uniform vec3 textColor;
    
    void main() {
        vec4 sampled = vec4(1.0, 1.0, 1.0, texture(text, TexCoords).r);
        color = vec4(textColor, 1.0) * sampled;
    }
)";

} // namespace Shaders 