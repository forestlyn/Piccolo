#version 310 es

#extension GL_GOOGLE_include_directive : enable

#include "constants.h"

layout(input_attachment_index = 0, set = 0, binding = 0) uniform highp subpassInput in_color;

layout(location = 0) out highp vec4 outColor;

void main() {
    // 只使用当前像素的简单FXAA
    highp vec4 center_color = subpassLoad(in_color);
    highp vec3 color = center_color.rgb;

    // 使用导数进行边缘检测（不需要采样周围像素）
    highp vec3 dx = dFdx(color);
    highp vec3 dy = dFdy(color);
    highp float edge = length(dx * dx + dy * dy);

    // 简单的抗锯齿处理
    highp float blend_factor = clamp(edge * 3.0, 0.0, 0.3);

    // 轻微模糊来抗锯齿
    outColor = vec4(color * (1.0 - blend_factor) + vec3(0.5) * blend_factor, 1.0);
}