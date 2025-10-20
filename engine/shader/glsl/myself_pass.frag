#version 310 es

#extension GL_GOOGLE_include_directive : enable

#include "constants.h"

layout(input_attachment_index = 0, set = 0, binding = 0) uniform highp subpassInput in_color;

layout(set = 0, binding = 1) uniform sampler2D color_grading_lut_texture_sampler;

layout(location = 0) out highp vec4 out_color;

void main() {
    highp ivec2 lut_tex_size = textureSize(color_grading_lut_texture_sampler, 0);
    highp float _COLORS = float(lut_tex_size.y);
    highp float _Nums = float(lut_tex_size.x) / _COLORS;

    highp vec4 color = subpassLoad(in_color).rgba;

    highp float blue_index = color.b * _Nums;
    highp float green_index = color.g;
    highp float red_index = color.r;

    highp vec2 uv_0 = vec2((floor(blue_index) + red_index) / _Nums, green_index);
    highp vec2 uv_1 = vec2((floor(blue_index + 1.0) + red_index) / _Nums, green_index);
    highp vec4 color_sample_0 = texture(color_grading_lut_texture_sampler, uv_0);
    highp vec4 color_sample_1 = texture(color_grading_lut_texture_sampler, uv_1);
    out_color = mix(color_sample_0, color_sample_1, fract(blue_index));
    //out_color = mix(color, vec4(0.0), 0.0);
}
