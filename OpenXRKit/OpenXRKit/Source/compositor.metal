/*
** Copyright 2024-2025, Warren Moore
**
** SPDX-License-Identifier: Apache-2.0 OR MIT
*/

#include <metal_stdlib>
using namespace metal;

struct QuadVertex {
    float4 position [[position]];
    float2 texCoords;
};

constexpr sampler linearSampler(coord::normalized, filter::linear, mip_filter::none, address::clamp_to_edge);

template <typename Scalar>
Scalar linear_from_srgb_comp(Scalar c) {
    Scalar higher = powr((c + Scalar { 0.055 } ) / Scalar { 1.055 }, Scalar { 2.4 });
    Scalar lower = c / Scalar { 12.92 };
    return mix(lower, higher, step(Scalar { 0.04045 }, c));
}

template <typename Vector3>
Vector3 linear_from_srgb(Vector3 rgb) {
    return Vector3 { linear_from_srgb_comp(rgb.r), linear_from_srgb_comp(rgb.g), linear_from_srgb_comp(rgb.b) };
}

[[vertex]]
QuadVertex compositor_vertex(uint vertexID [[vertex_id]],
                             constant float3x3 &displayTransform [[buffer(0)]])
{
    float2 positions[] = { { -1.0f, 1.0f }, { -1.0f, -3.0f }, { 3.0f, 1.0f } };
    float2 position = positions[vertexID];
    float2 texCoords = position * float2(0.5f, -0.5f) + float2(0.5f, 0.5f);
    QuadVertex out;
    out.position = float4(position, 0.0f, 1.0f);
    out.texCoords = (displayTransform * float3(texCoords, 1.0f)).xy;
    return out;
}

[[fragment]]
half4 compositor_fragment_rgb(QuadVertex in [[stage_in]],
                              texture2d<half, access::sample> colorTexture)
{
    half4 color = colorTexture.sample(linearSampler, in.texCoords);
    return color;
}

[[fragment]]
half4 compositor_fragment_ycbcr(QuadVertex in [[stage_in]], 
                                texture2d<half, access::sample> yTexture,
                                texture2d<half, access::sample> cbcrTexture)
{
    half Y = yTexture.sample(linearSampler, in.texCoords).x;
    half2 chrominance = cbcrTexture.sample(linearSampler, in.texCoords).xy;
    half Cb = chrominance[0], Cr = chrominance[1];
    // Ref. https://www.itu.int/rec/T-REC-T.871-201105-I/en (p.4)
    // Ref. https://developer.apple.com/forums/thread/126632
    half r = Y                + 1.402h  * Cr - 0.701h;
    half g = Y - 0.3441h * Cb - 0.7141h * Cr + 0.5291h;
    half b = Y + 1.722h  * Cb - 0.886h;
    half3 rgb { r, g, b };
    return half4(rgb, 1.0h);
}
