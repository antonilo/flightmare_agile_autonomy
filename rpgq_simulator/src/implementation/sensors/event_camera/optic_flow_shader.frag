#version 330 core

#define M_PI 3.1415926535897932384626433832795

out vec4 FragColor;

// angular and linear camera velocity
uniform vec3 C_v_C;
uniform vec3 C_omega_WC;

// angular and linear object velocity
uniform bool dynamic_object;
uniform vec3 C_r_CK;
uniform vec3 C_v_K;
uniform vec3 C_omega_WK;

// camera intrinsics
uniform float fx;
uniform float fy;
uniform float u0;
uniform float v0;
uniform float width;
uniform float height;

// depth range used
uniform float near;
uniform float far;

// taken from: https://learnopengl.com/Advanced-OpenGL/Depth-testing
float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // back to NDC
    return (2.0 * near * far) / (far + near - z * (far - near));
}

void main()
{
    float depth = LinearizeDepth(gl_FragCoord.z);
    float inv_z = 1.0f / depth;

    // calibrated coordinates
    float u = (gl_FragCoord.x - u0) / fx;
    float v = (gl_FragCoord.y - v0) / fy;

    // optic flow computation
    vec2 flow;
    if (dynamic_object)
    {
        float x = u*depth;
        float y = v*depth;
        float x_obj = x - C_r_CK.x;
        float y_obj = y - C_r_CK.y;
        float z_obj = depth - C_r_CK.z; // TODO, should not be done in loop, remains constant for object
        float vx = C_v_K.x - C_v_C.x - (C_omega_WC.y * depth - C_omega_WC.z * y) + (C_omega_WK.y * z_obj - C_omega_WK.z * y_obj);
        float vy = C_v_K.y - C_v_C.y - (C_omega_WC.z * x - C_omega_WC.x * depth) + (C_omega_WK.z * x_obj - C_omega_WK.x * z_obj);
        float vz = C_v_K.z - C_v_C.z - (C_omega_WC.x * y - C_omega_WC.y * x) + (C_omega_WK.x * y_obj - C_omega_WK.y * x_obj);

        flow = vec2(fx*(inv_z*vx - u*inv_z*vz), fy*(inv_z*vy - v*inv_z*vz));
    }
    else
    {
        // Eq. (3) in https://arxiv.org/pdf/1510.01972.pdf
        float vx_trans = -inv_z * C_v_C.x
                        + u * inv_z * C_v_C.z;

        float vx_rot = u * v * C_omega_WC.x
                    - (1.0 + u * u) * C_omega_WC.y
                    + v * C_omega_WC.z;

        float vy_trans = -inv_z * C_v_C.y
                        + v * inv_z * C_v_C.z;

        float vy_rot = (1.0 + v * v) * C_omega_WC.x
                    - u * v * C_omega_WC.y
                    - u * C_omega_WC.z;

        float vx = vx_trans + vx_rot;
        float vy = vy_trans + vy_rot;

        flow = vec2(fx * vx, fy * vy);
    }

    // red component: x component of the flow
    // green component: y component of the flow
    FragColor = vec4(flow.x, flow.y, 0.0f, 1.0f);
}
