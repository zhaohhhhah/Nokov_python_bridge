#include "NokovSDKClient.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

static_assert(sizeof(float) == 4, "This bridge expects 32-bit float");

extern "C" {

struct NokovFrameInfo
{
    std::int64_t timestamp;
    std::int32_t frame_number;
    std::int32_t rigid_body_count;
    std::int32_t frame_params;
    std::uint32_t timecode;
    std::uint32_t timecode_subframe;
    float latency;
};

struct NokovRigidBody
{
    std::int32_t id;
    std::int32_t has_extend;
    std::int32_t params;

    float x;
    float y;
    float z;

    float qx;
    float qy;
    float qz;
    float qw;

    float mean_error;

    float roll;
    float pitch;
    float yaw;

    float roll_vel;
    float pitch_vel;
    float yaw_vel;

    float roll_acc;
    float pitch_acc;
    float yaw_acc;

    float vx;
    float vy;
    float vz;
    float speed;

    float ax;
    float ay;
    float az;
    float acc;
};

}

namespace
{
std::mutex g_mutex;
NokovSDKClient* g_client = nullptr;
std::string g_server_ip;
NokovFrameInfo g_frame_info{};
std::vector<NokovRigidBody> g_bodies;
bool g_have_frame = false;

void DataCallback(sFrameOfMocapData* frame, void* /*user_data*/)
{
    if (frame == nullptr)
        return;

    const int body_count = std::clamp(frame->nRigidBodies, 0, MAX_RIGIDBODIES);

    std::vector<NokovRigidBody> local_bodies(static_cast<std::size_t>(body_count));
    std::unordered_map<int, int> id_to_index;
    id_to_index.reserve(static_cast<std::size_t>(body_count));

    for (int i = 0; i < body_count; ++i)
    {
        const sRigidBodyData& src = frame->RigidBodies[i];
        NokovRigidBody& dst = local_bodies[static_cast<std::size_t>(i)];

        dst = NokovRigidBody{};
        dst.id = src.ID;
        dst.params = static_cast<std::int32_t>(src.params);

        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;

        dst.qx = src.qx;
        dst.qy = src.qy;
        dst.qz = src.qz;
        dst.qw = src.qw;

        dst.mean_error = src.MeanError;

        id_to_index[src.ID] = i;
    }

    const int extend_count = std::clamp(
        frame->extendData.nExtendDataNum,
        0,
        static_cast<int>(ExtendDataEndType)
    );

    for (int k = 0; k < extend_count; ++k)
    {
        const sExtendData& ext = frame->extendData.extendData[k];
        if (ext.type != ExtendDataRigidBody)
            continue;

        const int n = std::clamp(ext.number, 0, MAX_RIGIDBODIES);

        for (int i = 0; i < n; ++i)
        {
            const sRigidBodyExtendData& src = ext.extendData.rigidBodyExtend[i];
            const auto it = id_to_index.find(src.ID);
            if (it == id_to_index.end())
                continue;

            NokovRigidBody& dst = local_bodies[static_cast<std::size_t>(it->second)];
            dst.has_extend = 1;

            dst.roll = src.roll;
            dst.pitch = src.pitch;
            dst.yaw = src.yaw;

            dst.roll_vel = src.rollVel;
            dst.pitch_vel = src.pitchVel;
            dst.yaw_vel = src.yawVel;

            dst.roll_acc = src.rollAvel;
            dst.pitch_acc = src.pitchAvel;
            dst.yaw_acc = src.yawAvel;

            dst.vx = src.xVel;
            dst.vy = src.yVel;
            dst.vz = src.zVel;
            dst.speed = src.rVel;

            dst.ax = src.xAvel;
            dst.ay = src.yAvel;
            dst.az = src.zAvel;
            dst.acc = src.rAvel;
        }
    }

    NokovFrameInfo local_info{};
    local_info.timestamp = static_cast<std::int64_t>(frame->iTimeStamp);
    local_info.frame_number = static_cast<std::int32_t>(frame->iFrame);
    local_info.rigid_body_count = static_cast<std::int32_t>(body_count);
    local_info.frame_params = static_cast<std::int32_t>(frame->params);
    local_info.timecode = static_cast<std::uint32_t>(frame->Timecode);
    local_info.timecode_subframe = static_cast<std::uint32_t>(frame->TimecodeSubframe);
    local_info.latency = frame->fLatency;

    std::lock_guard<std::mutex> lock(g_mutex);
    g_frame_info = local_info;
    g_bodies.swap(local_bodies);
    g_have_frame = true;
}
}

extern "C" {

int nokov_connect(const char* server_ip)
{
    if (server_ip == nullptr || server_ip[0] == '\0')
        return -1001;

    if (g_client != nullptr)
        return 0;

    {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_have_frame = false;
        g_frame_info = NokovFrameInfo{};
        g_bodies.clear();
    }

    g_server_ip = server_ip;
    g_client = new NokovSDKClient();

    const int callback_ret = g_client->SetDataCallback(DataCallback, nullptr);
    if (callback_ret != 0)
    {
        delete g_client;
        g_client = nullptr;
        return -1002;
    }

    const int init_ret = g_client->Initialize(g_server_ip.data());
    if (init_ret != 0)
    {
        delete g_client;
        g_client = nullptr;
        return init_ret;
    }

    return 0;
}

int nokov_get_latest(
    NokovFrameInfo* frame_info,
    NokovRigidBody* output,
    int max_bodies
)
{
    if (frame_info == nullptr)
        return -1003;

    if (max_bodies < 0)
        return -1004;

    if (max_bodies > 0 && output == nullptr)
        return -1005;

    std::lock_guard<std::mutex> lock(g_mutex);

    if (!g_have_frame)
        return 0;

    *frame_info = g_frame_info;

    const int count = std::min(
        static_cast<int>(g_bodies.size()),
        max_bodies
    );

    for (int i = 0; i < count; ++i)
        output[i] = g_bodies[static_cast<std::size_t>(i)];

    return count;
}

int nokov_has_frame()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_have_frame ? 1 : 0;
}

void nokov_disconnect()
{
    NokovSDKClient* client = g_client;
    g_client = nullptr;

    if (client != nullptr)
    {
        client->Uninitialize();
        delete client;
    }

    std::lock_guard<std::mutex> lock(g_mutex);
    g_have_frame = false;
    g_frame_info = NokovFrameInfo{};
    g_bodies.clear();
    g_server_ip.clear();
}

}
