#include <mujoco/mujoco.h>
#include <iostream>
#include <cmath>
#include "hako_asset.h"
#include "hako_conductor.h"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"

static mjData* mujoco_data;
static mjModel* mujoco_model;
static const char* asset_name = "Mujoco";
static const char* robot_name = "Cube";
static int channel_id = 0;
static int pdu_size = 72;

static void quaternionToEuler(double qw, double qx, double qy, double qz, double& roll, double& pitch, double& yaw) {
    // オイラー角をクォータニオンから計算する
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // ピッチが±90度のときは値を制限
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}
static int my_on_initialize(hako_asset_context_t* context)
{
    (void)context;
    return 0;
}
static int my_on_reset(hako_asset_context_t* context)
{
    (void)context;
    return 0;
}

static int my_on_simulation_step(hako_asset_context_t* context)
{
    (void)context;
    mj_step(mujoco_model, mujoco_data);  // シミュレーションを1ステップ進める
    std::cout << "Time: " << mujoco_data->time
                << ", Height: " << mujoco_data->qpos[2] << std::endl;
    double qw = mujoco_data->qpos[3];
    double qx = mujoco_data->qpos[4];
    double qy = mujoco_data->qpos[5];
    double qz = mujoco_data->qpos[6];

    double roll, pitch, yaw;
    quaternionToEuler(qw, qx, qy, qz, roll, pitch, yaw);

    HakoCpp_Twist twist;
    twist.linear.x = mujoco_data->qpos[0];
    twist.linear.y = mujoco_data->qpos[1];
    twist.linear.z = mujoco_data->qpos[2];
    twist.angular.x = roll;
    twist.angular.y = pitch;
    twist.angular.z = yaw;

    Hako_Twist* raw_data;
    auto _ret = hako_convert_cpp2pdu_Twist(twist, &raw_data);
    if (hako_asset_pdu_write(robot_name, channel_id, (const char*)hako_get_top_ptr_pdu(raw_data), static_cast<size_t>(pdu_size)) != 0) {
        std::cerr << "Failed to write PDU." << std::endl;
    }
    hako_destroy_pdu(raw_data);
    return 0;
}

static hako_asset_callbacks_t my_callback = {
    .on_initialize = my_on_initialize,
    .on_manual_timing_control = nullptr,
    .on_simulation_step = my_on_simulation_step,
    .on_reset = my_on_reset
};

int main(int argc, const char* argv[]) 
{
    if (argc != 2) {
        printf("Usage: %s <config_path>\n", argv[0]);
        return 1;
    }
    // モデルの読み込み
    char error[1000];
    mujoco_model = mj_loadXML("model/cube_drop.xml", nullptr, error, sizeof(error));
    if (!mujoco_model) {
        std::cerr << "Failed to load model: " << error << std::endl;
        return 1;
    }

    // データの作成
    mujoco_data = mj_makeData(mujoco_model);

    // 初期化
    mujoco_data->qpos[2] = 100.0;  // キューブの初期高さを設定
    mj_forward(mujoco_model, mujoco_data);  // 状態を計算して反映

    const char* config_path = argv[1];
    hako_time_t delta_time_usec = 20000;
    hako_conductor_start(delta_time_usec, 100000);
    int ret = hako_asset_register(asset_name, config_path, &my_callback, delta_time_usec, HAKO_ASSET_MODEL_PLANT);
    if (ret != 0) {
        printf("ERORR: hako_asset_register() returns %d.", ret);
        return 1;
    }
    ret = hako_asset_start();

    // リソース解放
    mj_deleteData(mujoco_data);
    mj_deleteModel(mujoco_model);
    hako_conductor_stop();
    return 0;
}
