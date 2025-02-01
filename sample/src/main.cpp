#include <mujoco/mujoco.h>
#include <iostream>
#include <cmath>
#include "hako_asset.h"
#include "hako_conductor.h"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64.hpp"

static mjData* mujoco_data;
static mjModel* mujoco_model;
static const char* asset_name = "Mujoco";
static const char* robot_name = "Cube";
static int channel_id = 0;
static int left_wheel_channel_id = 1;
static int right_wheel_channel_id = 2;
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

    const int base_id = mj_name2id(mujoco_model, mjOBJ_BODY, "tb3_base");
    const int left_wheel_id = mj_name2id(mujoco_model, mjOBJ_BODY, "left_wheel");
    const int right_wheel_id = mj_name2id(mujoco_model, mjOBJ_BODY, "right_wheel");

    // 本体（ベース）から見た相対位置
    double left_wheel_rel_x = mujoco_model->body_pos[3 * left_wheel_id];
    double left_wheel_rel_y = mujoco_model->body_pos[3 * left_wheel_id + 1];
    double left_wheel_rel_z = mujoco_model->body_pos[3 * left_wheel_id + 2];
    //std::cout << "left_wheel_rel_x: " << left_wheel_rel_x << ", left_wheel_rel_y: " << left_wheel_rel_y << ", left_wheel_rel_z: " << left_wheel_rel_z << std::endl;

    double right_wheel_rel_x = mujoco_model->body_pos[3 * right_wheel_id];
    double right_wheel_rel_y = mujoco_model->body_pos[3 * right_wheel_id + 1];
    double right_wheel_rel_z = mujoco_model->body_pos[3 * right_wheel_id + 2];

    // 本体のワールド座標
    double base_x = mujoco_data->qpos[0];
    double base_y = mujoco_data->qpos[1];
    double base_z = mujoco_data->qpos[2];

    //std::cout << "base_x: " << base_x << ", base_y: " << base_y << ", base_z: " << base_z << std::endl;
    // 左車輪のワールド座標
    double left_wheel_x = base_x + left_wheel_rel_x;
    double left_wheel_y = base_y + left_wheel_rel_y;
    double left_wheel_z = base_z + left_wheel_rel_z;
    //std::cout << "left_wheel_x: " << left_wheel_x << ", left_wheel_y: " << left_wheel_y << ", left_wheel_z: " << left_wheel_z << std::endl;

    // 右車輪のワールド座標
    double right_wheel_x = base_x + right_wheel_rel_x;
    double right_wheel_y = base_y + right_wheel_rel_y;
    double right_wheel_z = base_z + right_wheel_rel_z;

    mujoco_data->ctrl[0] = 2.0;  // 左モーターに0.5の出力
    mujoco_data->ctrl[1] = 2.0;  // 右モーターに0.5の出力

    //std::cout << "Time: " << mujoco_data->time
    //            << ", x, y, z:  " << mujoco_data->qpos[0] << ", " << mujoco_data->qpos[1] << ", " << mujoco_data->qpos[2] << std::endl;
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

    HakoCpp_Float64 left_wheel_angle;
    HakoCpp_Float64 right_wheel_angle;
    left_wheel_angle.data = mujoco_data->qpos[7];   // 左車輪の角度
    right_wheel_angle.data = mujoco_data->qpos[8];  // 右車輪の角度
#if 0
    std::cout << "Time: " << mujoco_data->time
                << ", x, y, z:  " << twist.linear.x << ", " << twist.linear.y << ", " << twist.linear.z
                << ", roll, pitch, yaw: " << twist.angular.x << ", " << twist.angular.y << ", " << twist.angular.z
                << ", left wheel angle: " << left_wheel_angle
                << ", right wheel angle: " << right_wheel_angle << std::endl;
#endif

    Hako_Twist* raw_data;
    auto _ret = hako_convert_cpp2pdu_Twist(twist, &raw_data);
    if (hako_asset_pdu_write(robot_name, channel_id, (const char*)hako_get_top_ptr_pdu(raw_data), static_cast<size_t>(pdu_size)) != 0) {
        std::cerr << "Failed to write PDU." << std::endl;
    }
    hako_destroy_pdu(raw_data);
    HakoCpp_Twist left_wheel =  {};
    left_wheel.linear.x = left_wheel_x;
    left_wheel.linear.y = left_wheel_y;
    left_wheel.linear.z = left_wheel_z;
    left_wheel.angular.x = roll;
    left_wheel.angular.y = mujoco_data->qpos[7];
    left_wheel.angular.z = yaw;
    Hako_Twist* raw_data_left_wheel;
    _ret = hako_convert_cpp2pdu_Twist(left_wheel, &raw_data_left_wheel);
    if (hako_asset_pdu_write("left_wheel", left_wheel_channel_id, (const char*)hako_get_top_ptr_pdu(raw_data_left_wheel), static_cast<size_t>(pdu_size)) != 0) {
        std::cerr << "Failed to write PDU." << std::endl;
    }
    hako_destroy_pdu(raw_data_left_wheel);

    HakoCpp_Twist right_wheel =  {};
    right_wheel.linear.x = right_wheel_x;
    right_wheel.linear.y = right_wheel_y;
    right_wheel.linear.z = right_wheel_z;
    right_wheel.angular.x = roll;
    right_wheel.angular.y = mujoco_data->qpos[8];
    right_wheel.angular.z = yaw;
    Hako_Twist* raw_data_right_wheel;
    _ret = hako_convert_cpp2pdu_Twist(right_wheel, &raw_data_right_wheel);
    if (hako_asset_pdu_write("right_wheel", right_wheel_channel_id, (const char*)hako_get_top_ptr_pdu(raw_data_right_wheel), static_cast<size_t>(pdu_size)) != 0) {
        std::cerr << "Failed to write PDU." << std::endl;
    }
    hako_destroy_pdu(raw_data_right_wheel);

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
    mujoco_model = mj_loadXML("model/tb3.xml", nullptr, error, sizeof(error));
    if (!mujoco_model) {
        std::cerr << "Failed to load model: " << error << std::endl;
        return 1;
    }

    // データの作成
    mujoco_data = mj_makeData(mujoco_model);

    // 初期化
    mujoco_data->qpos[2] = 0.0;
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
