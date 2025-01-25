#include <mujoco/mujoco.h>
#include <iostream>

int main() {
    // モデルの読み込み
    char error[1000];
    mjModel* model = mj_loadXML("model/cube_drop.xml", nullptr, error, sizeof(error));
    if (!model) {
        std::cerr << "Failed to load model: " << error << std::endl;
        return 1;
    }

    // データの作成
    mjData* data = mj_makeData(model);

    // 初期化
    data->qpos[2] = 100.0;  // キューブの初期高さを設定
    mj_forward(model, data);  // 状態を計算して反映

    // シミュレーション
    for (int i = 0; i < 100; ++i) {
        mj_step(model, data);  // シミュレーションを1ステップ進める
        std::cout << "Step: " << i << ", Time: " << data->time
                  << ", Height: " << data->qpos[2] << std::endl;
    }

    // リソース解放
    mj_deleteData(data);
    mj_deleteModel(model);

    return 0;
}
