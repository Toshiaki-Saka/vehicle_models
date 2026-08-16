# vehicle_models

日本語ドキュメント: [`docs_ja/`](docs_ja) ／ English documentation: [`docs_en/`](docs_en)

車両運動モデルのヘッダオンリー C++17 ライブラリ。二輪モデル（キネマティック／ダイナミック）、アッカーマン幾何、4輪ダブルトラックモデル、タイヤモデル、線形ハンドリング解析を、外部依存なしで一式提供します。

- **依存ライブラリなし**（標準ライブラリのみ）。Eigen 不要
- **ヘッダオンリー**。`add_subdirectory` / `FetchContent` / `find_package` のいずれでも利用可
- Windows(MSVC) / Linux(GCC, Clang) / macOS、C++17 および C++20 でビルド確認済み
- `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror` で警告ゼロ
- 単体テストは解析解との突き合わせ（定常円旋回、収束次数、荷重移動の総和）で検証

---

## 収録モデル

| モデル | 状態数 | 状態量 | 用途と妥当性 |
|---|---|---|---|
| `UnicycleModel` | 3 | x, y, ψ | スキッドステア、プランナ側の抽象。その場旋回可（アッカーマン車両には不適） |
| `DifferentialDriveModel` | 3 | x, y, ψ | 車輪角速度入力。`toBodyVelocity` / `toWheelRates` で相互変換 |
| `KinematicBicycleModel` | 4 | x, y, ψ, v | 基準点を後軸／重心／前軸から選択。横加速度が概ね 0.4 g 以下で妥当 |
| `KinematicBicycleSteerModel` | 5 | + δ | 操舵アクチュエータの一次遅れ＋レート制限つき。制御器の実機検証向け |
| `LinearLateralBicycleModel` | 5 | x, y, ψ, v_y, r | 定常 v_x での線形2自由度。A/B 行列を直接取得でき、LQR/MPC の設計プラント |
| `DynamicBicycleModel<Tire>` | 6 | x, y, ψ, v_x, v_y, r | 非線形単輪。タイヤモデルをテンプレートで差し替え |
| `BlendedBicycleModel<Tire>` | 6 | 同上 | 低速でキネマティックへブレンド。低速シャトル／バレー領域の特異点対策 |
| `DoubleTrackModel<Tire>` | 6 | 同上 | 4輪独立。前後・左右荷重移動、輪別アッカーマン舵角、複合スリップ |

タイヤモデル：`LinearTire`（線形＋飽和）、`FialaTire`（ブラシ、3次立ち上がり）、`PacejkaTire`（Magic Formula 純横力）。摩擦楕円 `frictionEllipseScale` 付き。

幾何・解析ユーティリティ：

- `ackermann.hpp` — 内外輪舵角、旋回半径、輪別車輪速、アッカーマン誤差、ハンドル角換算、最小回転半径
- `linear_analysis.hpp` — アンダーステア勾配 K、特性速度、限界速度、ニュートラルステアポイント、スタティックマージン、ヨーレートゲイン、定常円旋回、ヨーモードの固有値・減衰比

---

## ビルド

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Windows（Visual Studio）:

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

オプション：`VEHICLE_MODELS_BUILD_TESTS` / `VEHICLE_MODELS_BUILD_EXAMPLES` / `VEHICLE_MODELS_INSTALL` / `VEHICLE_MODELS_WERROR`。サブプロジェクトとして取り込んだ場合は既定で OFF になります。

### 他プロジェクトからの利用

```cmake
# 1) サブディレクトリとして
add_subdirectory(third_party/vehicle_models)

# 2) インストール済みパッケージとして
find_package(vehicle_models 0.1 REQUIRED)

target_link_libraries(my_app PRIVATE vehicle_models::vehicle_models)
```

ROS 2 パッケージに入れる場合も、依存が標準ライブラリのみなので `include/` をそのまま同梱できます。

---

## 使い方

```cpp
#include "vehicle_models/vehicle_models.hpp"
using namespace vehicle_models;

VehicleParameters p = makeShuttleParameters();   // 低速シャトルのプリセット
for (const auto& e : p.validate()) std::cerr << e << "\n";  // 設定値の妥当性検査

// キネマティック二輪（後軸基準）を 10 ms 周期で回す
KinematicBicycleModel model(p, ReferencePoint::RearAxle);
auto x = KinematicBicycleState::make(0.0, 0.0, 0.0, 4.0);   // x, y, yaw, v
const auto u = KinematicBicycleInput::make(0.0, deg2rad(8.0));  // accel, steer
x = step(model, x, u, 0.01);                     // 既定は RK4

// 非線形単輪モデル（Fiala タイヤ）
DynamicBicycleModel<tire::FialaTire> dyn(p);
auto s = DynamicBicycleState::make(0, 0, 0, 15.0, 0, 0);
s = step(dyn, s, dyn.inputFromAcceleration(0.0, deg2rad(3.0)), 0.01);
const auto f = dyn.computeForces(s, DynamicBicycleInput::make(0.0, deg2rad(3.0)));
// f.slip_front, f.fy_front, f.fz_front, f.ay ... 妥当性監視やログに使える中間量

// アッカーマン幾何
const auto g = AckermannGeometry::from(p);
const auto wheels = roadWheelAngles(g, deg2rad(20.0));   // 内外輪舵角
const auto speeds = wheelSpeeds(g, 5.0, 0.3);            // 4輪の車輪速

// 線形ハンドリング解析（解析解）
const double K = analysis::understeerGradientDegPerG(p); // [deg/g]
const auto ss = analysis::steadyStateCornering(p, 15.0, deg2rad(2.0));
```

積分器は `stepEuler` / `stepHeun` / `stepRK4`、または `step(..., IntegratorType::RK4)`。`simulate(model, x0, u, duration, dt)` で固定入力の一括積分もできます。モデル側が満たすべき要件は「`State` / `Input` 型、`derivative()`、`normalizeState()`」の3点のみなので、独自モデルを足しても同じ積分器がそのまま使えます。

---

## 実行例

```bash
./build/handling_report   # 3プリセットの操舵幾何表とハンドリング諸元
./build/step_steer > step_steer.csv   # ステップ操舵をキネマティック／単輪／ダブルトラックで同時実行
```

`handling_report` の出力例（乗用車プリセット）:

```
understeer gradient : +0.0034 rad/(m/s^2)  (+1.917 deg/g)
static margin       : +0.1056
characteristic speed: 28.13 m/s (101.3 km/h)
```

`step_steer` は 20 m/s・3 deg のステップ操舵で、キネマティックモデルがヨーレートを約 1.5 倍過大評価することを示します（0.388 vs 0.248 rad/s）。モデル選択が効いてくる境界がそのまま数字で見えます。

---

## Python シミュレーション GUI

同じモデル群を Python に移植し、GUI シミュレータを [`python/`](python) に用意しています。C++ のビルドは不要です。

```bash
cd python
pip install -r requirements.txt   # numpy, matplotlib
python run_gui.py
```

1 つの車両パラメータを全タブで共有し、操縦シミュレーション（ステップ操舵／ランプ操舵／正弦波／定常円旋回／旋回制動／スラローム／ダブルレーンチェンジ／基準ルート追従）、アニメーション再生、ハンドリング解析、性能計測（加速・制動・限界横加速度・g-g 線図）、タイヤモデル比較、アッカーマン幾何を、モデルを並べて比較できます。

### 基準ルート走行アニメーション

![基準ルートを各モデルで走行](docs_en/images/route_demo.gif)

`data/reference_route.csv`（1010 m、最小半径 25 m）を全モデルに同じドライバで走らせ、経路追従誤差を並べて見せます。tkinter は不要です。

```bash
cd python
python demo_route.py                                  # 画面再生
python demo_route.py --save route.gif --rate 8        # 書き出し
python demo_route.py --models all --overview route.png
```

ドライバは全モデル共通で、横方向がルート上への投影に対する Pure Pursuit

$$
\delta = \arctan\frac{2 L \sin\eta}{L_d},
  \qquad L_d = \operatorname{clamp}(4 + 0.5\,v,\ 3,\ 18)\ [\mathrm{m}]
$$

前後方向が曲率 $\kappa$ から作った速度プロファイル

$$
v = \min\left(\sqrt{\frac{0.35\,\mu g}{\lvert \kappa \rvert}},\ v_{\max}\right)
$$

（これに前後加速度制限の逆算パスを掛けたもの）の PI 追従です。全モデルを**後軸基準で操舵**するため、状態量が重心のモデルも同じ点で比較できます。

乗用車プリセットで 1 km を走った結果（横偏差の最大値）:

| モデル | max &#124;e&#124; | rms e |
|---|---|---|
| キネマティック（重心） | 0.33 m | 0.08 m |
| 単輪ダイナミック（Fiala） | 0.79 m | 0.28 m |
| ダブルトラック（Fiala） | 0.80 m | 0.28 m |

キネマティックモデルはタイヤのスリップ角なしで即座に旋回するため、ダイナミックモデルの 2 倍以上正確に追従します。つまり「追従誤差が小さい」のはモデルが良いからではなく、誤差の原因を表現できていないからです。経路追従制御をキネマティックモデルだけで検証すると、この差分を見落とします。

移植の妥当性は C++ 単体テストと同じ検査 141 項目で検証済みです（`python tests/test_port.py`）。使い方は [docs_ja/python-gui.md](docs_ja/python-gui.md)（英語版: [docs_en/python-gui.md](docs_en/python-gui.md)）を参照してください。

---

## 設計上の注意

- **符号規約**：$\delta > 0$ が左旋回（ヨーレート正）。タイヤは $\alpha > 0$ で $F_y > 0$。スリップ角は

  $$
  \alpha_f = \delta - \arctan\frac{v_y + l_f r}{v_x},
    \qquad
    \alpha_r = -\arctan\frac{v_y - l_r r}{v_x}
  $$

- **低速特異点**：動力学モデルの $1/v_x$ は `guardDenominator` で下限を設けてあり、停止時も有限値を返します。ただし `low_speed_guard` を下回る領域の横運動は信用できません。`BlendedBicycleModel` を使ってください。
- **荷重移動**：ダブルトラックは1パス予測子（静荷重で $a_x, a_y$ を推定 → 荷重再計算 → 力再計算）。反復しないので実行時間が決定的です。
- **単位**：SI（m, s, rad, N, kg）。角度は例外なく rad で、`deg2rad` / `rad2deg` を用意しています。

詳細な運動方程式は [docs_ja/models.md](docs_ja/models.md) を参照してください。

## ドキュメント

| 文書 | 日本語 | English |
|---|---|---|
| 運動方程式 | [docs_ja/models.md](docs_ja/models.md) | [docs_en/models.md](docs_en/models.md) |
| Python GUI | [docs_ja/python-gui.md](docs_ja/python-gui.md) | [docs_en/python-gui.md](docs_en/python-gui.md) |
| Python API | [docs_ja/python-api.md](docs_ja/python-api.md) | [docs_en/python-api.md](docs_en/python-api.md) |
| 移植の検証 | [docs_ja/validation.md](docs_ja/validation.md) | [docs_en/validation.md](docs_en/validation.md) |
| 索引 | [docs_ja/README.md](docs_ja/README.md) | [docs_en/README.md](docs_en/README.md) |

数式はすべて LaTeX で記述してあり、GitHub 上でそのまま描画されます。

---

## ライセンス

Apache License 2.0
