# Python パッケージリファレンス

English edition: [`docs_en/python-api.md`](../docs_en/python-api.md)

`python/vehicle_models_py` は C++ ヘッダをそのまま移植したものに、GUI が必要と
するシミュレーション基盤を足したパッケージです。C++ のビルドには一切依存せず、
NumPy と（GUI 用に）matplotlib・tkinter があれば動きます。

```
python/
  vehicle_models_py/
    types.py             <- vehicle_models/types.hpp
    parameters.py        <- vehicle_models/vehicle_parameters.hpp
    tires.py             <- vehicle_models/tire/tire_models.hpp
    ackermann.py         <- vehicle_models/ackermann.hpp
    integrator.py        <- vehicle_models/integrator.hpp
    unicycle.py          <- vehicle_models/unicycle.hpp
    kinematic_bicycle.py <- vehicle_models/kinematic_bicycle.hpp
    dynamic_bicycle.py   <- vehicle_models/dynamic_bicycle.hpp
    double_track.py      <- vehicle_models/double_track.hpp
    linear_analysis.py   <- vehicle_models/linear_analysis.hpp
    maneuvers.py    ] Python 側にのみ存在する
    runner.py       ] シミュレーション基盤
    performance.py  ]
    gui/                 tkinter + matplotlib のフロントエンド
  tests/test_port.py     C++ 単体テストに対する検証
  tools/make_doc_figures.py
  run_gui.py
```

---

## 規約

**状態と入力は構造体ではなく1次元 NumPy 配列**です。各モジュールがインデックス
定数とファクトリ関数を公開します。

```python
from vehicle_models_py import dynamic_state, dynamic_input
from vehicle_models_py.dynamic_bicycle import X, Y, YAW, VX, VY, R

x = dynamic_state(0.0, 0.0, 0.0, 15.0, 0.0, 0.0)   # [x, y, yaw, vx, vy, r]
u = dynamic_input(fx=0.0, steer=0.05)               # [F_x, delta]
print(x[VX], x[R])
```

**モデルはパラメータを持つ dataclass** で、C++ の積分器が前提とする暗黙の
インタフェースと同じものを満たします。

```python
derivative(state, input) -> ndarray
normalize_state(state)   -> ndarray     # 角度の正規化・クランプ。in-place で書き換える
```

この2つのメソッドさえあれば `step()` と `simulate()` で動くので、自作モデルも
そのまま使えます。

**単位は C++ と同じ SI**（m, s, rad, N, kg）。`deg2rad` / `rad2deg` は `types.py`
にあります。$\delta > 0$ が左旋回で、ヨーレートは正になります。

---

## クイックスタート

```python
from vehicle_models_py import (KinematicBicycleModel, ReferencePoint,
                               DynamicBicycleModel, DoubleTrackModel,
                               FialaTire, IntegratorType, deg2rad,
                               dynamic_input, dynamic_state, kinematic_input,
                               kinematic_state, make_passenger_car_parameters,
                               simulate, step)
from vehicle_models_py import linear_analysis as analysis

p = make_passenger_car_parameters()
print(p.validate())                       # [] なら妥当なパラメータセット

# キネマティック二輪、後軸基準、10 ms 周期
model = KinematicBicycleModel(p, ReferencePoint.REAR_AXLE)
x = kinematic_state(0.0, 0.0, 0.0, 4.0)
x = step(model, x, kinematic_input(0.0, deg2rad(8.0)), 0.01)

# 非線形単輪（Fiala タイヤ）
dyn = DynamicBicycleModel(p, FialaTire(), FialaTire())
s = dynamic_state(0, 0, 0, 15.0, 0, 0)
s = step(dyn, s, dyn.input_from_acceleration(0.0, deg2rad(3.0)), 0.01)
f = dyn.compute_forces(s, dynamic_input(0.0, deg2rad(3.0)))
print(f.slip_front, f.fy_front, f.fz_front, f.ay)

# 4輪
dt_model = DoubleTrackModel(p, tire_front=FialaTire(), tire_rear=FialaTire())
forces = dt_model.compute_forces(s, dynamic_input(0.0, deg2rad(3.0)))
print(forces.normal_load, forces.slip_angle, forces.steer.left)

# ハンドリング解析（解析解）
print(analysis.understeer_gradient_deg_per_g(p))
ss = analysis.steady_state_cornering(p, 15.0, deg2rad(2.0))
print(ss.yaw_rate, ss.side_slip, ss.radius)

# 固定入力の一括積分
x_end = simulate(dyn, s, dynamic_input(0.0, deg2rad(2.0)), 5.0, 0.001,
                 IntegratorType.RK4)
```

---

## モジュール

### `types.py`
`PI`、`GRAVITY`、`deg2rad`、`rad2deg`、`normalize_angle`、`clamp_value`、
`signum`、`guard_denominator`、`Pose2D`。

### `parameters.py`
`VehicleParameters`（C++ 構造体と同じフィールド・同じ既定値の dataclass）、
`wheel_base()`、`static_load_front()`、`static_load_rear()`、`validate()`、
`copy()`。プリセットは `make_passenger_car_parameters`、
`make_shuttle_parameters`、`make_buggy_parameters`、
`make_oversteer_car_parameters` と、GUI が使う `PRESETS` 辞書。

静荷重は

$$
F_{zf} = \frac{m g l_r}{L},
\qquad
F_{zr} = \frac{m g l_f}{L}
$$

### `tires.py`
`LinearTire`、`FialaTire`、`PacejkaTire` — いずれも
`lateral_force(slip_angle, normal_force)` と
`cornering_stiffness_at(normal_force)` を持ちます。加えて
`PacejkaTire.from_cornering_stiffness()`、`friction_ellipse_scale()`、
`make_tire(kind, cornering_stiffness, nominal_load, friction)`。最後のものは
3種類のいずれも同じスティフネス・同じピーク力に合わせて生成します。

摩擦楕円のスケーリングは

$$
F_y \leftarrow F_y \sqrt{1 - \left(\frac{F_x}{\mu F_z}\right)^2}
$$

### `ackermann.py`
`AckermannGeometry`（`from_params()`）、`WheelAngles`、`WheelSpeeds`、
`turn_radius`、`steer_angle_for_radius`、`road_wheel_angles`、
`bicycle_angle_from_wheels`、`handwheel_to_road_wheel`、
`road_wheel_to_handwheel`、`ackermann_error`、`wheel_speeds`、
`wheel_angular_rates`、`minimum_turn_radius`。

$$
R = \frac{L}{\tan\delta},
\qquad
\delta = \arctan\frac{L}{R}
$$

### `integrator.py`
`IntegratorType`（`EULER`、`HEUN`、`RK4`）、`step_euler`、`step_heun`、
`step_rk4`、`step`、`simulate`。RK4 は

$$
x_{k+1} = x_k + \frac{h}{6}\left(k_1 + 2k_2 + 2k_3 + k_4\right)
$$

### `unicycle.py`
`UnicycleModel`、`DifferentialDriveModel`、`DifferentialDriveParams`、および
状態／入力ファクトリ `unicycle_state`、`unicycle_input`、`wheel_rate_input`。

### `kinematic_bicycle.py`
`ReferencePoint`（`REAR_AXLE`、`CENTER_OF_GRAVITY`、`FRONT_AXLE`）、
`KinematicBicycleModel`（`side_slip()`、`lateral_acceleration()`）、
`KinematicBicycleSteerModel`、および `kinematic_state`、`kinematic_input`、
`steer_dynamics_state`。

$$
\beta = \arctan\frac{l_r \tan\delta}{L},
\qquad
a_y = \frac{v^2 \tan\delta}{L}
$$

### `dynamic_bicycle.py`
`DynamicBicycleModel`（`compute_forces()` → `BicycleForces`、
`input_from_acceleration()`、`sync_tires_from_params()`）、
`LinearLateralBicycleModel`（`state_matrix()` / `input_matrix()` が $A$, $B$ を
NumPy 配列で返す）、`BlendedBicycleModel`（`blend_factor()`）、および
`dynamic_state`、`dynamic_input`、`lateral_state`、`steer_input`、`speed_of`、
`side_slip_of`。

### `double_track.py`
`DoubleTrackModel`、`DoubleTrackParams`、`DoubleTrackForces`、車輪インデックス
`FL, FR, RL, RR`。輪別の量は4要素の素の list で、上記定数で添字アクセスします。
`DoubleTrackForces.load_sum()` が C++ の `WheelQuantities::sum()` に対応し、
恒等式

$$
\sum_{i \in \{fl, fr, rl, rr\}} F_{z,i} = m g
$$

が単体テストで検査されます。

### `linear_analysis.py`
`understeer_gradient`、`understeer_gradient_deg_per_g`、`characteristic_speed`、
`critical_speed`、`neutral_steer_point`、`static_margin`、`yaw_rate_gain`、
`lateral_acceleration_gain`、`required_steer_angle`、`steady_state_cornering`
→ `SteadyState`、`yaw_mode` → `YawMode`、`max_lateral_acceleration`。

$$
K = \frac{m}{L}\left(\frac{l_r}{C_f} - \frac{l_f}{C_r}\right),
\qquad
\frac{r}{\delta} = \frac{v}{L + K v^2},
\qquad
V_{ch} = \sqrt{\frac{L}{K}},
\qquad
V_{cr} = \sqrt{-\frac{L}{K}}
$$

---

## シミュレーション基盤（Python のみ）

### `maneuvers.py`
`ManeuverConfig` が1回の試験走行の設定を保持し、`Maneuver` がそれをラップして
`command(t, pose, v) -> (steer, ax_override)` に答えます。`ax_override = None` の
とき前後方向はランナ側の速度コントローラに委ねられます。開ループのプロファイル
としてステップ、ランプ、正弦波、正弦波＋ドウェル、定常円旋回、旋回制動、直線
走行を用意し、`PurePursuitDriver` がスラロームとダブルレーンチェンジの基準経路を
追従します（基準経路は `reference_path()` で取得でき、プロット用に使えます）。
`RouteDriver` は `Route`（種別 `ROUTE`）を追従します。折れ線の弧長に対する Pure
Pursuit と、ルートの速度プロファイルに対する PI に、コーナー手前で減速を始める
プレビュー走査を組み合わせたものです。

### `route.py`
基準ルートを折れ線として扱うモジュールです。$y = f(x)$ の曲線ではないので、経路が
任意の角度に曲がっていても構いません。

```python
from vehicle_models_py.route import (analyse_tracking, load_route,
                                     speed_profile, travel_time)
from vehicle_models_py.maneuvers import ROUTE, ManeuverConfig
from vehicle_models_py.runner import rear_axle_track, run_maneuver

route = load_route()                       # data/reference_route.csv
profile = speed_profile(route, p, ay_ratio=0.35)   # ルート各点の [m/s]
cfg = ManeuverConfig(kind=ROUTE, dt=0.005, route=route, route_speed=profile,
                     duration=travel_time(route, profile) * 1.2 + 3.0,
                     initial_speed=float(profile[0]))

results = run_maneuver(p, cfg, ["kin_cog", "dynamic", "double_track"], "Fiala")
for res in results:
    report = analyse_tracking(route, *rear_axle_track(res, p),
                              time=res.time, profile=profile)
    print(res.label, report.summary["lateral_max"], report.summary["finish_time"])
```

- `load_route(path)` はヘッダ名（`x_m`、`y_m`、`yaw`、`curvature` とその別名）で
  列を探すので、余分な列があっても列順が違っても読めます。ファイルに `yaw` と
  `curvature` がなければ幾何から数値微分します。弧長は `Route` が自前で求めます。
- `Route.project(x, y, hint)` はルート上の最近傍点を `Projection`（`index`、
  `s`、符号付き `lateral`、`heading`、`curvature`）で返します。`hint` により探索が
  局所化されるため、自分自身に接近するようなルートでも正しく動きます。
- `speed_profile(route, params, ...)` は各点の速度を曲率で制限し、

  $$
  v_i \le \min\left(\sqrt{\frac{a_{y,\max}}{\lvert \kappa_i \rvert}},\ v_{\max}\right),
    \qquad a_{y,\max} = \eta\,\mu g
  $$

  （係数 $\eta$ が `ay_ratio`、既定 0.35）、続いて `accel_min` による後ろ向き
  パスと `accel_max` による前向きパスを掛けて、全区間で

  $$
  \left\lvert \frac{v_{i+1}^2 - v_i^2}{2\,\Delta s_i} \right\rvert
    \le a_{x,\max}
  $$

  が成り立つようにします。これによりコーナー手前から減速が始まり、かつ車両が実際に
  実現できるプロファイルになります。
- `analyse_tracking(route, x, y, yaw, ...)` はサンプルごとの横偏差・方位誤差・
  進捗を含む `TrackingReport` を返します。渡す軌跡は重心ではなく後軸のもの
  （`runner.rear_axle_track`）にしてください。重心を渡すと車体スリップ角が追従
  誤差として計上されてしまいます。

### `route_animation.py`
`RouteScene` / `build_route_animation` が、ルート走行を全モデル1枚の図で
アニメーション化します（集団を追うカメラ、ミニマップ、横偏差・速度・横加速度の
チャネル）。`route_overview` は同じ走行を静止画にしたものです。コマンドライン
フロントエンドは `python demo_route.py`。

### `runner.py`
状態レイアウトと入力の違いを1つのインタフェースの裏に隠すアダプタ群です。同じ
マヌーバを全モデルに流し込めます。

```python
from vehicle_models_py.maneuvers import ManeuverConfig, STEP_STEER
from vehicle_models_py.runner import run_maneuver, to_csv
from vehicle_models_py.parameters import make_passenger_car_parameters
from vehicle_models_py.types import deg2rad

p = make_passenger_car_parameters()
cfg = ManeuverConfig(kind=STEP_STEER, duration=8.0, dt=0.002,
                     initial_speed=20.0, steer_amplitude=deg2rad(3.0),
                     t_start=1.0, hold_speed=True)

results = run_maneuver(p, cfg, ["kin_cog", "dynamic", "double_track"],
                       tire_kind="Fiala")
for res in results:
    print(res.label, res.summary["r_final"], res.summary["ay_peak"])
    print(res["r"][-1], res["beta"][-1])      # 各チャネルは NumPy 配列

open("run.csv", "w").write(to_csv(results))
```

`MODEL_CATALOG` が利用可能なモデルキー（`kin_rear`、`kin_cog`、`kin_steer`、
`linear2dof`、`dynamic`、`blended`、`double_track`）を列挙し、`STATE_REFERENCE`
が各モデルの $(x, y)$ が車両のどの点かを示します。`rear_axle_track(result,
params)` は走行結果を後軸基準に変換するので、$l_r$ のオフセットが混入せずに
モデル同士を比較できます。どの走行も同じチャネル集合
（`x, y, yaw, vx, vy, v, r, beta, ax, ay, steer_cmd, steer, alpha_f, alpha_r,
curvature, fz_fl…fz_rr, fy_f, fy_r, steer_l, steer_r`）を埋め、モデルが出せない
量は `NaN` になります。欠測チャネルをプロットしても、誤解を招くゼロではなく何も
描かれません。

### `performance.py`
`acceleration_run`、`braking_run`、`ramp_steer_run`（ハンドリング線図と実測の
アンダーステア勾配）、`gg_diagram`、`max_cornering_speed`、`steady_state_table`。
時間のかかるものには進捗コールバックがあります。

ハンドリング線図はランプ操舵から

$$
\delta - \frac{L}{R} = K a_y + \mathcal{O}(a_y^2)
$$

の関係をプロットしたもので、原点付近の傾きがそのままアンダーステア勾配 $K$ です。

---

## C++ API との意図的な差異

| C++ | Python | 理由 |
|---|---|---|
| 名前付きアクセサを持つ `StateVector<Derived, N>` 構造体 | 1次元 `numpy.ndarray` ＋インデックス定数 | 積分器が1行で書け、ログ・プロットが自然になる |
| テンプレート `DynamicBicycleModel<Tire>` | `DynamicBicycleModel(params, tire_front, tire_rear)` | Python にテンプレートはない。タイヤは普通のオブジェクト |
| `setTireStiffness` のオーバーロード群 | `_set_tire_stiffness` 内の `isinstance` ディスパッチ | 効果は同じで、解決が実行時になるだけ |
| `tire.corneringStiffness(fz)` | `tire.cornering_stiffness_at(fz)` | `cornering_stiffness` はデータフィールド名として既に使用済み |
| `WheelQuantities::sum()` | `DoubleTrackForces.load_sum()` | 輪別の量が素の list のため |
| `syncTiresFromParams()` はスティフネスのみコピー | `sync_tires_from_params(sync_friction=False)` | C++ と同じ既定値＋オプトイン（下記参照） |
| camelCase | snake_case | PEP 8 |
| プリセット3種 | ＋ `make_oversteer_car_parameters()` | 限界速度が有限になる例はハンドリング解析で最も分かりやすい |

### 摩擦係数の伝播ギャップ

C++ の `DynamicBicycleModel::syncTiresFromParams()` は
`cornering_stiffness_front/rear` をタイヤへコピーしますが `params.friction` は
コピーしません。そのため $\mu = 0.7$ のプリセット（バギー）から作った単輪モデル
でも、タイヤ側の $\mu$ は 1.0 のままで、飽和条件

$$
\lvert F_y \rvert \le \mu F_z
$$

が誤った $\mu$ で評価されます。`DoubleTrackModel` は伝播させます。Python 移植は
既定でこの C++ の挙動を再現しつつ、`sync_tires_from_params(sync_friction=True)`
を追加しています。GUI と `performance.py` はこのオプトインを使うため、画面上の
全モデルが同じ路面を共有します。低摩擦車両で Python と C++ の結果を比較する際は、
両者がこの点で揃っているか確認してください。

---

## 独自モデルの追加

2つのメソッドを用意すれば既存の積分器で動き、アダプタを足せばランナからも使え
ます。

```python
import numpy as np
from dataclasses import dataclass
from vehicle_models_py.types import normalize_angle
from vehicle_models_py import simulate

@dataclass
class MyModel:
    gain: float = 1.0
    n_states = 3

    def derivative(self, s, u):
        return np.array([u[0] * np.cos(s[2]), u[0] * np.sin(s[2]),
                         self.gain * u[1]])

    def normalize_state(self, s):
        s[2] = normalize_angle(s[2])
        return s

x = simulate(MyModel(), np.zeros(3), np.array([2.0, 0.3]), 5.0, 0.01)
```

GUI に載せる場合は `runner.ModelAdapter` を継承して（`reset`、`sample`、
`advance`、`speed` を実装）、`runner.MODEL_CATALOG` に `ModelOption` を追加し、
`gui/theme.py:MODEL_COLORS` で色を割り当ててください。
