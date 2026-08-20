# Python バインディングの検証

English edition: [`docs_en/validation.md`](../docs_en/validation.md)

Python 側は C++ のモデルを再実装したものではなく、`_core`（pybind11）経由で
`include/vehicle_models/*.hpp` をそのまま呼びます。したがって「両実装が一致して
いるか」を問う必要はもうありません。検証すべきは**バインディングが状態量の並び・
引数の順序・単位を取り違えていないか**であり、以下の3つがそれを担保します。

---

## 1. C++ 単体テストを Python で再実行

[`python/tests/test_port.py`](../python/tests/test_port.py) は
`test/test_kinematics.cpp` / `test/test_dynamics.cpp` / `test/test_integrator.cpp`
を1対1で写したものです。車両も操縦も許容誤差もすべて同じです。

```bash
cd python
python tests/test_port.py
```

```
checks run : 141
all checks passed
```

検査項目：

| 区分 | 検査内容 |
|---|---|
| ユニサイクル | $2\pi/\omega$ 後に円が閉じること、直進距離、その場旋回 |
| 差動二輪 | 車輪角速度 ↔ 車体速度の往復変換、純回転、走行距離 |
| アッカーマン幾何 | 理想条件 $\cot\delta_o - \cot\delta_i = T/L$、左右対称性、半径・ハンドル角の往復変換、最小回転半径 |
| 車輪速 | 後輪の平均が車体速度に一致、外輪の方が速い、角速度換算 |
| キネマティック二輪 | ヨーレート $v\tan\delta/L$、円軌道が閉じること、3つの基準点の等価性、加速と上限クランプ |
| 操舵アクチュエータ | ステップ直後のレート制限、指令値への収束、機械的リミット |
| タイヤモデル | 線形の傾きと飽和、Fiala の傾き／単調性／すべり域の値、Pacejka の $BCD$ とピーク、摩擦楕円（0 / 0.6 / 1.0） |
| 線形ハンドリング解析 | 特性速度でゲインがニュートラル値の半分、 $\delta = L/R + K a_y$、アンダーステア車が全速度域で安定、オーバーステア車が限界速度以上で不安定 |
| 線形モデル vs 解析解 | 20 s のシミュレーションが `steadyStateCornering()` に $10^{-6}$ 以内で一致 |
| ダイナミック二輪 | 定常値が 2 % 以内、直進が平衡点、指令加速度、荷重移動の総和が $mg$、停止時も微分が有限、 $\lvert F_y \rvert \le \mu F_z$ |
| ブレンド | 両端と中間でのブレンド係数、極低速でキネマティック予測に一致、ブレンド速度以上で純ダイナミックと一致（ $10^{-12}$） |
| ダブルトラック | 静荷重配分、制動時・旋回時の荷重移動、荷重総和が常に $mg$、内輪の方が大きく切れる、限界以下で単輪モデルと一致（5 %）、 $a_y$ が有界、複合スリップで横力が減る |
| パラメータ検証 | 3つのプリセットが妥当、意図的に壊した設定がちょうど3件の違反を報告 |
| 積分器の次数 | 四分円軌道で Euler / Heun / RK4 の誤差比が 2 / 4 / 16 |
| runner / performance | ダイナミックモデルが解析解の定常値に到達、キネマティックが過大評価、速度コントローラが動作点を保持、制動距離と限界横加速度が物理的に妥当 |

収束次数の検査は刻み幅を半分にしたときの誤差比

$$
\frac{\lVert e(h) \rVert}{\lVert e(h/2) \rVert} \approx 2^{p},
\qquad p = 1,\ 2,\ 4
$$

（それぞれ Euler / Heun / RK4）で行っています。この検査は積分器が C++ 側で回って
いることの確認でもあります。刻み幅を変えたときの次数が出るのは、`_core` が
`integrator.hpp` のテンプレートを型消去したモデルに対して実体化しているためで、
Python 側に積分器のコピーはありません。

---

## 2. C++ ドキュメント記載値の再現

C++ の `README.md` に記載した数値が、Python から呼んでもそのまま出ます。

| 量 | C++ README | Python |
|---|---|---|
| アンダーステア勾配（乗用車） | `+0.0034 rad/(m/s²)` / `+1.917 deg/g` | `+0.0034` / `+1.917 deg/g` |
| スタティックマージン | `+0.1056` | `+0.1056` |
| 特性速度 | `28.13 m/s (101.3 km/h)` | `28.13 m/s (101.3 km/h)` |
| 20 m/s・3 deg ステップ操舵 — キネマティックのヨーレート | `0.388 rad/s` | `0.388206 rad/s` |
| 20 m/s・3 deg ステップ操舵 — 単輪のヨーレート | `0.248 rad/s` | `0.247893 rad/s` |

再現手順：

```bash
cd python
python -c "
from vehicle_models_py import make_passenger_car_parameters as car
from vehicle_models_py import linear_analysis as a
p = car()
print('%+.3f deg/g' % a.understeer_gradient_deg_per_g(p))
print('%+.4f' % a.static_margin(p))
print('%.2f m/s' % a.characteristic_speed(p))
"
```

---

## 3. コンパイル済み実行例との直接比較

[`python/tools/compare_with_cpp.py`](../python/tools/compare_with_cpp.py) は
ビルド済みの `step_steer` を実行し、同一の実験を Python 側から再現して、全チャネルの
全サンプルを後述の許容値と照合し、境界に最も近いサンプルを報告します。

```bash
cmake -S . -B build -DVEHICLE_MODELS_BUILD_EXAMPLES=ON -DVEHICLE_MODELS_BUILD_PYTHON=ON
cmake --build build --config Release
cd python
python tools/compare_with_cpp.py ../build/Release/step_steer
```

```
(at the sample closest to its own bound)
channel                |diff|      tolerance    verdict
t                   0.000e+00      1.000e-12         ok
steer_deg           0.000e+00      1.000e-12         ok
r_kin               0.000e+00      1.000e-12         ok
r_dyn               0.000e+00      1.000e-12         ok
r_dtr               0.000e+00      1.000e-12         ok
beta_dyn_deg        0.000e+00      1.000e-12         ok
ay_dyn              0.000e+00      1.000e-12         ok
ay_dtr              0.000e+00      1.000e-12         ok
```

**許容値の根拠。** 両者は同じ C++ コードを実行するので、比較が見うる差は CSV が
持ち込むものだけです。`examples/step_steer.cpp` は `std::setprecision(17)` で
出力します。17 桁は `double` が厳密に往復する最短の精度なので、ファイルは計算値を
そのまま運び、量子化を一切加えません。残るのは 2 つの呼び出し経路そのものの一致度
であり、検査は `math.isclose` と同じ形でその境界を定めます。

$$
\bigl\lvert y^{\text{C++}}_k - y^{\text{Py}}_k \bigr\rvert
\le \max\bigl(\varepsilon_{\text{rel}} \max(\lvert y^{\text{C++}}_k \rvert,
\lvert y^{\text{Py}}_k \rvert),\ \varepsilon_{\text{abs}}\bigr),
\qquad
\varepsilon_{\text{rel}} = 10^{-9},\quad \varepsilon_{\text{abs}} = 10^{-12}
$$

全チャネルがビット単位で一致するため、報告される差はちょうど 0 で、境界には
まったく近づきません。`tolerance` 列は境界に最も近いサンプルでの値を示しますが、
完全一致の場合それは先頭行であり、そこでは全チャネルが 0 なので
$`\varepsilon_{\text{abs}}`$ が効いています。

> 以前の版は CSV を `%.4f` と `%.6f` で出力しながら一律 $10^{-9}$ を課していました。
> 丸められた出力では、実装がどれだけ一致していても満たせない条件です。まず許容値を
> 印字桁の量子化幅の半分へ緩めて検査を成立させ、次に出力精度そのものを上げることで
> 量子化を取り除き、厳しい境界を維持できるようにしました。

---

## 移行時の等価性確認

C++ バインディングへ移行した際、旧 Python 実装との数値的な等価性を直接測定して
います。4つのプリセット × 7つのモデル × 3つのタイヤモデル = 84通りについて、
`runner` の全23チャネルを 400 ステップ分ずつ比較したところ、**すべて $10^{-9}$
以内で一致**しました。`docs_en/images/` の図も、再生成した結果がコミット済みの
ものとバイト単位で同一です。

移行で1件だけ挙動の違いが見つかり、修正済みです。pybind11 の enum は呼び出しご
とに新しいオブジェクトを返すため、`runner.py` が `ReferencePoint` を `is` で
比較していた箇所が常に偽になり、重心基準のキネマティックモデルが後軸基準として
評価されていました。`==` に修正してあります。同種の比較を書くときは注意して
ください。

---

## 数値上の注意

- 状態量は Python 側で 1 次元 `numpy` 配列として扱い、境界で C++ の固定長
  `StateVector` に詰め替えます。この変換は値のコピーだけで、丸めは起きません。
- マヌーバランナは積分*前*に出力をサンプリングします。したがって時刻 $t$ の
  ログは $t + \Delta t$ ではなく $t$ の状態です。C++ の `step_steer` も同じです。
- `simulate()` は指定した継続時間ぴったりで終わるよう最終ステップを短縮します
  （ $N = \lfloor T/\Delta t \rfloor$ 回のフルステップの後、最後は
  $T - N\Delta t$）。C++ の実装をそのまま呼んでいます。
- RK4 の中間段 $k_1 \dots k_4$ は `normalizeState()` を通しません。正規化される
  のは各ステップの最終状態だけです。
